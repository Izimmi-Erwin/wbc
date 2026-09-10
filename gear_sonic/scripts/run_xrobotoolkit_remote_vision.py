#!/usr/bin/env python3
"""Send Isaac's existing ``ego_view`` camera stream to PICO Remote Vision.

This is deliberately a display-only side branch:

    Isaac image publisher :5555 -> this process -> PICO Remote Vision :12345

It neither imports Isaac Sim nor opens OpenXR, and it does not publish on any
Sonic/WBC port.  The PICO must already be running XRoboToolkit Remote Vision
with ``PICO4U`` selected and ``Listen`` pressed before this program starts.

The Remote Vision receiver protocol was recovered and runtime-verified from
the installed XRoboToolkit 1.1.1 application:

    uint32_be h264_access_unit_size
    uint8[h264_access_unit_size] h264_access_unit

GStreamer emits Annex-B access units with an AUD NAL (``00 00 00 01 09``).
The reader below groups that byte stream at each AUD, then emits one verified
length-prefixed Remote Vision frame for every complete access unit.

Run with Isaac Sim's Python environment, which already contains cv2, zmq,
msgpack, and numpy::

    /home/colin/Erwin/isaac-sim-4.5.0/python.sh \\
      gear_sonic/scripts/run_xrobotoolkit_remote_vision.py
"""

from __future__ import annotations

import argparse
import base64
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import BinaryIO

import cv2
import msgpack
import numpy as np
import zmq


AUD_START_CODE = b"\x00\x00\x00\x01\x09"


@dataclass(frozen=True)
class Config:
    camera_host: str
    camera_port: int
    camera_name: str
    headset_host: str
    stream_port: int
    width: int
    height: int
    stereo_layout: str
    fps: float
    bitrate_kbps: int
    socket_timeout_seconds: float


class H264AccessUnitReader:
    """Split x264's AUD-delimited Annex-B output into complete access units."""

    def __init__(self, stream: BinaryIO) -> None:
        self._stream = stream
        self._buffer = bytearray()

    def read_access_units(self):
        while True:
            chunk = self._stream.read(64 * 1024)
            if not chunk:
                return
            self._buffer.extend(chunk)

            while True:
                first = self._buffer.find(AUD_START_CODE)
                if first < 0:
                    # x264 writes SPS/PPS before the first AUD.  They are
                    # decoder configuration, not disposable preamble: keep
                    # them until a complete first access unit arrives.
                    break
                boundary = self._buffer.find(AUD_START_CODE, first + len(AUD_START_CODE))
                if boundary < 0:
                    break
                access_unit = bytes(self._buffer[:boundary])
                del self._buffer[:boundary]
                if access_unit:
                    yield access_unit


class Encoder:
    """Persistent raw-RGB -> x264 subprocess with an access-unit queue."""

    def __init__(self, config: Config) -> None:
        self._config = config
        self._process: subprocess.Popen[bytes] | None = None
        self._access_units: queue.Queue[bytes] = queue.Queue(maxsize=3)
        self._reader_error: BaseException | None = None
        self._reader_thread: threading.Thread | None = None

    def start(self) -> None:
        command = [
            "gst-launch-1.0",
            "-q",
            "fdsrc",
            "fd=0",
            "!",
            "rawvideoparse",
            "format=rgb",
            f"width={self._config.width}",
            f"height={self._config.height}",
            f"framerate={int(round(self._config.fps))}/1",
            "!",
            "videoconvert",
            "!",
            "video/x-raw,format=I420",
            "!",
            "x264enc",
            "tune=zerolatency",
            "speed-preset=ultrafast",
            f"bitrate={self._config.bitrate_kbps}",
            f"key-int-max={max(1, int(round(self._config.fps)))}",
            "bframes=0",
            "byte-stream=true",
            "aud=true",
            # The Remote Vision decoder receives complete Annex-B access
            # units, not an out-of-band codec-data record.  Put SPS/PPS in
            # the first IDR access unit (and every following IDR) so a new
            # PICO TCP connection can configure MediaCodec immediately.
            "option-string=repeat-headers=1",
            "!",
            "fdsink",
            "fd=1",
            "sync=false",
        ]
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self._reader_thread = threading.Thread(target=self._read_access_units, daemon=True)
        self._reader_thread.start()
        threading.Thread(target=self._relay_stderr, daemon=True).start()

    def _read_access_units(self) -> None:
        assert self._process is not None and self._process.stdout is not None
        try:
            for access_unit in H264AccessUnitReader(self._process.stdout).read_access_units():
                try:
                    self._access_units.put_nowait(access_unit)
                except queue.Full:
                    try:
                        self._access_units.get_nowait()
                    except queue.Empty:
                        pass
                    self._access_units.put_nowait(access_unit)
        except BaseException as exc:  # preserve error for main loop diagnostics
            self._reader_error = exc

    def _relay_stderr(self) -> None:
        assert self._process is not None and self._process.stderr is not None
        for line in iter(self._process.stderr.readline, b""):
            text = line.decode("utf-8", errors="replace").rstrip()
            if text:
                print(f"[RemoteVision][gstreamer] {text}", file=sys.stderr, flush=True)

    def write_rgb(self, rgb: np.ndarray) -> None:
        if self._reader_error is not None:
            raise RuntimeError("H.264 access-unit reader failed") from self._reader_error
        if self._process is None or self._process.stdin is None:
            raise RuntimeError("encoder is not running")
        if self._process.poll() is not None:
            raise RuntimeError(f"encoder exited with code {self._process.returncode}")
        self._process.stdin.write(rgb.tobytes(order="C"))

    def get_access_unit_nowait(self) -> bytes | None:
        try:
            return self._access_units.get_nowait()
        except queue.Empty:
            return None

    def close(self) -> None:
        if self._process is None:
            return
        if self._process.stdin is not None:
            try:
                self._process.stdin.close()
            except OSError:
                pass
        try:
            self._process.terminate()
            self._process.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self._process = None


def parse_args() -> Config:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-host", default="localhost")
    parser.add_argument("--camera-port", type=int, default=5555)
    parser.add_argument("--camera-name", default="ego_view")
    parser.add_argument(
        "--headset-host",
        required=True,
        help="Current PICO Remote Vision IP; USB shared-network addresses may change.",
    )
    parser.add_argument("--stream-port", type=int, default=12345)
    parser.add_argument("--width", type=int, default=2160)
    parser.add_argument("--height", type=int, default=810)
    parser.add_argument(
        "--stereo-layout",
        choices=("sbs", "mono"),
        default="sbs",
        help=(
            "PICO4U frame layout. 'sbs' duplicates ego_view into independent "
            "left/right 1080x810 eye images; 'mono' is the legacy full-frame mode."
        ),
    )
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--bitrate-kbps", type=int, default=8000)
    parser.add_argument("--socket-timeout-seconds", type=float, default=5.0)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0 or args.bitrate_kbps <= 0:
        parser.error("width, height, fps, and bitrate-kbps must be positive")
    if args.stereo_layout == "sbs" and args.width % 2:
        parser.error("SBS output requires an even width")
    return Config(**vars(args))


def letterbox_rgb(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Scale RGB input without distortion and center it in a black PICO frame."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected RGB HxWx3 image, got shape {image.shape}")
    src_h, src_w = image.shape[:2]
    scale = min(width / src_w, height / src_h)
    resized_w, resized_h = max(1, round(src_w * scale)), max(1, round(src_h * scale))
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    result = np.zeros((height, width, 3), dtype=np.uint8)
    x0 = (width - resized_w) // 2
    y0 = (height - resized_h) // 2
    result[y0 : y0 + resized_h, x0 : x0 + resized_w] = resized
    return result


def format_remote_vision_frame(image: np.ndarray, config: Config) -> np.ndarray:
    """Build the PICO4U frame from Isaac's mono ego camera.

    PICO4U treats its 2160x810 input as side-by-side stereo.  Until Isaac
    publishes separate left/right cameras, duplicate the same correctly fitted
    eye image into both halves.  This produces binocularly aligned mono rather
    than fake depth and avoids each eye seeing only half of a full-width image.
    """
    if config.stereo_layout == "mono":
        return letterbox_rgb(image, config.width, config.height)
    eye = letterbox_rgb(image, config.width // 2, config.height)
    return np.concatenate((eye, eye), axis=1)


def decode_ego_view(packed: bytes, camera_name: str) -> np.ndarray | None:
    message = msgpack.unpackb(packed, raw=False)
    encoded = message.get("images", {}).get(camera_name)
    if not encoded:
        return None
    if isinstance(encoded, str):
        encoded = base64.b64decode(encoded)
    elif isinstance(encoded, bytearray):
        encoded = bytes(encoded)
    if not isinstance(encoded, bytes):
        raise TypeError(f"camera payload has unsupported type {type(encoded).__name__}")
    # Isaac's sensor protocol stores RGB numerical channels in JPEG.  imdecode
    # preserves those numerical channel values, so feed it to rawvideoparse as
    # RGB rather than performing an additional BGR conversion.
    rgb = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if rgb is None:
        raise ValueError("cv2.imdecode failed for ego_view JPEG")
    return rgb


def connect_receiver(config: Config) -> socket.socket:
    receiver = socket.create_connection(
        (config.headset_host, config.stream_port), timeout=config.socket_timeout_seconds
    )
    receiver.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    receiver.settimeout(config.socket_timeout_seconds)
    print(
        f"[RemoteVision] connected to PICO receiver tcp://{config.headset_host}:{config.stream_port}",
        flush=True,
    )
    return receiver


def main() -> int:
    config = parse_args()
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    subscriber.setsockopt(zmq.CONFLATE, 1)
    subscriber.setsockopt(zmq.RCVHWM, 1)
    subscriber.setsockopt(zmq.LINGER, 0)
    subscriber.connect(f"tcp://{config.camera_host}:{config.camera_port}")

    encoder = Encoder(config)
    receiver: socket.socket | None = None
    sent_frames = 0
    received_images = 0
    next_input_time = 0.0
    last_report = time.monotonic()
    report_sent_frames = 0
    report_received_images = 0
    try:
        print(
            "[RemoteVision] waiting for Isaac image frames on "
            f"tcp://{config.camera_host}:{config.camera_port}; PICO Remote Vision must already be Listening",
            flush=True,
        )
        receiver = connect_receiver(config)
        encoder.start()
        poller = zmq.Poller()
        poller.register(subscriber, zmq.POLLIN)
        while True:
            events = dict(poller.poll(timeout=20))
            if subscriber in events:
                packed = subscriber.recv()
                image = decode_ego_view(packed, config.camera_name)
                if image is not None:
                    received_images += 1
                    now = time.monotonic()
                    if now >= next_input_time:
                        encoder.write_rgb(format_remote_vision_frame(image, config))
                        next_input_time = now + 1.0 / config.fps

            while (access_unit := encoder.get_access_unit_nowait()) is not None:
                if len(access_unit) > 10 * 1024 * 1024:
                    raise RuntimeError(f"refusing oversized H.264 access unit: {len(access_unit)} bytes")
                assert receiver is not None
                receiver.sendall(struct.pack(">I", len(access_unit)))
                receiver.sendall(access_unit)
                sent_frames += 1

            now = time.monotonic()
            if now - last_report >= 2.0:
                elapsed = now - last_report
                print(
                    "[RemoteVision] "
                    f"input={received_images - report_received_images}/{elapsed:.1f}s "
                    f"sent={sent_frames - report_sent_frames}/{elapsed:.1f}s total_sent={sent_frames}",
                    flush=True,
                )
                last_report = now
                report_sent_frames = sent_frames
                report_received_images = received_images
    except KeyboardInterrupt:
        print("[RemoteVision] stopped by operator", flush=True)
        return 0
    except (ConnectionError, OSError) as exc:
        print(
            "[RemoteVision] PICO receiver disconnected. Press Listen again in Remote Vision, "
            f"then restart this bridge: {exc}",
            file=sys.stderr,
            flush=True,
        )
        return 2
    finally:
        if receiver is not None:
            receiver.close()
        encoder.close()
        subscriber.close(0)
        context.term()


if __name__ == "__main__":
    raise SystemExit(main())
