"""Local, acknowledged episode boundaries between Isaac and the PICO manager."""

import time
import uuid

import zmq


class EpisodeControlServer:
    """Only the manager thread uses this socket and changes the streaming mode.

    A hold is idempotent and forces PLANNER until its owner releases it. Release
    leaves a button latch: only a new physical A+X edge can reenter POSE.
    """

    def __init__(self, port=5564):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://127.0.0.1:{port}")
        self.session = uuid.uuid4().hex
        self.token = None
        self.wait_for_ax = False
        self.released_token = None
        self.release_this_tick = False
        self.hold_ticks = 0
        self.pose_epoch = 0
        self.mode = "OFF"
        self.pending = None

    def poll(self):
        if self.pending is None and self.socket.poll(0):
            request = self.socket.recv_json()
            error = None
            operation = request.get("operation")
            token = request.get("token")
            if operation == "hold":
                if not isinstance(token, str) or not token:
                    error = "hold requires a token"
                elif self.token not in (None, token):
                    error = "another episode owns the hold"
                elif self.mode == "OFF":
                    error = "policy is OFF"
                elif self.token is None:
                    self.token = token
                    self.hold_ticks = 0
            elif operation == "release":
                if self.token == token and token:
                    self.token = None
                    self.wait_for_ax = True
                    self.released_token = token
                    self.release_this_tick = True
                elif self.token is not None or token != self.released_token:
                    error = "release token mismatch"
            elif operation != "status":
                error = "unknown operation"
            self.pending = error or "ok"

    def constrain(self, current, requested, *, ax_rising, emergency=False):
        if emergency:
            self.token = None
            self.wait_for_ax = False
            self.release_this_tick = False
            return "OFF"
        if self.token is not None:
            return "PLANNER"
        if self.release_this_tick:
            self.release_this_tick = False
            return "PLANNER"
        if self.wait_for_ax:
            if ax_rising and current == "PLANNER":
                self.wait_for_ax = False
                return "POSE"
            return "PLANNER"
        return requested

    def published(self, mode):
        if mode == "POSE" and self.mode not in ("POSE", "POSE_PAUSE"):
            self.pose_epoch += 1
        self.mode = mode
        if self.token is not None and mode == "PLANNER":
            self.hold_ticks += 1
        if self.pending is not None:
            self.socket.send_json({
                "ok": self.pending == "ok", "error": self.pending,
                "session": self.session, "mode": self.mode,
                "pose_epoch": self.pose_epoch, "hold_token": self.token,
                "hold_ticks": self.hold_ticks, "wait_for_ax": self.wait_for_ax,
                "released_token": self.released_token,
            })
            self.pending = None

    @property
    def idle_only(self):
        return self.token is not None or self.wait_for_ax

    def close(self):
        self.socket.close(0)
        self.context.term()


class EpisodeControlClient:
    """Nonblocking REQ client; a lost reply never authorizes a reset."""

    def __init__(self, port=5564):
        self.context = zmq.Context()
        self.endpoint = f"tcp://127.0.0.1:{port}"
        self.socket = None
        self.pending = None
        self.sent_at = 0.0
        self._connect()

    def _connect(self):
        if self.socket is not None:
            self.socket.close(0)
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.connect(self.endpoint)
        self.pending = None

    def request(self, operation, token=None):
        if self.pending is not None:
            return
        self.socket.send_json({"operation": operation, "token": token})
        self.pending = operation
        self.sent_at = time.monotonic()

    def poll(self):
        if self.pending is None:
            return None
        if self.socket.poll(0):
            result = self.socket.recv_json()
            result["operation"] = self.pending
            self.pending = None
            return result
        if time.monotonic() - self.sent_at > 1.0:
            self._connect()
        return None

    def close(self):
        self.socket.close(0)
        self.context.term()
