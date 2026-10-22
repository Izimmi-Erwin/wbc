"""Independent episode RGB sensors; no viewport or teleoperation publishers."""

import json
from pathlib import Path

import numpy as np


DEFAULT_WRIST_CONFIG = Path(__file__).resolve().parents[1] / "data/cameras/wrist_cameras.json"


class EpisodeRGBCamera:
    def read(self):
        import cv2

        # The callback bundles pixels and their Fabric timestamp. get_rgba()
        # reads the annotator separately and can return a different frame.
        frame = self.camera.get_current_frame()
        number = int(frame.get("rendering_frame", -1))
        stamp = float(frame.get("rendering_time", -1.0))
        if getattr(self, "_cached_stamp", None) == (number, stamp):
            return self._cached_read
        rgba = np.asarray(frame.get("rgba"))
        width, height = self.metadata["resolution"]
        valid = (number >= 0 and stamp >= 0 and np.isfinite(stamp)
                 and rgba.shape == (height, width, 4) and bool(np.any(rgba[:, :, :3])))
        encoded = b""
        if valid:
            ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR),
                                    [cv2.IMWRITE_JPEG_QUALITY, 90])
            valid = bool(ok)
            if ok:
                encoded = jpeg.tobytes()
        result = (encoded, valid, number, stamp)
        if valid:
            self._cached_stamp, self._cached_read = (number, stamp), result
        return result

    def close(self):
        self.camera.pause()


class EpisodeWristCamera(EpisodeRGBCamera):
    def __init__(self, backend, name, config, provenance):
        import omni.usd
        from isaacsim.sensors.camera import Camera

        parent = f"{backend.prim_path}/{config['parent_link']}"
        if not omni.usd.get_context().get_stage().GetPrimAtPath(parent).IsValid():
            raise ValueError(f"Wrist camera parent does not exist: {parent}")
        width, height = config["resolution"]
        fx, fy, cx, cy = config["intrinsics"]
        if (width <= 0 or height <= 0 or fx <= 0 or not np.isfinite([fx, fy, cx, cy]).all()
                or not np.allclose([fy, cx, cy], [fx, width / 2, height / 2])):
            raise ValueError("Wrist optics require centered, square-pixel pinhole intrinsics")
        if config["distortion_model"] != "pinhole" or any(config["distortion_coefficients"]):
            raise ValueError("Distorted wrist calibration requires a different camera projection")
        q = np.asarray(config["local_quaternion_wxyz"], dtype=float)
        p = np.asarray(config["local_position"], dtype=float)
        if (q.shape != (4,) or p.shape != (3,) or not np.isfinite(p).all()
                or not np.isclose(np.linalg.norm(q), 1.) or config["camera_axes"] != "usd"):
            raise ValueError("Wrist pose requires finite XYZ and unit wxyz quaternion in USD axes")
        self.path = f"{parent}/episode_{name}_camera"
        self.camera = Camera(prim_path=self.path, name=f"episode_{name}_camera",
                             resolution=(width, height))
        self.camera.set_local_pose(translation=p, orientation=q, camera_axes="usd")
        self.camera.initialize()
        self.camera.set_projection_type("pinhole")
        self.camera.set_focal_length(0.4)
        self.camera.set_horizontal_aperture(width * 0.4 / fx)
        self.camera.set_clipping_range(*config["clipping_range"])
        actual = np.asarray(self.camera.get_intrinsics_matrix())
        if not np.allclose(actual, [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], atol=1e-3):
            raise ValueError(f"Wrist rendered intrinsics differ: {actual}")
        self.metadata = {**config, "prim_path": self.path, "K": actual.tolist(),
                         "provenance": provenance,
                         "world_pose_convention": "T_world_camera_optical; XYZ metres + wxyz; optical X right, Y down, Z forward"}

    def world_pose(self):
        position, quaternion = self.camera.get_world_pose(camera_axes="ros")
        return np.asarray([*position, *quaternion], dtype=np.float32)


def create_wrist_cameras(backend, config_path=None):
    config = json.loads(Path(config_path or DEFAULT_WRIST_CONFIG).read_text())
    if set(config["cameras"]) != {"left_wrist", "right_wrist"}:
        raise ValueError("Wrist configuration must contain left_wrist and right_wrist")
    return {name: EpisodeWristCamera(backend, name, camera, config["provenance"])
            for name, camera in config["cameras"].items()}
