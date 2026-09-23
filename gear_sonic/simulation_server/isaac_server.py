"""Isaac Sim backend for Sonic WBC, camera publishing, and keyboard control."""

from __future__ import annotations

from dataclasses import dataclass, field
import csv
import importlib.util
import os
from pathlib import Path
import sys
from threading import RLock
import time
from typing import Any
from xml.etree import ElementTree

import numpy as np

from gear_sonic.robot_interface.joint_mapping import (
    SONIC_BODY_JOINT_NAMES,
    build_joint_mapping,
)
from gear_sonic.robot_interface.isaac_robot_assets import (
    DEFAULT_ROBOT_MODEL,
    get_robot_asset,
)


DEFAULT_G1_USD_PATH = get_robot_asset(DEFAULT_ROBOT_MODEL).usd_path
DEFAULT_SONICSTAR_TASK_SCENE_LAYER_PATH = (
    Path(__file__).resolve().parents[1]
    / "data"
    / "scenes"
    / "sonicstar"
    / "sonicstar_task_scene.usda"
)
DEFAULT_MUJOCO_G1_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "data"
    / "robot_model"
    / "model_data"
    / "g1"
    / "g1_29dof_with_hand.xml"
)
DEFAULT_FLUXVLA_SITE_PACKAGES = Path(
    "/home/limx/miniconda3/envs/fluxvla/lib/python3.10/site-packages"
)
ISAAC_ASSETS_HOST = "omniverse-content-production.s3-us-west-2.amazonaws.com"

@dataclass(frozen=True)
class MuJoCoCameraSpec:
    """An authored MuJoCo camera definition used as an Isaac camera contract."""

    name: str
    parent_body_name: str
    local_position: tuple[float, float, float]
    local_euler_xyz: tuple[float, float, float]
    source_path: Path
    vertical_fov_degrees: float = 45.0


def ensure_isaac_assets_bypass_proxy(environ: dict[str, str] | None = None) -> bool:
    """Bypass a local proxy only for Isaac Sim's configured S3 asset host.

    The preserved ``add_default_ground_plane`` implementation resolves
    ``default_environment.usd`` from this host.  On this workstation the local
    proxy can establish CONNECT but times out during its TLS handshake, whereas
    a direct connection succeeds.  Preserve all other proxy behavior and only
    append the required host to both libcurl-compatible no-proxy variables.

    Returns whether either variable changed, primarily for startup diagnostics.
    """
    environ = os.environ if environ is None else environ
    changed = False
    for key in ("NO_PROXY", "no_proxy"):
        entries = [entry.strip() for entry in environ.get(key, "").split(",") if entry.strip()]
        if ISAAC_ASSETS_HOST not in entries:
            entries.append(ISAAC_ASSETS_HOST)
            environ[key] = ",".join(entries)
            changed = True
    return changed


def load_mujoco_camera_spec(
    camera_name: str = "head_camera",
    model_path: str | Path = DEFAULT_MUJOCO_G1_MODEL_PATH,
) -> MuJoCoCameraSpec:
    """Read a named, body-mounted camera directly from the active G1 MJCF."""

    source_path = Path(model_path).expanduser().resolve()
    root = ElementTree.parse(source_path).getroot()
    for body in root.iter("body"):
        parent_name = body.get("name")
        if not parent_name:
            continue
        for child in body:
            if child.tag != "camera" or child.get("name") != camera_name:
                continue
            try:
                position = tuple(float(value) for value in child.attrib["pos"].split())
                euler = tuple(float(value) for value in child.attrib["euler"].split())
            except KeyError as exc:
                raise ValueError(f"MuJoCo camera {camera_name!r} has no {exc.args[0]!r} attribute") from exc
            if len(position) != 3 or len(euler) != 3:
                raise ValueError(f"MuJoCo camera {camera_name!r} must have three position and Euler values")
            return MuJoCoCameraSpec(
                name=camera_name,
                parent_body_name=parent_name,
                local_position=position,
                local_euler_xyz=euler,
                source_path=source_path,
                # MJCF fovy is in degrees, including when compiler angle is radian.
                vertical_fov_degrees=float(child.get("fovy", "45")),
            )
    raise ValueError(f"MuJoCo camera {camera_name!r} was not found in {source_path}")


def mujoco_intrinsic_xyz_euler_to_quat_wxyz(euler_xyz: np.ndarray) -> np.ndarray:
    """Convert MuJoCo's default intrinsic-XYZ Euler convention to Isaac wxyz."""

    roll, pitch, yaw = np.asarray(euler_xyz, dtype=np.float64)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    # MuJoCo accumulates lower-case (intrinsic) ``xyz`` rotations by
    # post-multiplication: qx * qy * qz.  This is not the common helper's
    # extrinsic-XYZ convention.
    return np.asarray(
        [
            cr * cp * cy - sr * sp * sy,
            sr * cp * cy + cr * sp * sy,
            cr * sp * cy - sr * cp * sy,
            cr * cp * sy + sr * sp * cy,
        ],
        dtype=np.float32,
    )


def _ensure_optional_site_packages(*module_names: str) -> None:
    """Expose optional runtime deps when Isaac Python does not ship them.

    Isaac Sim's bundled Python in this workspace does not include the msgpack /
    cv2 stack used by the MuJoCo camera protocol, while the local FluxVLA env
    does.  Adding that site-packages path keeps the wire protocol identical
    without modifying FluxVLA, WBC, USD, or control parameters.
    """

    missing = [name for name in module_names if importlib.util.find_spec(name) is None]
    if not missing:
        return
    site_packages = DEFAULT_FLUXVLA_SITE_PACKAGES
    if site_packages.exists() and str(site_packages) not in sys.path:
        sys.path.insert(0, str(site_packages))


def is_native_xr_vr_experience(experience_path: str | Path | None) -> bool:
    """Return whether *experience_path* is Isaac's native VR experience.

    The native XR follower must not be enabled for a normal on-screen Isaac
    viewport, the AR profile, or the separate Televiz camera-stream route.
    """

    if experience_path is None:
        return False
    return Path(experience_path).name == "isaacsim.exp.base.xr.vr.kit"


def select_native_xr_follow_anchor(
    *,
    robot_prim_path: str,
    root_body_prim_path: str,
    is_prim_path_valid: Any,
) -> str | None:
    """Select a verified G1 visual anchor without inventing an eye offset.

    The official G1 runtime hierarchy used here has ``head_link`` as a child
    of ``torso_link``.  It has no controllable neck DOF, but its composed USD
    transform still follows the robot.  Falling back toward the root preserves
    following behavior if a future asset variant lacks that visual prim.
    """

    robot_prim_path = robot_prim_path.rstrip("/")
    candidates = (
        f"{robot_prim_path}/torso_link/head_link",
        f"{robot_prim_path}/torso_link/head",
        f"{robot_prim_path}/torso_link",
        root_body_prim_path,
        robot_prim_path,
    )
    for candidate in candidates:
        if candidate and is_prim_path_valid(candidate):
            return candidate
    return None


def wait_for_native_xr_anchor(
    *,
    expected_anchor: str,
    pump_kit_frame: Any,
    get_stage_anchor: Any,
    max_frames: int = 20,
) -> str | None:
    """Pump Kit frames until XR reports the requested native stage anchor.

    This must only pump Kit/XR updates.  Calling ``World.step`` here would
    advance G1 physics while merely configuring a visual viewpoint.
    """

    actual_anchor = None
    for _ in range(max(0, int(max_frames))):
        pump_kit_frame()
        actual_anchor = get_stage_anchor()
        if actual_anchor == expected_anchor:
            break
    return actual_anchor


class IsaacNativeXrRobotFollower:
    """Bind Isaac's native XR physical-world anchor to the moving G1 USD prim.

    This deliberately uses the XR viewport controller's documented ``custom
    anchor`` setting rather than setting an active camera.  The controller
    keeps ``/_xr/stage/xrCamera`` as the HMD camera, so Pico head rotation
    remains native OpenXR input.  Since the anchor is a child of G1, its world
    transform changes with the robot without recurring teleports or guessed
    camera offsets.
    """

    def __init__(self, backend: "IsaacSimulationBackend") -> None:
        self.backend = backend
        self.anchor_path: str | None = None
        self._last_world_pose: (
            tuple[tuple[float, float, float], tuple[float, float, float, float]] | None
        ) = None
        self._last_report_time = 0.0

    def start(self) -> bool:
        if not is_native_xr_vr_experience(self.backend.experience_path):
            return False
        try:
            import carb
            from isaacsim.core.utils.prims import is_prim_path_valid
            from omni.kit.xr.core import XRCore

            self.anchor_path = select_native_xr_follow_anchor(
                robot_prim_path=self.backend.prim_path,
                root_body_prim_path=self.backend.robot_asset.root_body_prim_path,
                is_prim_path_valid=is_prim_path_valid,
            )
            if self.anchor_path is None:
                print("[IsaacNativeXrFollower] disabled: no valid G1 anchor prim was found")
                return False

            # The VR kit creates the profile asynchronously.  First request it
            # and let Kit process a few rendered frames, then apply the custom
            # anchor through the same settings the official XR viewport uses.
            XRCore.get_singleton().request_enable_profile("vr")
            for _ in range(3):
                self.backend.simulation_app.update()

            settings = carb.settings.get_settings()
            settings.set("/xrstage/profile/vr/customAnchor", self.anchor_path)
            settings.set("/xr/profile/vr/anchorMode", "custom anchor")
            settings.set("/xr/profile/vr/adjustForUserHeight", False)
            # Process the XR viewport controller's settings subscriptions
            # without advancing G1 physics during visual setup.
            configured_anchor = wait_for_native_xr_anchor(
                expected_anchor=self.anchor_path,
                pump_kit_frame=self.backend.simulation_app.update,
                get_stage_anchor=XRCore.get_singleton().get_stage_anchor_prim_path,
            )

            self._last_world_pose = self._world_pose()
            if configured_anchor != self.anchor_path:
                print(
                    "[IsaacNativeXrFollower] disabled: XR did not bind the requested "
                    f"G1 anchor requested={self.anchor_path} actual={configured_anchor}"
                )
                self.anchor_path = None
                return False
            print(
                "[IsaacNativeXrFollower] native VR custom anchor configured "
                f"anchor={self.anchor_path} world_pose={self._last_world_pose} "
                f"xr_stage_anchor={configured_anchor}; Pico head rotation remains native XR"
            )
            return True
        except Exception as exc:
            self.anchor_path = None
            print(f"[IsaacNativeXrFollower] disabled: could not configure native XR anchor: {exc!r}")
            return False

    def observe_after_step(self) -> None:
        """Log real anchor motion sparsely; no pose or camera is written here."""

        if self.anchor_path is None:
            return
        world_pose = self._world_pose()
        if world_pose is None or world_pose == self._last_world_pose:
            return
        now = time.monotonic()
        if now - self._last_report_time < 1.0:
            self._last_world_pose = world_pose
            return
        previous = self._last_world_pose
        self._last_world_pose = world_pose
        self._last_report_time = now
        print(
            "[IsaacNativeXrFollower] G1 anchor moved "
            f"anchor={self.anchor_path} world_pose={world_pose} previous_pose={previous}"
        )

    def _world_pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
        if self.anchor_path is None:
            return None
        try:
            from pxr import UsdGeom
            import omni.usd

            prim = omni.usd.get_context().get_stage().GetPrimAtPath(self.anchor_path)
            if not prim or not prim.IsValid():
                return None
            world_transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            translation = world_transform.ExtractTranslation()
            rotation = world_transform.ExtractRotationQuat()
            orientation_wxyz = (rotation.GetReal(), *rotation.GetImaginary())
            return (
                tuple(round(float(value), 4) for value in translation),
                tuple(round(float(value), 4) for value in orientation_wxyz),
            )
        except Exception:
            return None


class IsaacImagePublisher:
    """Isaac camera publisher with the same ZMQ/msgpack schema as MuJoCo port 5555."""

    def __init__(
        self,
        *,
        port: int = 5555,
        camera_name: str = "ego_view",
        camera_source: str = "robot_ego",
        width: int = 640,
        height: int = 480,
        image_dt: float = 1.0 / 60.0,
        verbose: bool = True,
        local_translation: tuple[float, float, float] = (0.06, 0.0, 0.25),
        ego_camera_intrinsics: tuple[float, float, float, float] | None = None,
    ):
        self.port = int(port)
        self.camera_name = str(camera_name)
        self.camera_source = str(camera_source)
        if self.camera_source not in {"robot_ego", "gui_perspective"}:
            raise ValueError(
                "camera_source must be 'robot_ego' or 'gui_perspective', "
                f"got {self.camera_source!r}"
            )
        self.width = int(width)
        self.height = int(height)
        # Nominal RGB K in pixels at this camera resolution, not depth K.
        intrinsics = np.asarray(
            (620.80, 625.22, 320.0, 240.0)
            if ego_camera_intrinsics is None else ego_camera_intrinsics,
            dtype=np.float64,
        )
        if ego_camera_intrinsics is None:
            # Preserve direct callers using smaller images: resize the 640x480
            # reference K. This does not select a different hardware profile.
            intrinsics *= (self.width / 640.0, self.height / 480.0) * 2
        if self.camera_name == "ego_view" and (
            intrinsics.shape != (4,)
            or not np.all(np.isfinite(intrinsics))
            or np.any(intrinsics[:2] <= 0)
            or not 0 <= intrinsics[2] < self.width
            or not 0 <= intrinsics[3] < self.height
        ):
            raise ValueError(
                "ego_camera_intrinsics must be finite (fx>0, fy>0, cx, cy) "
                "with an in-image principal point"
            )
        self.ego_camera_intrinsics = tuple(float(value) for value in intrinsics)
        self.image_dt = float(image_dt)
        self.verbose = bool(verbose)
        self.local_translation = np.asarray(local_translation, dtype=np.float32)
        if self.local_translation.shape != (3,):
            raise ValueError("local_translation must contain exactly three values")
        self.sensor_server = None
        self.camera = None
        self._gui_rgb_annotator = None
        self._gui_render_product_path = None
        self._last_publish_time = 0.0
        self._fallback_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._consecutive_black_frames = 0
        self._last_camera_recovery_time = 0.0
        self._mujoco_camera_spec: MuJoCoCameraSpec | None = None

    def start(self, backend: "IsaacSimulationBackend") -> None:
        _ensure_optional_site_packages("msgpack", "msgpack_numpy", "cv2", "zmq")
        from gear_sonic.utils.mujoco_sim.sensor_server import SensorServer

        self.sensor_server = SensorServer()
        self.sensor_server.start_server(port=self.port)
        self._try_create_camera(backend)
        print(
            "[IsaacImagePublisher] publishing MuJoCo-compatible camera stream "
            f"camera={self.camera_name!r} source={self.camera_source!r} "
            f"port={self.port} shape=({self.height},{self.width},3)"
        )

    def stop(self) -> None:
        if self.sensor_server is not None:
            try:
                self.sensor_server.stop_server()
            except Exception:
                pass
        self.sensor_server = None
        self.camera = None
        if self._gui_rgb_annotator is not None and self._gui_render_product_path is not None:
            try:
                self._gui_rgb_annotator.detach([self._gui_render_product_path])
            except Exception:
                pass
        self._gui_rgb_annotator = self._gui_render_product_path = None

    def maybe_publish(self, backend: "IsaacSimulationBackend") -> None:
        if self.sensor_server is None:
            return
        now = time.time()
        if now - self._last_publish_time < self.image_dt:
            return
        self._last_publish_time = now
        self.publish(backend=backend, timestamp=now)

    def publish(self, *, backend: "IsaacSimulationBackend", timestamp: float | None = None) -> None:
        if self.sensor_server is None:
            return
        from gear_sonic.utils.mujoco_sim.sensor_server import (
            ImageMessageSchema,
        )

        image = self._read_camera_image()
        # ``World.reset()`` (for example Isaac's backspace reset path) can
        # invalidate Camera's render-product annotator.  Its getters then
        # return a correctly shaped but all-zero image forever.  Reinitialize
        # the existing camera after consecutive black reads instead of
        # publishing a misleading black frame to Pico indefinitely.
        if image.size and not np.any(image):
            self._consecutive_black_frames += 1
            now = time.monotonic()
            if (
                self._consecutive_black_frames >= 2
                and now - self._last_camera_recovery_time >= 2.0
            ):
                self._last_camera_recovery_time = now
                self._recover_camera_after_world_reset(backend)
                image = self._read_camera_image()
                self._consecutive_black_frames = 0 if np.any(image) else 2
        else:
            self._consecutive_black_frames = 0
        timestamp = time.time() if timestamp is None else float(timestamp)
        image_msg = ImageMessageSchema(
            timestamps={self.camera_name: timestamp},
            images={self.camera_name: image},
        )
        serialized_data = image_msg.serialize()
        # MuJoCo's ImagePublishProcess also includes a legacy top-level key
        # with the same JPEG payload. Reuse it without encoding a second time.
        serialized_data[self.camera_name] = (
            serialized_data['images'][self.camera_name]
        )
        self.sensor_server.send_message(serialized_data)

    def _recover_camera_after_world_reset(self, backend: "IsaacSimulationBackend") -> None:
        """Rebuild the Camera render-product state without resetting physics."""
        if self.camera_source == "gui_perspective":
            self._try_create_camera(backend)
            return
        if self.camera is None:
            self._try_create_camera(backend)
            return
        try:
            initialize = getattr(self.camera, "initialize", None)
            if callable(initialize):
                initialize()
            self._configure_ego_camera_optics()
            if backend.world is not None:
                # The camera annotator receives data only after rendered
                # frames following initialization.  Do not call world.reset:
                # that would disrupt the active WBC/IK session.
                for _ in range(3):
                    backend.world.step(render=True)
            print("[IsaacImagePublisher] recovered camera after all-zero frame read")
        except Exception as exc:
            print(f"[IsaacImagePublisher] warning: camera recovery failed: {exc!r}")

    def _configure_ego_camera_optics(self) -> None:
        """Apply the FluxBisim RGB-intrinsics convention to the ego camera only."""
        if self.camera is None or self._mujoco_camera_spec is None:
            return
        fx, fy, cx, cy = self.ego_camera_intrinsics
        # FluxBisim selects a virtual 3 um pixel pitch and one mean focal
        # length. It approximates K; it does not reproduce unequal fx/fy.
        pixel_size_mm = 0.003
        mean_focal_px = (fx + fy) / 2.0
        self.camera.set_focal_length(mean_focal_px * pixel_size_mm / 10.0)
        self.camera.set_horizontal_aperture(self.width * pixel_size_mm / 10.0)
        self.camera.set_vertical_aperture(self.height * pixel_size_mm / 10.0)
        # Keep the centered case as an ordinary pinhole. For a calibrated
        # off-center principal point use FluxBisim's polynomial approximation.
        if cx == self.width / 2.0 and cy == self.height / 2.0:
            self.camera.set_projection_type("pinhole")
        else:
            diagonal = 2.0 * np.hypot(max(cx, self.width - cx), max(cy, self.height - cy))
            diagonal_fov = float(np.rad2deg(2.0 * np.arctan2(diagonal, fx + fy)))
            self.camera.set_projection_type("fisheyePolynomial")
            self.camera.set_rational_polynomial_properties(
                self.width, self.height, cx, cy, diagonal_fov, np.zeros(8)
            )
        # Preserve the existing clip planes, pose, and GUI Perspective camera.
        self.camera.set_clipping_range(near_distance=0.02, far_distance=100.0)
        print(
            "[IsaacImagePublisher] ego RGB optics (FluxBisim mean-focal approximation): "
            f"requested_fx_fy_cx_cy={self.ego_camera_intrinsics} "
            f"effective_fx_fy=({mean_focal_px}, {mean_focal_px}) "
            f"resolution=({self.width}, {self.height})"
        )

    def _try_create_camera(self, backend: "IsaacSimulationBackend") -> None:
        if self.camera_source == "gui_perspective":
            # The GUI render product is the *published* source in this mode,
            # but keep the G1-mounted camera prim alive as a selectable GUI
            # backup.  It owns its separate render product and does not alter
            # the active Perspective viewport or its 5555 payload.
            self._try_attach_gui_perspective()
        try:
            try:
                from isaacsim.sensors.camera import Camera
            except Exception:
                from omni.isaac.sensor import Camera
            from isaacsim.core.utils.prims import is_prim_path_valid

            robot_prim = getattr(backend, "prim_path", "/World/G1").rstrip("/")
            root_body = getattr(getattr(backend, "robot_asset", None), "root_body_prim_path", "")

            if self.camera_name == "ego_view":
                # Scheme 2's image is the MJCF ``head_camera`` stream.  Read
                # that authored transform instead of accepting a separately
                # tuned Isaac offset; otherwise the two simulators drift.
                self._mujoco_camera_spec = load_mujoco_camera_spec()
                camera_parent_path = f"{robot_prim}/{self._mujoco_camera_spec.parent_body_name}"
                if not is_prim_path_valid(camera_parent_path):
                    raise RuntimeError(
                        "MuJoCo ego camera parent is absent from the loaded Isaac stage: "
                        f"{camera_parent_path}"
                    )
                local_translation = np.asarray(self._mujoco_camera_spec.local_position, dtype=np.float32)
                local_euler_xyz = np.asarray(self._mujoco_camera_spec.local_euler_xyz, dtype=np.float32)
            else:
                camera_parent_path = self._resolve_head_camera_parent(
                    robot_prim=robot_prim,
                    root_body=root_body,
                    is_prim_path_valid=is_prim_path_valid,
                )
                local_translation = self.local_translation
                local_euler_xyz = np.asarray([0.0, -0.8, -1.57], dtype=np.float32)

            orientation = mujoco_intrinsic_xyz_euler_to_quat_wxyz(local_euler_xyz)

            camera_prim_path = f"{camera_parent_path}/{self.camera_name}_camera"
            self.camera = Camera(
                prim_path=camera_prim_path,
                name=f"{self.camera_name}_camera",
                translation=local_translation,
                resolution=(self.width, self.height),
                frequency=max(1, int(round(1.0 / self.image_dt))),
            )
            self.camera.set_local_pose(
                translation=local_translation,
                orientation=orientation,
                camera_axes="usd",
            )
            initialize = getattr(self.camera, "initialize", None)
            if initialize is not None:
                initialize()
            self._configure_ego_camera_optics()
            if backend.world is not None:
                # Camera's RGB annotator is populated asynchronously.  A
                # single rendered frame is not reliable on the first Isaac
                # frame (it commonly reads as a valid-shaped all-zero image).
                # Warm it through several render frames before its first
                # 5555 publication.
                for _ in range(12):
                    backend.world.step(render=True)
            self._try_configure_xr_view(backend, camera_prim_path)
            parent_world_xyz = self._get_prim_world_xyz(camera_parent_path)
            camera_world_xyz = self._get_prim_world_xyz(camera_prim_path)
            print(
                "[IsaacImagePublisher] Isaac camera initialized "
                f"at {camera_prim_path} parent={camera_parent_path} "
                f"local_translation={local_translation.tolist()} "
                f"parent_world_xyz={parent_world_xyz} camera_world_xyz={camera_world_xyz} "
                f"local_euler={local_euler_xyz.tolist()} camera_axes=usd "
                f"mjcf_reference_fovy_degrees={self._mujoco_camera_spec.vertical_fov_degrees if self._mujoco_camera_spec else None} "
                f"clipping_range={self.camera.get_clipping_range()} "
                f"mujoco_source={self._mujoco_camera_spec.source_path if self._mujoco_camera_spec else None}"
            )
        except Exception as exc:
            self.camera = None
            print(
                "[IsaacImagePublisher] warning: Isaac Camera unavailable; "
                f"publishing black fallback frames. error={exc!r}"
            )

    def _try_attach_gui_perspective(self) -> None:
        """Read the active GUI Perspective product without changing its camera or size."""
        try:
            from omni.kit.viewport.utility import get_active_viewport
            import omni.replicator.core as rep

            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError("Isaac GUI active viewport is unavailable")
            camera_path = str(getattr(viewport, "camera_path", ""))
            if camera_path != "/OmniverseKit_Persp":
                raise RuntimeError(
                    "GUI Perspective source requires active viewport camera "
                    f"'/OmniverseKit_Persp', got {camera_path!r}"
                )
            render_product_path = viewport.get_render_product_path()
            if not render_product_path:
                raise RuntimeError("active GUI viewport has no render product")
            self._gui_rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._gui_rgb_annotator.attach([render_product_path])
            self._gui_render_product_path = str(render_product_path)
            resolution = tuple(int(value) for value in viewport.get_texture_resolution())
            print(
                "[IsaacImagePublisher] attached to live GUI Perspective "
                f"camera={camera_path} render_product={self._gui_render_product_path} "
                f"viewport_resolution={resolution}; GUI pose and resolution are unchanged"
            )
        except Exception as exc:
            self._gui_rgb_annotator = self._gui_render_product_path = None
            print(
                "[IsaacImagePublisher] warning: GUI Perspective capture unavailable; "
                f"publishing black fallback frames. error={exc!r}"
            )

    @staticmethod
    def _get_prim_world_xyz(prim_path: str) -> tuple[float, float, float] | None:
        """Return a compact world-space position for startup diagnostics."""
        try:
            from pxr import UsdGeom
            import omni.usd

            prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return None
            translation = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation()
            return tuple(round(float(value), 4) for value in translation)
        except Exception:
            return None

    @staticmethod
    def _resolve_head_camera_parent(*, robot_prim: str, root_body: str, is_prim_path_valid: Any) -> str:
        """Find the head prim in the *loaded* G1 stage, with safe fallbacks."""
        # This is the conventional path in the local official G1 asset.  It is
        # still checked against the loaded stage; the scan below handles an
        # asset whose head hierarchy differs.
        explicit_head = f"{robot_prim}/torso_link/head"
        if is_prim_path_valid(explicit_head):
            return explicit_head

        try:
            from pxr import Usd
            import omni.usd

            robot = omni.usd.get_context().get_stage().GetPrimAtPath(robot_prim)
            if robot and robot.IsValid():
                head_candidates = []
                # Usd.Prim has no GetDescendants() API.  PrimRange traverses
                # the actual composed stage, including the referenced G1 USD.
                for prim in Usd.PrimRange(robot):
                    name = prim.GetName().lower()
                    if name == "head" or name.startswith("head_") or name.endswith("_head"):
                        path = str(prim.GetPath())
                        if is_prim_path_valid(path):
                            head_candidates.append(path)
                if head_candidates:
                    # Prefer the shallowest matching prim: this is the head
                    # frame/visual root, not a mesh/material below it.
                    return min(head_candidates, key=lambda path: (path.count("/"), path))
        except Exception as exc:
            print(f"[IsaacImagePublisher] warning: unable to scan G1 head prim: {exc!r}")

        for fallback in (f"{robot_prim}/torso_link", root_body, robot_prim):
            if fallback and is_prim_path_valid(fallback):
                print(
                    "[IsaacImagePublisher] warning: no head prim was found; "
                    f"falling back to {fallback}"
                )
                return fallback
        return robot_prim

    def _try_configure_xr_view(self, backend: "IsaacSimulationBackend", camera_prim_path: str) -> None:
        # Native VR owns ``/_xr/stage/xrCamera`` and is intentionally anchored
        # by ``IsaacNativeXrRobotFollower``.  Re-selecting a streamed RGB
        # camera here would switch the profile back to active-camera mode and
        # break robot-following.  The 5555 route remains available in non-XR
        # runs and is not part of the native Pico view.
        if is_native_xr_vr_experience(getattr(backend, "experience_path", None)):
            print(
                "[IsaacImagePublisher] native VR active; leaving XR camera under "
                "IsaacNativeXrFollower (5555 camera is not the Pico view)"
            )
            return
        experience_name = ""
        experience_path = getattr(backend, "experience_path", None)
        if experience_path is not None:
            experience_name = Path(experience_path).name

        if not experience_name:
            return

        try:
            import carb
            from omni.kit.viewport.utility import get_active_viewport

            settings = carb.settings.get_settings()
            settings.set("/xr/profile/ar/anchorMode", "active camera")
            settings.set("/xr/profile/vr/anchorMode", "active camera")
            settings.set("/xr/profile/ar/adjustForUserHeight", False)
            settings.set("/xr/profile/vr/adjustForUserHeight", False)

            viewport = get_active_viewport()
            if viewport is not None:
                # ``camera_path`` assignment is supported by the legacy
                # viewport wrapper, but the XR VR experience can expose the
                # concrete viewport API instead.  Use its explicit method
                # when available so the active XR camera is actually changed
                # rather than merely updating a Python-side property.
                set_active_camera = getattr(viewport, "set_active_camera", None)
                if callable(set_active_camera):
                    set_active_camera(camera_prim_path)
                else:
                    viewport.camera_path = camera_prim_path

            profile_name = "vr" if ".vr." in experience_name else "ar"
            xr_core = None
            try:
                from omni.kit.xr.core import XRCore

                xr_core = XRCore.get_singleton()
                xr_core.request_enable_profile(profile_name)
            except Exception as exc:
                print(
                    "[IsaacImagePublisher] warning: could not request XR profile "
                    f"{profile_name!r}: {exc!r}"
                )

            if backend.world is not None:
                # Let XR create and bind its internal ``xrCamera`` first.
                # Selecting the G1 camera only before this point is lost when
                # XR takes over the viewport, so the active-camera anchor is
                # never reset to the robot head.
                backend.world.step(render=True)

            if viewport is not None:
                set_active_camera = getattr(viewport, "set_active_camera", None)
                if callable(set_active_camera):
                    set_active_camera(camera_prim_path)
                else:
                    viewport.camera_path = camera_prim_path

            if backend.world is not None:
                # The XR viewport controller observes the re-selection above,
                # restores ``xrCamera``, and teleports/anchors it to the G1
                # head camera on these frames.
                for _ in range(12):
                    backend.world.step(render=True)

            stage_anchor = None
            if xr_core is not None:
                try:
                    stage_anchor = xr_core.get_stage_anchor_prim_path()
                except Exception as exc:
                    print(f"[IsaacImagePublisher] warning: unable to read XR stage anchor: {exc!r}")
            print(
                "[IsaacImagePublisher] XR view configured "
                f"profile={profile_name!r} anchorMode='active camera' "
                f"adjustForUserHeight=False viewport_camera={camera_prim_path} "
                f"active_camera={getattr(viewport, 'camera_path', None)} "
                f"stage_anchor={stage_anchor}"
            )
        except Exception as exc:
            print(
                "[IsaacImagePublisher] warning: could not configure XR active camera; "
                f"error={exc!r}"
            )

    def _read_camera_image(self) -> np.ndarray:
        if self.camera_source == "gui_perspective":
            return self._read_gui_perspective_image()
        if self.camera is None:
            return self._fallback_frame

        try:
            rgba = None
            for getter_name in ("get_rgba", "get_rgb"):
                getter = getattr(self.camera, getter_name, None)
                if getter is not None:
                    rgba = getter()
                    if rgba is not None:
                        break
            if rgba is None:
                return self._fallback_frame
            image = np.asarray(rgba)
            if image.size == 0:
                return self._fallback_frame
            if image.ndim == 3 and image.shape[-1] >= 3:
                image = image[..., :3]
            image = np.asarray(image)
            if image.dtype != np.uint8:
                max_value = float(np.nanmax(image)) if image.size else 0.0
                if max_value <= 1.0:
                    image = image * 255.0
                image = np.clip(image, 0, 255).astype(np.uint8)
            if image.shape[:2] != (self.height, self.width):
                try:
                    import cv2

                    image = cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_AREA)
                except Exception:
                    return self._fallback_frame
            return np.ascontiguousarray(image)
        except Exception as exc:
            if self.verbose:
                print(f"[IsaacImagePublisher] warning: failed to read camera frame: {exc!r}")
            return self._fallback_frame

    def _read_gui_perspective_image(self) -> np.ndarray:
        if self._gui_rgb_annotator is None:
            return self._fallback_frame
        try:
            image = np.asarray(self._gui_rgb_annotator.get_data())
            if image.size == 0:
                return self._fallback_frame
            if (
                image.ndim == 3
                and image.shape[-1] == 4
                and image.dtype == np.uint8
            ):
                import cv2

                # Drop alpha into packed RGB before resizing to avoid a slow
                # implicit copy of the strided three-channel view.
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
            elif image.ndim == 3 and image.shape[-1] >= 3:
                image = image[..., :3]
            if image.ndim != 3 or image.shape[-1] != 3:
                return self._fallback_frame
            if image.dtype != np.uint8:
                max_value = float(np.nanmax(image)) if image.size else 0.0
                if max_value <= 1.0:
                    image = image * 255.0
                image = np.clip(image, 0, 255).astype(np.uint8)
            if image.shape[:2] != (self.height, self.width):
                import cv2

                image = cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_AREA)
            return np.ascontiguousarray(image)
        except Exception as exc:
            if self.verbose:
                print(f"[IsaacImagePublisher] warning: failed to read GUI Perspective frame: {exc!r}")
            return self._fallback_frame


class IsaacDeployRelay:
    """Own deploy port 5556 and forward command and pose publishers to it."""

    def __init__(
        self,
        *,
        command_host: str = "localhost",
        command_port: int = 5562,
        pose_host: str = "localhost",
        pose_port: int = 5563,
        deploy_host: str = "*",
        deploy_port: int = 5556,
    ):
        self.command_endpoint = f"tcp://{command_host}:{int(command_port)}"
        self.pose_endpoint = f"tcp://{pose_host}:{int(pose_port)}"
        self.deploy_endpoint = f"tcp://{deploy_host}:{int(deploy_port)}"
        self._ctx: Any | None = None
        self._command_socket: Any | None = None
        self._pose_socket: Any | None = None
        self._deploy_socket: Any | None = None
        self._zmq: Any | None = None

    def start(self) -> bool:
        if self._ctx is not None:
            return True
        try:
            _ensure_optional_site_packages("zmq")
            import zmq

            self._zmq = zmq
            self._ctx = zmq.Context()
            self._command_socket = self._ctx.socket(zmq.SUB)
            self._command_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self._command_socket.connect(self.command_endpoint)
            self._pose_socket = self._ctx.socket(zmq.SUB)
            self._pose_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self._pose_socket.connect(self.pose_endpoint)
            self._deploy_socket = self._ctx.socket(zmq.PUB)
            self._deploy_socket.bind(self.deploy_endpoint)
            print(
                "[IsaacDeployRelay] deploy owner bound at "
                f"{self.deploy_endpoint}; inputs={self.command_endpoint},{self.pose_endpoint}"
            )
            return True
        except Exception as exc:
            print(f"[IsaacDeployRelay] disabled: {exc!r}")
            self.stop()
            return False

    def poll_once(self) -> int:
        if self._zmq is None or self._deploy_socket is None:
            return 0
        forwarded = 0
        for socket in (self._command_socket, self._pose_socket):
            if socket is None:
                continue
            while True:
                try:
                    self._deploy_socket.send(socket.recv(flags=self._zmq.NOBLOCK))
                    forwarded += 1
                except self._zmq.Again:
                    break
        return forwarded

    def stop(self) -> None:
        for socket_name in ("_command_socket", "_pose_socket", "_deploy_socket"):
            socket = getattr(self, socket_name)
            if socket is not None:
                try:
                    socket.close(0)
                except Exception:
                    pass
                setattr(self, socket_name, None)
        if self._ctx is not None:
            try:
                self._ctx.term()
            except Exception:
                pass
        self._ctx = self._zmq = None


class IsaacKeyboardCommandSubscriber:
    """Subscribe to the shared FluxVLA keyboard channel for simulator-only keys.

    ``send_keyboard_cmd.py`` owns the PUB side on port 5580. Isaac always
    mirrors the simulator-local keys ``9`` and ``Backspace``. When explicitly
    enabled by ``--isaac-forward-k-to-deploy``, it also reproduces the original
    inference script's stateful ``k`` toggle by publishing commands on port
    5562. ``IsaacDeployRelay`` is the sole publisher that binds deploy port
    5556, so full-body pose traffic cannot race the keyboard publisher.
    """

    SIM_KEYS = {"9", "backspace", "\b", "\x7f"}

    def __init__(
        self,
        backend: "IsaacSimulationBackend",
        *,
        host: str = "localhost",
        port: int = 5580,
        forward_k_to_deploy: bool = False,
        deploy_host: str = "*",
        deploy_port: int = 5562,
    ):
        self.backend = backend
        self.host = host
        self.port = int(port)
        self.endpoint = f"tcp://{host}:{self.port}"
        self._ctx: Any | None = None
        self._socket: Any | None = None
        self._zmq: Any | None = None
        self._running = False
        self.forward_k_to_deploy = bool(forward_k_to_deploy)
        self.deploy_endpoint = f"tcp://{deploy_host}:{int(deploy_port)}"
        self._deploy_socket: Any | None = None
        self._build_command_message: Any | None = None
        self._deploy_running = False
        self._deploy_mode = "OFF"

    def start(self) -> bool:
        if self._running:
            return True
        try:
            _ensure_optional_site_packages("zmq")
            import zmq

            self._zmq = zmq
            self._ctx = zmq.Context()
            self._socket = self._ctx.socket(zmq.SUB)
            self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self._socket.setsockopt(zmq.CONFLATE, 1)
            self._socket.setsockopt(zmq.RCVTIMEO, 0)
            self._socket.connect(self.endpoint)
            if self.forward_k_to_deploy:
                from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message

                self._deploy_socket = self._ctx.socket(zmq.PUB)
                self._deploy_socket.bind(self.deploy_endpoint)
                self._build_command_message = build_command_message
                # Match the inference runner's publisher setup: allow the
                # deploy SUB socket to complete its ZMQ handshake before the
                # first keyboard command can be emitted.
                time.sleep(0.1)
                print(
                    "[IsaacKeyboard] deploy-control k forwarder bound at "
                    f"{self.deploy_endpoint}; k toggles planner start/stop"
                )
            self._running = True
            print(
                "[IsaacKeyboard] connected to shared keyboard channel "
                f"{self.endpoint}; handling 9/backspace"
            )
            return True
        except Exception as exc:
            print(f"[IsaacKeyboard] disabled: {exc!r}")
            self.stop()
            return False

    def poll_once(self) -> bool:
        if not self._running or self._socket is None or self._zmq is None:
            return False
        try:
            key = self._socket.recv_string(flags=self._zmq.NOBLOCK)
        except self._zmq.Again:
            return False
        except Exception as exc:
            print(f"[IsaacKeyboard] receive failed; disabling subscriber: {exc!r}")
            self.stop()
            return False

        normalized = str(key).lower()
        if normalized in self.SIM_KEYS:
            self.backend.keyboard(normalized)
            return True
        if normalized == "k" and self.forward_k_to_deploy:
            self._toggle_deploy_planner()
            return True
        return False

    def _toggle_deploy_planner(self) -> None:
        """Send the original inference script's stateful ``k`` command."""
        if self._deploy_socket is None or self._build_command_message is None:
            print("[IsaacKeyboard] k ignored: deploy forwarder is unavailable")
            return
        if self._deploy_running:
            planner = self._deploy_mode == "PLANNER"
            self._deploy_socket.send(
                self._build_command_message(start=False, stop=True, planner=planner)
            )
            # Same send cadence as send_cpp_control_command() in the original
            # FluxVLA runner.  State changes only after the wire message has
            # been submitted to ZMQ.
            time.sleep(0.01)
            self._deploy_running = False
            self._deploy_mode = "OFF"
            print(f"[IsaacKeyboard] k -> deploy planner stop (planner={planner})")
        else:
            self._deploy_socket.send(
                self._build_command_message(start=True, stop=False, planner=True)
            )
            time.sleep(0.01)
            self._deploy_running = True
            self._deploy_mode = "PLANNER"
            print("[IsaacKeyboard] k -> deploy planner start (planner=True)")

    def stop(self) -> None:
        self._running = False
        if self._deploy_socket is not None:
            try:
                self._deploy_socket.close(0)
            except Exception:
                pass
            self._deploy_socket = None
        if self._socket is not None:
            try:
                self._socket.close(0)
            except Exception:
                pass
            self._socket = None
        if self._ctx is not None:
            try:
                self._ctx.term()
            except Exception:
                pass
            self._ctx = None
        self._zmq = None


class IsaacRealtimeStatePublisher:
    """Optional Isaac-side publisher for GR00T/FluxVLA g1_debug frames on port 5557.

    The normal four-terminal flow should leave this disabled because
    ``g1_deploy_onnx_ref`` already owns 5557 and publishes the canonical C++
    payload. This class is a protocol-compatible fallback for Isaac-only audits.
    """

    def __init__(self, *, port: int = 5557, topic: str = "g1_debug", verbose: bool = True):
        self.port = int(port)
        self.topic = str(topic)
        self.verbose = bool(verbose)
        self.context = None
        self.socket = None
        self.index = 0
        self._last_config_publish = 0.0

    def start(self, backend: "IsaacSimulationBackend") -> None:
        _ensure_optional_site_packages("msgpack", "zmq")
        import zmq

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 20)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{self.port}")
        print(
            "[IsaacRealtimeStatePublisher] optional g1_debug publisher running "
            f"at tcp://*:{self.port} topic={self.topic!r}"
        )

    def stop(self) -> None:
        if self.socket is not None:
            try:
                self.socket.close()
            except Exception:
                pass
        if self.context is not None:
            try:
                self.context.term()
            except Exception:
                pass
        self.socket = None
        self.context = None

    def publish(self, backend: "IsaacSimulationBackend", state: IsaacRobotState | None = None) -> None:
        if self.socket is None:
            return
        _ensure_optional_site_packages("msgpack", "zmq")
        import msgpack
        import zmq

        state = backend.read_joint_state() if state is None else state
        now = time.time()
        if now - self._last_config_publish >= 2.0:
            self._send(
                "robot_config",
                {
                    "control_loop_type": "cpp",
                    "simulator": "isaac",
                    "robot_model": backend.robot_model,
                    "body_dof": len(SONIC_BODY_JOINT_NAMES),
                    "topic_prefix": self.topic,
                },
                msgpack_module=msgpack,
                zmq_module=zmq,
            )
            self._last_config_publish = now

        payload = self._build_debug_payload(backend=backend, state=state, timestamp=now)
        self._send(self.topic, payload, msgpack_module=msgpack, zmq_module=zmq)
        self.index += 1

    def _send(self, topic: str, payload: dict[str, Any], *, msgpack_module: Any, zmq_module: Any) -> None:
        packed = msgpack_module.packb(payload, use_bin_type=True)
        # Match the C++ ZMQ output handler: one frame, topic prefix prepended
        # directly to the msgpack payload.
        frame = topic.encode("utf-8") + packed
        try:
            self.socket.send(frame, flags=zmq_module.NOBLOCK)
        except zmq_module.Again:
            if self.verbose:
                print(f"[IsaacRealtimeStatePublisher] warning: dropped {topic} frame")

    def _build_debug_payload(
        self,
        *,
        backend: "IsaacSimulationBackend",
        state: IsaacRobotState,
        timestamp: float,
    ) -> dict[str, Any]:
        body_q = self._sonic_order(state.joint_position, backend)
        body_dq = self._sonic_order(state.joint_velocity, backend)
        base_quat = _fit_float_list(state.root_quaternion, 4, fill=0.0)
        if len(base_quat) == 4 and not any(base_quat):
            base_quat = [1.0, 0.0, 0.0, 0.0]
        base_ang_vel = _fit_float_list(state.root_angular_velocity, 3, fill=0.0)
        base_pos = _fit_float_list(state.root_position, 3, fill=0.0)
        zero3 = [0.0, 0.0, 0.0]
        zero4 = [1.0, 0.0, 0.0, 0.0]
        zero7 = [0.0] * 7
        zero29 = [0.0] * len(SONIC_BODY_JOINT_NAMES)
        return {
            "control_loop_type": "cpp",
            "index": int(self.index),
            "ros_timestamp": float(timestamp),
            "base_quat": base_quat,
            "base_ang_vel": base_ang_vel,
            "body_torso_quat": base_quat,
            "body_torso_ang_vel": base_ang_vel,
            "body_q": body_q,
            "body_dq": body_dq,
            "left_hand_q": zero7,
            "left_hand_dq": zero7,
            "right_hand_q": zero7,
            "right_hand_dq": zero7,
            "last_action": zero29,
            "last_left_hand_action": zero7,
            "last_right_hand_action": zero7,
            "token_state": [],
            "motor_temperature": [0.0] * 58,
            "base_trans_target": base_pos,
            "base_quat_target": base_quat,
            "body_q_target": body_q,
            "base_trans_measured": base_pos,
            "base_quat_measured": base_quat,
            "body_q_measured": body_q,
            "left_hand_q_measured": zero7,
            "right_hand_q_measured": zero7,
            "vr_3point_position": zero3 * 3,
            "vr_3point_orientation": zero4 * 3,
            "vr_3point_compliance": zero3,
        }

    def _sonic_order(self, values: list[float], backend: "IsaacSimulationBackend") -> list[float]:
        if backend.joint_mapping is None:
            return _fit_float_list(values, len(SONIC_BODY_JOINT_NAMES), fill=0.0)
        array = np.asarray(values, dtype=np.float64).reshape(-1)
        ordered = []
        for isaac_index in backend.joint_mapping.sonic_body_to_isaac_dof:
            ordered.append(float(array[isaac_index]) if isaac_index < array.size else 0.0)
        return ordered


@dataclass
class IsaacElasticBandState:
    """MuJoCo ElasticBand runtime equivalent for Isaac.

    Mirrors ``gear_sonic.utils.mujoco_sim.unitree_sdk2py_bridge.ElasticBand``:
    force and torque are applied to the target rigid body in world coordinates.
    """

    kp_pos: float = 10000.0
    kd_pos: float = 1000.0
    kp_ang: float = 1000.0
    kd_ang: float = 10.0
    point: np.ndarray = field(default_factory=lambda: np.asarray([0.0, 0.0, 1.0], dtype=np.float32))
    target_yaw: float = 0.0  # World Z heading in radians; zero preserves the original behavior.
    length: float = 0.0
    enabled: bool = True
    target_link: str = ""
    target_prim_path: str = ""
    force: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    torque: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    last_error: str | None = None

    def compute(
        self,
        *,
        position: np.ndarray,
        orientation_wxyz: np.ndarray,
        linear_velocity: np.ndarray,
        angular_velocity: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        delta_x = self.point - position
        force = self.kp_pos * (delta_x + np.asarray([0.0, 0.0, self.length], dtype=np.float32))
        force = force + self.kd_pos * (-linear_velocity)
        # q_current * inverse(q_target): orientation error in world coordinates,
        # matching the frame of the applied torque and angular velocity.
        w, x, y, z = orientation_wxyz
        c, s = np.cos(self.target_yaw / 2.0), np.sin(self.target_yaw / 2.0)
        rotvec = _quat_wxyz_to_rotvec([c * w + s * z, c * x - s * y, c * y + s * x, c * z - s * w])
        torque = -self.kp_ang * rotvec - self.kd_ang * angular_velocity
        self.force = np.asarray(force, dtype=np.float32)
        self.torque = np.asarray(torque, dtype=np.float32)
        return self.force, self.torque

    def as_dict(self) -> dict[str, Any]:
        return {
            "elastic_band_enabled": bool(self.enabled),
            "elastic_band_target_link": self.target_link,
            "elastic_band_target_prim_path": self.target_prim_path,
            "elastic_band_anchor": [float(x) for x in self.point],
            "elastic_band_target_yaw": float(self.target_yaw),
            "elastic_band_length": float(self.length),
            "elastic_band_force": [float(x) for x in self.force],
            "elastic_band_torque": [float(x) for x in self.torque],
            "elastic_band_last_error": self.last_error,
            "elastic_band_max_force": None,
        }


@dataclass
class IsaacJointMappingResult:
    """Mapping from Sonic body order to Isaac articulation DOF indices."""

    sonic_body_joint_names: list[str]
    isaac_dof_names: list[str]
    sonic_body_to_isaac_dof: list[int]

    def as_rows(self) -> list[dict[str, Any]]:
        rows = []
        for sonic_idx, isaac_idx in enumerate(self.sonic_body_to_isaac_dof):
            rows.append(
                {
                    "sonic_body_index": sonic_idx,
                    "sonic_body_joint": self.sonic_body_joint_names[sonic_idx],
                    "isaac_dof_index": isaac_idx,
                    "isaac_dof_name": self.isaac_dof_names[isaac_idx],
                }
            )
        return rows


@dataclass
class IsaacRobotState:
    """Small serializable snapshot of the Isaac G1 articulation state."""

    joint_position: list[float] = field(default_factory=list)
    joint_velocity: list[float] = field(default_factory=list)
    root_position: list[float] = field(default_factory=list)
    root_quaternion: list[float] = field(default_factory=list)
    root_linear_velocity: list[float] = field(default_factory=list)
    root_angular_velocity: list[float] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "joint_position": self.joint_position,
            "joint_velocity": self.joint_velocity,
            "root_position": self.root_position,
            "root_quaternion": self.root_quaternion,
            "root_linear_velocity": self.root_linear_velocity,
            "root_angular_velocity": self.root_angular_velocity,
        }


@dataclass
class IsaacBackspaceSnapshot:
    """State restored by Isaac's MuJoCo-window Backspace equivalent."""

    source: str
    root_position: np.ndarray
    root_quaternion: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    root_linear_velocity: np.ndarray
    root_angular_velocity: np.ndarray
    scene_xform_ops: dict[str, list[dict[str, Any]]] = field(default_factory=dict)


class IsaacSimulationBackend:
    """Run the G1 scene and bridge Sonic WBC commands to Isaac Sim."""

    def __init__(
        self,
        robot_model: str | None = None,
        usd_path: str | Path | None = None,
        prim_path: str | None = None,
        articulation_root_path: str | None = None,
        headless: bool = True,
        physics_dt: float = 1.0 / 200.0,
        rendering_dt: float = 1.0 / 60.0,
        startup_steps: int = 5,
        dds_bridge: Any | None = None,
        scene_layer_path: str | Path | None = None,
        collision_layer_path: str | Path | None = None,
        experience_path: str | Path | None = None,
        forward_k_to_deploy: bool = False,
    ):
        self.robot_asset = get_robot_asset(robot_model)
        self.robot_model = self.robot_asset.name
        self.usd_path = Path(usd_path).expanduser().resolve() if usd_path else self.robot_asset.usd_path
        self.experience_path = (
            Path(experience_path).expanduser().resolve() if experience_path else None
        )
        self.scene_layer_path = (
            Path(scene_layer_path).expanduser().resolve() if scene_layer_path else None
        )
        self.collision_layer_path = (
            Path(collision_layer_path).expanduser().resolve() if collision_layer_path else None
        )
        self.prim_path = prim_path or self.robot_asset.prim_path
        self.requested_articulation_root_path = articulation_root_path or self.robot_asset.articulation_root
        self.headless = headless
        self.physics_dt = physics_dt
        self.rendering_dt = rendering_dt
        self.startup_steps = startup_steps

        self.simulation_app = None
        self.world = None
        self.ground_plane = None
        self.scene_reset_bodies: list[Any] = []
        self.episode_collector = None
        self._scene_randomization: dict[str, tuple[np.ndarray, np.ndarray, float]] = {}
        self.articulation = None
        self.articulation_root_path: str | None = None
        self.joint_mapping: IsaacJointMappingResult | None = None
        self.dds_bridge = dds_bridge
        self.forward_k_to_deploy = bool(forward_k_to_deploy)
        self.scene_layer_report: dict[str, Any] = {}
        self.collision_layer_report: dict[str, Any] = {}
        self.robot_asset_report: dict[str, Any] = self.robot_asset.as_dict()
        self.joint_mapping_report: dict[str, Any] = {}
        self.image_publish_process: IsaacImagePublisher | None = None
        self.native_xr_robot_follower: IsaacNativeXrRobotFollower | None = None
        self.realtime_state_publisher: IsaacRealtimeStatePublisher | None = None
        self.deploy_relay: IsaacDeployRelay | None = None
        self.keyboard_subscriber: IsaacKeyboardCommandSubscriber | None = None
        self.elastic_band = IsaacElasticBandState(
            target_link=Path(self.robot_asset.root_body_prim_path).name,
            target_prim_path=self.robot_asset.root_body_prim_path,
        )
        self._elastic_band_rigid_prim = None
        self._elastic_band_trace_path: Path | None = None
        self._elastic_band_trace_header_written = False
        self._reset_trace_path: Path | None = None
        self._reset_trace_header_written = False
        self._backspace_snapshot: IsaacBackspaceSnapshot | None = None
        self._control_lock = RLock()
        self._running = False
        self._sim_time = 0.0

    def start(self) -> None:
        """Launch Isaac, load G1 USD, initialize physics, and validate joints."""

        if self._running:
            return

        if not self.usd_path.exists():
            raise FileNotFoundError(f"G1 USD not found: {self.usd_path}")
        if self.usd_path == self.robot_asset.usd_path:
            self.robot_asset_report = self.robot_asset.as_dict()
        else:
            self.robot_asset_report = self.robot_asset.as_dict()
            self.robot_asset_report["usd_path_override"] = str(self.usd_path)

        # Isaac imports must happen after SimulationApp starts and must stay
        # lazy so normal Python can import this module for static checks.
        if ensure_isaac_assets_bypass_proxy():
            print(f"[IsaacBackend] assets host bypasses local proxy: {ISAAC_ASSETS_HOST}")
        from isaacsim import SimulationApp

        launch_config = {"headless": self.headless}
        experience = str(self.experience_path) if self.experience_path else ""
        if experience:
            print(f"[IsaacBackend] experience: {experience}")
        self.simulation_app = SimulationApp(launch_config, experience=experience)

        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim, SingleArticulation, SingleRigidPrim
        from isaacsim.core.utils.prims import get_articulation_root_api_prim_path
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, update_stage
        from pxr import UsdPhysics

        self.world = World(stage_units_in_meters=1.0, backend="numpy", device="cpu")
        self.world.set_simulation_dt(physics_dt=self.physics_dt, rendering_dt=self.rendering_dt)
        self.ground_plane = self.world.scene.add_default_ground_plane()

        if self.scene_layer_path is not None:
            self.apply_scene_layer(self.scene_layer_path)

        print(f"[IsaacBackend] robot model: {self.robot_model}")
        print(f"[IsaacBackend] robot_usd_path: {self.usd_path}")
        print(f"[IsaacBackend] loading USD: {self.usd_path}")
        print(f"[IsaacBackend] reference prim: {self.prim_path}")
        print(f"[IsaacBackend] configured articulation root: {self.requested_articulation_root_path}")
        add_reference_to_stage(usd_path=str(self.usd_path), prim_path=self.prim_path)
        if self.collision_layer_path is not None:
            self.apply_collision_layer(self.collision_layer_path)

        for _ in range(60):
            update_stage()
            if omni.usd.get_context().get_stage_loading_status()[2] == 0:
                break

        self.articulation_root_path = self._resolve_articulation_root_path(
            get_articulation_root_api_prim_path=get_articulation_root_api_prim_path,
            get_current_stage=get_current_stage,
            usd_physics=UsdPhysics,
        )
        print(f"[IsaacBackend] articulation root: {self.articulation_root_path}")

        self.articulation = self.world.scene.add(
            SingleArticulation(prim_path=self.articulation_root_path, name="sonic_g1")
        )
        self._elastic_band_rigid_prim = self.world.scene.add(
            RigidPrim(
                prim_paths_expr=self.elastic_band.target_prim_path,
                name="sonic_g1_elastic_band_target",
            )
        )
        # Tagged task objects use PhysX state for reset, rather than USD xforms
        # alone, so moving bodies also have their velocities cleared.
        self.scene_reset_bodies = []
        self._scene_randomization = {}
        for prim in get_current_stage().Traverse():
            if not prim.GetAttribute("wbc:resetOnBackspace").Get():
                continue
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(f"Backspace reset requires a rigid body: {prim.GetPath()}")
            self.scene_reset_bodies.append(
                self.world.scene.add(
                    SingleRigidPrim(
                        prim_path=str(prim.GetPath()),
                        name=f"wbc_scene_body_{len(self.scene_reset_bodies)}",
                    )
                )
            )
            radius = prim.GetAttribute("wbc:randomizationRadius").Get()
            if radius is not None:
                radius = float(radius)
                if not np.isfinite(radius) or radius < 0:
                    raise ValueError(f"Invalid randomization radius on {prim.GetPath()}: {radius}")
                body = self.scene_reset_bodies[-1]
                # Radius zero explicitly fixes the authored spawn pose.
                # Snapshot recapture must never move the sampling center.
                position, orientation = body.get_world_pose()
                self._scene_randomization[body.prim_path] = (
                    np.asarray(position).copy(), np.asarray(orientation).copy(), radius
                )
        if self.episode_collector is not None:
            self.episode_collector.prepare_scene()
        self.world.reset()
        self._reset_scene_bodies(randomized_only=True)

        for _ in range(max(0, self.startup_steps)):
            self.world.step(render=not self.headless)
            self._sim_time += self.physics_dt

        self._running = True
        self._validate_joint_mapping()
        self._validate_configured_prims()
        self._start_native_xr_robot_follower()
        self._capture_backspace_snapshot(source="isaac_world_reset")
        self._print_robot_summary()

    def _start_native_xr_robot_follower(self) -> None:
        """Enable native XR following only for the explicit VR experience."""

        if not is_native_xr_vr_experience(self.experience_path):
            return
        follower = IsaacNativeXrRobotFollower(self)
        if follower.start():
            self.native_xr_robot_follower = follower

    def apply_scene_layer(self, scene_layer_path: str | Path) -> dict[str, Any]:
        """Load an optional SonicStar task scene USD layer.

        The layer is sublayered into the anonymous runtime stage before the G1
        USD is referenced. Robot USD files and collision calibration layers are
        left separate.
        """

        layer_path = Path(scene_layer_path).expanduser().resolve()
        report: dict[str, Any] = {
            "requested_layer": str(layer_path),
            "applied": False,
            "errors": [],
        }
        if not layer_path.exists():
            report["errors"].append(f"scene layer not found: {layer_path}")
            self.scene_layer_report = report
            raise FileNotFoundError(report["errors"][-1])
        try:
            from isaacsim.core.utils.stage import get_current_stage
            from pxr import Sdf

            layer = Sdf.Layer.FindOrOpen(str(layer_path))
            if layer is None:
                raise RuntimeError(f"failed to open scene layer: {layer_path}")
            stage = get_current_stage()
            root_layer = stage.GetRootLayer()
            sublayers = list(root_layer.subLayerPaths)
            if str(layer_path) not in sublayers:
                root_layer.subLayerPaths.append(str(layer_path))
            report["applied"] = True
            report["root_layer"] = root_layer.identifier
            report["sublayer_count"] = len(root_layer.subLayerPaths)
            report["layer_default_prim"] = str(layer.defaultPrim)
            report["layer_root_prims"] = [str(prim.path) for prim in layer.rootPrims]
        except Exception as exc:
            report["errors"].append(repr(exc))
            self.scene_layer_report = report
            raise

        self.scene_layer_report = report
        print(f"[IsaacBackend] scene layer: {report}")
        return report

    def apply_collision_layer(self, collision_layer_path: str | Path) -> dict[str, Any]:
        """Load an optional Isaac-only USD layer for collision calibration.

        The layer is sublayered into the anonymous runtime stage. Source robot
        USD files are not edited or saved.
        """

        layer_path = Path(collision_layer_path).expanduser().resolve()
        report: dict[str, Any] = {
            "requested_layer": str(layer_path),
            "applied": False,
            "errors": [],
        }
        if not layer_path.exists():
            report["errors"].append(f"collision layer not found: {layer_path}")
            self.collision_layer_report = report
            raise FileNotFoundError(report["errors"][-1])
        try:
            from isaacsim.core.utils.stage import get_current_stage
            from pxr import Sdf

            layer = Sdf.Layer.FindOrOpen(str(layer_path))
            if layer is None:
                raise RuntimeError(f"failed to open collision layer: {layer_path}")
            stage = get_current_stage()
            root_layer = stage.GetRootLayer()
            sublayers = list(root_layer.subLayerPaths)
            if str(layer_path) not in sublayers:
                root_layer.subLayerPaths.append(str(layer_path))
            report["applied"] = True
            report["root_layer"] = root_layer.identifier
            report["sublayer_count"] = len(root_layer.subLayerPaths)
            if self.prim_path != "/World/G1":
                report["warning"] = (
                    "default calibration layer authors /World/G1 paths; "
                    f"current prim_path={self.prim_path!r}"
                )
        except Exception as exc:
            report["errors"].append(repr(exc))
            self.collision_layer_report = report
            raise

        self.collision_layer_report = report
        print(f"[IsaacBackend] collision layer: {report}")
        return report

    def stop(self) -> None:
        self.close()

    def close(self) -> None:
        """Close Isaac resources."""

        self._running = False
        if self.episode_collector is not None:
            self.episode_collector.close()
            self.episode_collector = None
        if self.keyboard_subscriber is not None:
            self.keyboard_subscriber.stop()
            self.keyboard_subscriber = None
        if self.deploy_relay is not None:
            self.deploy_relay.stop()
            self.deploy_relay = None
        self.stop_realtime_state_publisher()
        self.stop_image_publish()
        if self.world is not None:
            try:
                self.world.stop()
            except Exception:
                pass
        if self.simulation_app is not None:
            self.simulation_app.close()
        self.world = None
        self.articulation = None
        self.simulation_app = None

    def initialize_articulation_state_for_control(
        self,
        *,
        joint_positions: Any,
        root_position: Any,
        root_orientation: Any,
        configure_drives: Any | None = None,
        target_writer: Any | None = None,
        state_audit_writer: Any | None = None,
        reset_world: bool = True,
        startup_warmup_steps: int = 20,
        state_commit_steps: int = 20,
        target_commit_steps: int = 20,
        render: bool = False,
    ) -> dict[str, Any]:
        """Canonical Isaac control initialization sequence.

        The sequence is intentionally separated into reset, drive config,
        state teleport, velocity clearing, state commit, target activation,
        and target/contact settle.  It does not change DDS, WBC, FluxVLA, or
        the source USD.
        """

        self._require_started()
        if self.world is None or self.articulation is None:
            raise RuntimeError("Isaac articulation must exist before control initialization")

        import numpy as np

        joint_positions_array = np.asarray(joint_positions, dtype=np.float32)
        root_position_array = np.asarray(root_position, dtype=np.float32)
        root_orientation_array = np.asarray(root_orientation, dtype=np.float32)
        report: dict[str, Any] = {
            "sequence": [],
            "reset_world": bool(reset_world),
            "startup_warmup_steps": int(startup_warmup_steps),
            "state_commit_steps": int(state_commit_steps),
            "target_commit_steps": int(target_commit_steps),
        }

        def step_many(label: str, count: int) -> None:
            count = max(0, int(count))
            for _ in range(count):
                self.world.step(render=render)
                self._sim_time += self.physics_dt
            report["sequence"].append({"operation": label, "steps": count})

        if reset_world:
            with self._control_lock:
                self.world.reset()
                self._reset_scene_bodies(randomized_only=True)
                self._sim_time = 0.0
                self._validate_joint_mapping()
                report["sequence"].append({"operation": "reset", "steps": 0})

        step_many("startup_warmup", startup_warmup_steps)

        if configure_drives is not None:
            drive_rows = configure_drives()
            report["drive_rows"] = len(drive_rows) if drive_rows is not None else 0
            report["sequence"].append({"operation": "configure_drives", "steps": 0})

        with self._control_lock:
            self.articulation.set_world_pose(position=root_position_array, orientation=root_orientation_array)
            report["sequence"].append({"operation": "set_world_pose", "steps": 0})

            self.articulation.set_joint_positions(joint_positions_array)
            report["sequence"].append({"operation": "set_joint_positions", "steps": 0})
            if state_audit_writer is not None:
                state_audit_writer(
                    label="T0_after_set_joint_positions",
                    simulation_time=self._sim_time,
                    state=self.read_joint_state(),
                    target_joint_positions_by_isaac=joint_positions_array,
                )

            zero_joint_velocities = np.zeros_like(joint_positions_array, dtype=np.float32)
            zero_root_velocity = np.zeros(3, dtype=np.float32)
            self.articulation.set_joint_velocities(zero_joint_velocities)
            self.articulation.set_linear_velocity(zero_root_velocity)
            self.articulation.set_angular_velocity(zero_root_velocity)
            self._capture_backspace_snapshot(
                source="canonical_control_init",
                root_position=root_position_array,
                root_quaternion=root_orientation_array,
                joint_position=joint_positions_array,
                joint_velocity=zero_joint_velocities,
                root_linear_velocity=zero_root_velocity,
                root_angular_velocity=zero_root_velocity,
            )
            report["sequence"].append({"operation": "clear_velocities", "steps": 0})

        step_many("state_commit", state_commit_steps)
        if state_audit_writer is not None:
            state_audit_writer(
                label="T1_after_state_commit",
                simulation_time=self._sim_time,
                state=self.read_joint_state(),
                target_joint_positions_by_isaac=joint_positions_array,
            )

        if target_writer is not None:
            target_writer(joint_positions_array)
        else:
            articulation_view = getattr(self.articulation, "_articulation_view", None)
            view_set_targets = getattr(articulation_view, "set_joint_position_targets", None)
            if view_set_targets is not None:
                view_set_targets(np.expand_dims(joint_positions_array, axis=0))
            elif hasattr(self.articulation, "set_joint_position_targets"):
                self.articulation.set_joint_position_targets(joint_positions_array)
            else:
                raise RuntimeError("Isaac articulation does not expose a joint target writer")
        report["sequence"].append({"operation": "set_joint_position_targets", "steps": 0})

        step_many("target_commit", target_commit_steps)
        return report

    def step(self, render: bool | None = None) -> None:
        self._require_started()
        should_render = (not self.headless) if render is None else render
        with self._control_lock:
            self._apply_elastic_band_force()
            self.world.step(render=should_render)
            self._sim_time += self.physics_dt
            if self.native_xr_robot_follower is not None:
                self.native_xr_robot_follower.observe_after_step()

    def render(self) -> None:
        self.step(render=True)

    def start_image_publish_subprocess(
        self,
        *,
        start_method: str = "spawn",
        camera_port: int = 5555,
        camera_name: str = "ego_view",
        camera_source: str = "robot_ego",
        image_dt: float = 1.0 / 60.0,
        width: int = 640,
        height: int = 480,
        local_translation: tuple[float, float, float] = (0.06, 0.0, 0.25),
        ego_camera_intrinsics: tuple[float, float, float, float] | None = None,
    ) -> None:
        """Start Isaac's MuJoCo-compatible camera publisher.

        The name mirrors the MuJoCo simulator API even though Isaac publishes
        in-process; the wire protocol is identical to MuJoCo's 5555 publisher.
        ``start_method`` is accepted for API compatibility and intentionally
        unused.
        """

        del start_method
        self._require_started()
        if is_native_xr_vr_experience(self.experience_path):
            print(
                "[IsaacBackend] native XR VR selected: ignoring ego_view/5555 publisher; "
                "the Pico view is the native G1-following XR anchor"
            )
            return
        if self.image_publish_process is not None:
            return
        publisher = IsaacImagePublisher(
            port=camera_port,
            camera_name=camera_name,
            camera_source=camera_source,
            width=width,
            height=height,
            image_dt=image_dt,
            local_translation=local_translation,
            ego_camera_intrinsics=ego_camera_intrinsics,
        )
        publisher.start(self)
        self.image_publish_process = publisher

    def stop_image_publish(self) -> None:
        if self.image_publish_process is not None:
            self.image_publish_process.stop()
        self.image_publish_process = None

    def start_realtime_state_publisher(
        self,
        *,
        port: int = 5557,
        topic: str = "g1_debug",
    ) -> None:
        """Start optional Isaac-side 5557 publisher.

        Do not enable this while ``g1_deploy_onnx_ref`` is running, because the
        C++ process normally owns 5557 and emits the canonical GR00T payload.
        """

        self._require_started()
        if self.realtime_state_publisher is not None:
            return
        publisher = IsaacRealtimeStatePublisher(port=port, topic=topic)
        publisher.start(self)
        self.realtime_state_publisher = publisher

    def stop_realtime_state_publisher(self) -> None:
        if self.realtime_state_publisher is not None:
            self.realtime_state_publisher.stop()
        self.realtime_state_publisher = None

    def spin(
        self,
        *,
        frequency_hz: int = 500,
        duration_s: float | None = None,
        dds_bridge: Any | None = None,
        log_dir: str | Path | None = None,
    ) -> Any:
        """Run the Isaac DDS bridge loop.

        Loop body:

            physics step -> read joint state -> publish lowstate/IMU -> receive lowcmd -> apply target

        This does not start FluxVLA, WBC, or evaluation logic. Optional camera
        and state publishers can be started before entering the loop.
        """

        self._require_started()
        if frequency_hz <= 0:
            raise ValueError(f"frequency_hz must be positive, got {frequency_hz}")

        bridge = dds_bridge or self.dds_bridge
        owns_bridge = bridge is None
        if bridge is None:
            from gear_sonic.robot_interface.isaac_unitree_bridge import IsaacUnitreeBridge

            bridge = IsaacUnitreeBridge(
                articulation=self.articulation,
                joint_mapping=self.joint_mapping,
                log_dir=log_dir,
            )
        else:
            bridge.articulation = self.articulation
            bridge.joint_mapping = self.joint_mapping
            if log_dir is not None and getattr(bridge, "log_dir", None) is None:
                bridge.log_dir = Path(log_dir)

        self.dds_bridge = bridge
        bridge.start()
        if self.forward_k_to_deploy:
            self.deploy_relay = IsaacDeployRelay()
            self.deploy_relay.start()
        self.keyboard_subscriber = IsaacKeyboardCommandSubscriber(
            self, forward_k_to_deploy=self.forward_k_to_deploy
        )
        self.keyboard_subscriber.start()
        if log_dir is not None:
            self._elastic_band_trace_path = Path(log_dir) / "elastic_band_trace.csv"
            self._reset_trace_path = Path(log_dir) / "reset_trace.csv"
        period_s = 1.0 / float(frequency_hz)
        start_time = time.monotonic()
        next_tick = start_time
        active_command = None
        try:
            while self._running:
                if duration_s is not None and time.monotonic() - start_time >= duration_s:
                    break

                collector = self.episode_collector
                if collector is not None:
                    collector.before_step()
                frozen = collector is not None and collector.frozen
                render_for_publishers = self.image_publish_process is not None or collector is not None
                if frozen:
                    # Keep GUI and DDS responsive without advancing the reset pose
                    # or applying stale teleop commands while waiting for manual A+X.
                    self.world.render()
                    active_command = None
                else:
                    self.step(render=(not self.headless) or render_for_publishers)
                performance_monitor = getattr(bridge, "performance_monitor", None)
                if performance_monitor is not None:
                    performance_monitor.record_physics_step()
                state = self.read_joint_state()
                if collector is not None and not frozen:
                    collector.after_step(state, bridge)
                if self.image_publish_process is not None:
                    self.image_publish_process.maybe_publish(self)
                if self.realtime_state_publisher is not None:
                    self.realtime_state_publisher.publish(self, state)
                obs = state.as_dict()
                obs["time"] = self._sim_time
                bridge.publish_low_state(obs)
                command = bridge.receive_low_cmd()
                if command is not None:
                    active_command = command
                elif active_command is not None:
                    active_command = bridge.peek_low_cmd() or active_command

                if collector is not None and (collector.frozen or (
                    active_command is not None and active_command.received_time < collector.resume_wall_time
                )):
                    active_command = None
                if active_command is not None and not bridge.is_command_current(active_command):
                    active_command = None
                if active_command is not None:
                    bridge.apply_joint_command(active_command)
                if self.keyboard_subscriber is not None:
                    # Mirror MuJoCo viewer-only keys from send_keyboard_cmd.py:
                    # 9 toggles ElasticBand, Backspace performs runtime reset.
                    self.keyboard_subscriber.poll_once()
                if self.deploy_relay is not None:
                    self.deploy_relay.poll_once()

                next_tick += period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()
                    # Yield to DDS callbacks if the step loop overruns its period.
                    time.sleep(0)
        finally:
            if self.keyboard_subscriber is not None:
                self.keyboard_subscriber.stop()
                self.keyboard_subscriber = None
            if self.deploy_relay is not None:
                self.deploy_relay.stop()
                self.deploy_relay = None
            if owns_bridge:
                bridge.stop()

        return bridge.stats

    def read_joint_state(self) -> IsaacRobotState:
        """Read joint and root state from the Isaac articulation."""

        self._require_started()
        joint_position = self._tolist(self.articulation.get_joint_positions())
        joint_velocity = self._tolist(self.articulation.get_joint_velocities())
        root_position, root_quaternion = self.articulation.get_world_pose()
        root_linear_velocity = self._tolist(self.articulation.get_linear_velocity())
        root_angular_velocity = self._tolist(self.articulation.get_angular_velocity())

        return IsaacRobotState(
            joint_position=self._tolist(rootless(joint_position)),
            joint_velocity=self._tolist(rootless(joint_velocity)),
            root_position=self._tolist(root_position),
            root_quaternion=self._tolist(root_quaternion),
            root_linear_velocity=root_linear_velocity,
            root_angular_velocity=root_angular_velocity,
        )

    def keyboard(self, key: str) -> None:
        normalized = str(key).lower()
        if normalized == "9":
            self.set_elastic_band_enabled(not self.elastic_band.enabled)
            return
        if normalized in ("backspace", "\b", "\x7f"):
            if self.episode_collector is not None and self.episode_collector.request_manual_reset():
                return
            self.mujoco_backspace_reset()
            return
        print(f"[IsaacBackend] keyboard command not handled by Isaac backend: {key!r}")

    def set_elastic_band_enabled(self, enabled: bool) -> bool:
        with self._control_lock:
            self.elastic_band.enabled = bool(enabled)
            if not self.elastic_band.enabled:
                self.elastic_band.force = np.zeros(3, dtype=np.float32)
                self.elastic_band.torque = np.zeros(3, dtype=np.float32)
            print(f"ElasticBand enable: {self.elastic_band.enabled}")
        return True

    def mujoco_backspace_reset(self) -> None:
        """MuJoCo viewer Backspace equivalent, without restarting control processes.

        MuJoCo calls ``mj_resetData`` + clears ``xfrc_applied`` + ``mj_forward``.
        For Isaac, keep the running World/DDS/lowcmd bridge intact and restore
        the captured model/canonical initial robot state plus task-object xforms.
        ElasticBand enable state is intentionally preserved; only the currently
        applied force/torque cache is zeroed before the next physics step
        recomputes it, matching MuJoCo's ``xfrc_applied[:] = 0`` behavior.
        """

        self._require_started()
        if self.articulation is None:
            raise RuntimeError("Isaac articulation is unavailable")
        with self._control_lock:
            if self._backspace_snapshot is None:
                self._capture_backspace_snapshot(source="lazy_pre_backspace")
            assert self._backspace_snapshot is not None
            snapshot = self._backspace_snapshot
            before = self.read_joint_state().as_dict()
            band_before = bool(self.elastic_band.enabled)
            self._restore_scene_xform_ops(snapshot.scene_xform_ops)
            self._reset_scene_bodies()
            self.articulation.set_world_pose(
                position=np.asarray(snapshot.root_position, dtype=np.float32),
                orientation=np.asarray(snapshot.root_quaternion, dtype=np.float32),
            )
            self.articulation.set_joint_positions(np.asarray(snapshot.joint_position, dtype=np.float32))
            self.articulation.set_joint_velocities(np.asarray(snapshot.joint_velocity, dtype=np.float32))
            self.articulation.set_linear_velocity(np.asarray(snapshot.root_linear_velocity, dtype=np.float32))
            self.articulation.set_angular_velocity(np.asarray(snapshot.root_angular_velocity, dtype=np.float32))
            if self.dds_bridge is not None:
                self.dds_bridge.discard_episode_commands()
            self.elastic_band.force = np.zeros(3, dtype=np.float32)
            self.elastic_band.torque = np.zeros(3, dtype=np.float32)
            after = self.read_joint_state().as_dict()
            self._write_reset_trace(
                command="keyboard_backspace",
                snapshot_source=snapshot.source,
                before=before,
                after=after,
                elastic_band_before=band_before,
                elastic_band_after=bool(self.elastic_band.enabled),
                object_xform_count=sum(len(v) for v in snapshot.scene_xform_ops.values()),
            )
            print(
                "[IsaacBackend] MuJoCo Backspace reset applied "
                f"snapshot={snapshot.source!r} elastic_band_preserved={self.elastic_band.enabled}"
            )

    def _reset_scene_bodies(self, *, randomized_only: bool = False) -> None:
        """Reset tagged bodies; sample configured disks uniformly by area."""
        for body in self.scene_reset_bodies:
            randomization = self._scene_randomization.get(body.prim_path)
            if randomization is not None:
                center, orientation, radius = randomization
                # Same sampling rule as FluxBisim/tasks/utils.py.
                position = center.copy()
                if radius > 0:
                    theta = np.random.uniform(0.0, 2.0 * np.pi)
                    distance = radius * np.sqrt(np.random.uniform(0.0, 1.0))
                    position[0] += distance * np.cos(theta)
                    position[1] += distance * np.sin(theta)
                body.set_default_state(
                    position=position,
                    orientation=orientation.copy(),
                    linear_velocity=np.zeros(3, dtype=np.float32),
                    angular_velocity=np.zeros(3, dtype=np.float32),
                )
            elif randomized_only:
                continue
            body.post_reset()

    def _capture_backspace_snapshot(
        self,
        *,
        source: str,
        root_position: Any | None = None,
        root_quaternion: Any | None = None,
        joint_position: Any | None = None,
        joint_velocity: Any | None = None,
        root_linear_velocity: Any | None = None,
        root_angular_velocity: Any | None = None,
    ) -> None:
        if self.articulation is None:
            return
        if root_position is None or root_quaternion is None:
            root_position, root_quaternion = self.articulation.get_world_pose()
        if joint_position is None:
            joint_position = self.articulation.get_joint_positions()
        if joint_velocity is None:
            joint_velocity = self.articulation.get_joint_velocities()
        if root_linear_velocity is None:
            root_linear_velocity = self.articulation.get_linear_velocity()
        if root_angular_velocity is None:
            root_angular_velocity = self.articulation.get_angular_velocity()

        for body in self.scene_reset_bodies:
            if body.prim_path in self._scene_randomization:
                continue
            position, orientation = body.get_world_pose()
            body.set_default_state(
                position=np.asarray(position).copy(),
                orientation=np.asarray(orientation).copy(),
                linear_velocity=np.zeros(3, dtype=np.float32),
                angular_velocity=np.zeros(3, dtype=np.float32),
            )

        self._backspace_snapshot = IsaacBackspaceSnapshot(
            source=str(source),
            root_position=np.asarray(root_position, dtype=np.float32).reshape(3).copy(),
            root_quaternion=np.asarray(root_quaternion, dtype=np.float32).reshape(4).copy(),
            joint_position=np.asarray(joint_position, dtype=np.float32).reshape(-1).copy(),
            joint_velocity=np.asarray(joint_velocity, dtype=np.float32).reshape(-1).copy(),
            root_linear_velocity=np.asarray(root_linear_velocity, dtype=np.float32).reshape(3).copy(),
            root_angular_velocity=np.asarray(root_angular_velocity, dtype=np.float32).reshape(3).copy(),
            scene_xform_ops=self._capture_scene_xform_ops(),
        )

    def _capture_scene_xform_ops(self) -> dict[str, list[dict[str, Any]]]:
        if self.scene_layer_path is None:
            return {}
        try:
            from isaacsim.core.utils.stage import get_current_stage
            from pxr import UsdGeom

            stage = get_current_stage()
            root_path = "/World/SonicStarTask"
            root_prim = stage.GetPrimAtPath(root_path)
            if not root_prim or not root_prim.IsValid():
                return {}
            result: dict[str, list[dict[str, Any]]] = {}
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if path != root_path and not path.startswith(root_path + "/"):
                    continue
                xformable = UsdGeom.Xformable(prim)
                if not xformable:
                    continue
                ops = []
                for op in xformable.GetOrderedXformOps():
                    attr = op.GetAttr()
                    ops.append({"attr_name": attr.GetName(), "value": attr.Get()})
                if ops:
                    result[path] = ops
            return result
        except Exception as exc:
            print(f"[IsaacBackend] warning: failed to capture scene xforms for Backspace: {exc!r}")
            return {}

    def _restore_scene_xform_ops(self, scene_xform_ops: dict[str, list[dict[str, Any]]]) -> None:
        if not scene_xform_ops:
            return
        try:
            from isaacsim.core.utils.stage import get_current_stage

            stage = get_current_stage()
            for path, ops in scene_xform_ops.items():
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid():
                    continue
                for op in ops:
                    attr = prim.GetAttribute(op["attr_name"])
                    if attr:
                        attr.Set(op["value"])
        except Exception as exc:
            print(f"[IsaacBackend] warning: failed to restore scene xforms for Backspace: {exc!r}")

    def _write_reset_trace(
        self,
        *,
        command: str,
        snapshot_source: str,
        before: dict[str, Any],
        after: dict[str, Any],
        elastic_band_before: bool,
        elastic_band_after: bool,
        object_xform_count: int,
    ) -> None:
        if self._reset_trace_path is None:
            return
        row = {
            "timestamp": f"{time.time():.9f}",
            "sim_time": f"{self._sim_time:.9f}",
            "physics_step": int(round(self._sim_time / self.physics_dt)) if self.physics_dt else 0,
            "command": command,
            "snapshot_source": snapshot_source,
            "root_pose_before": repr(
                {
                    "position": before.get("root_position"),
                    "quaternion": before.get("root_quaternion"),
                }
            ),
            "root_pose_after": repr(
                {
                    "position": after.get("root_position"),
                    "quaternion": after.get("root_quaternion"),
                }
            ),
            "joint_q_before": repr(before.get("joint_position")),
            "joint_q_after": repr(after.get("joint_position")),
            "joint_dq_before": repr(before.get("joint_velocity")),
            "joint_dq_after": repr(after.get("joint_velocity")),
            "root_linear_velocity_before": repr(before.get("root_linear_velocity")),
            "root_linear_velocity_after": repr(after.get("root_linear_velocity")),
            "root_angular_velocity_before": repr(before.get("root_angular_velocity")),
            "root_angular_velocity_after": repr(after.get("root_angular_velocity")),
            "object_xform_count": int(object_xform_count),
            "elastic_band_before": bool(elastic_band_before),
            "elastic_band_after": bool(elastic_band_after),
            "control_process_restarted": False,
            "dds_reconnected": False,
        }
        self._reset_trace_path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self._reset_trace_path.exists() or not self._reset_trace_header_written
        with self._reset_trace_path.open("a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
                self._reset_trace_header_written = True
            writer.writerow(row)

    def _apply_elastic_band_force(self) -> None:
        if not self.elastic_band.enabled:
            self._write_elastic_band_trace()
            return
        if self.articulation is None:
            return
        if self._elastic_band_rigid_prim is None:
            self.elastic_band.last_error = "elastic band rigid prim view is not initialized"
            self._write_elastic_band_trace()
            return
        try:
            root_position, root_quaternion = self.articulation.get_world_pose()
            position = np.asarray(root_position, dtype=np.float32).reshape(3)
            orientation = np.asarray(root_quaternion, dtype=np.float32).reshape(4)
            linear_velocity = np.asarray(self.articulation.get_linear_velocity(), dtype=np.float32).reshape(3)
            angular_velocity = np.asarray(self.articulation.get_angular_velocity(), dtype=np.float32).reshape(3)
            force, torque = self.elastic_band.compute(
                position=position,
                orientation_wxyz=orientation,
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
            )
            self._elastic_band_rigid_prim.apply_forces_and_torques_at_pos(
                forces=np.expand_dims(force, axis=0),
                torques=np.expand_dims(torque, axis=0),
                positions=np.expand_dims(position, axis=0),
                is_global=True,
            )
            self.elastic_band.last_error = None
        except Exception as exc:
            self.elastic_band.last_error = repr(exc)
        self._write_elastic_band_trace()

    def _write_elastic_band_trace(self) -> None:
        if self._elastic_band_trace_path is None:
            return
        row = {
            "timestamp": f"{time.time():.9f}",
            "sim_time": f"{self._sim_time:.9f}",
            "physics_step": int(round(self._sim_time / self.physics_dt)) if self.physics_dt else 0,
            "elastic_band_enabled": bool(self.elastic_band.enabled),
            "elastic_band_target_link": self.elastic_band.target_link,
            "elastic_band_target_prim_path": self.elastic_band.target_prim_path,
            "elastic_band_anchor": repr([float(x) for x in self.elastic_band.point]),
            "elastic_band_force": repr([float(x) for x in self.elastic_band.force]),
            "elastic_band_torque": repr([float(x) for x in self.elastic_band.torque]),
            "elastic_band_last_error": self.elastic_band.last_error or "",
        }
        self._elastic_band_trace_path.parent.mkdir(parents=True, exist_ok=True)
        write_header = (
            not self._elastic_band_trace_path.exists()
            or not self._elastic_band_trace_header_written
        )
        with self._elastic_band_trace_path.open("a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            if write_header:
                writer.writeheader()
                self._elastic_band_trace_header_written = True
            writer.writerow(row)

    @property
    def dof_names(self) -> list[str]:
        self._require_started()
        return list(self.articulation.dof_names)

    @property
    def num_dof(self) -> int:
        self._require_started()
        return int(self.articulation.num_dof)

    def _require_started(self) -> None:
        if not self._running or self.world is None or self.articulation is None:
            raise RuntimeError("IsaacSimulationBackend.start() must be called first")

    def _resolve_articulation_root_path(
        self,
        get_articulation_root_api_prim_path,
        get_current_stage,
        usd_physics,
    ) -> str:
        if self.requested_articulation_root_path:
            return self.requested_articulation_root_path

        root_path = get_articulation_root_api_prim_path(self.prim_path)
        if root_path:
            return str(root_path)

        stage = get_current_stage()
        candidates = []
        prefix = self.prim_path.rstrip("/") + "/"
        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if prim_path == self.prim_path or prim_path.startswith(prefix):
                if prim.HasAPI(usd_physics.ArticulationRootAPI):
                    candidates.append(prim_path)

        if len(candidates) == 1:
            return candidates[0]
        if not candidates:
            raise RuntimeError(f"no articulation root found below {self.prim_path}")
        raise RuntimeError(f"multiple articulation roots found below {self.prim_path}: {candidates}")

    def _validate_joint_mapping(self) -> None:
        dof_names = self.dof_names
        duplicate_names = sorted({name for name in dof_names if dof_names.count(name) > 1})
        if duplicate_names:
            raise RuntimeError(f"duplicate Isaac DOF names: {duplicate_names}")

        expected_total = int(self.robot_asset.expected_total_dof)
        if len(dof_names) != expected_total:
            raise RuntimeError(
                f"{self.robot_model} DOF count mismatch: expected_total_dof={expected_total} "
                f"actual={len(dof_names)} names={dof_names}"
            )

        expected_body_names = list(self.robot_asset.body_joint_names)
        body_name_set = set(expected_body_names)
        allowed_name_set = body_name_set | set(self.robot_asset.hand_joint_names)
        extra_names = [name for name in dof_names if name not in allowed_name_set]
        if extra_names:
            raise RuntimeError(
                f"{self.robot_model} exposes DOFs outside configured body joints: {extra_names}"
            )
        missing_names = [name for name in expected_body_names if name not in set(dof_names)]
        if missing_names:
            raise RuntimeError(
                f"{self.robot_model} missing configured body joints: {missing_names}"
            )
        missing_hand_names = [name for name in self.robot_asset.hand_joint_names if name not in set(dof_names)]
        if missing_hand_names:
            raise RuntimeError(
                f"{self.robot_model} missing configured hand joints: {missing_hand_names}"
            )

        body_source_names = [name for name in dof_names if name in body_name_set]
        body_source_to_sonic = build_joint_mapping(
            source_joint_names=body_source_names,
            target_joint_names=expected_body_names,
        )
        sonic_body_to_isaac_dof = [
            dof_names.index(body_source_names[source_index]) for source_index in body_source_to_sonic
        ]
        self.joint_mapping = IsaacJointMappingResult(
            sonic_body_joint_names=expected_body_names,
            isaac_dof_names=dof_names,
            sonic_body_to_isaac_dof=sonic_body_to_isaac_dof,
        )
        self.joint_mapping_report = {
            "robot_model": self.robot_model,
            "status": "PASS",
            "expected_body_dof": self.robot_asset.expected_body_dof,
            "expected_total_dof": self.robot_asset.expected_total_dof,
            "actual_total_dof": len(dof_names),
            "hand_dof_available": self.robot_asset.hand_dof_available,
            "rows": self.joint_mapping.as_rows(),
        }

    def _validate_configured_prims(self) -> None:
        from isaacsim.core.utils.stage import get_current_stage

        stage = get_current_stage()
        checked: list[dict[str, Any]] = []
        for label, paths in (
            ("root_body", (self.robot_asset.root_body_prim_path,)),
            ("foot", self.robot_asset.foot_prim_paths),
            ("imu", self.robot_asset.imu_prim_paths),
        ):
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                exists = bool(prim and prim.IsValid())
                checked.append({"label": label, "path": path, "exists": exists})
                if not exists:
                    raise RuntimeError(
                        f"{self.robot_model} configured {label} prim does not exist: {path}"
                    )
        self.robot_asset_report = {
            **self.robot_asset_report,
            "configured_prims": checked,
        }

    def _print_robot_summary(self) -> None:
        print(f"[IsaacBackend] robot asset: {self.robot_asset_report}")
        print(f"[IsaacBackend] DOF count: {self.num_dof}")
        print(f"[IsaacBackend] joint/dof names ({len(self.dof_names)}):")
        for idx, name in enumerate(self.dof_names):
            print(f"  [{idx:02d}] {name}")

        if self.joint_mapping is not None:
            print("[IsaacBackend] Sonic body29 -> Isaac DOF mapping:")
            for row in self.joint_mapping.as_rows():
                print(
                    "  sonic[{sonic_body_index:02d}] {sonic_body_joint} "
                    "-> isaac[{isaac_dof_index:02d}] {isaac_dof_name}".format(**row)
                )

    @staticmethod
    def _tolist(value: Any) -> list[float]:
        if value is None:
            return []
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        if hasattr(value, "tolist"):
            value = value.tolist()
        if isinstance(value, tuple):
            value = list(value)
        return value


def rootless(values: Any) -> Any:
    """Compatibility hook for Isaac joint arrays.

    SingleArticulation joint APIs already return only DOF values, not floating
    root coordinates. The function is intentionally a no-op but keeps the state
    read path explicit.
    """

    return values


def _fit_float_list(values: Any, count: int, *, fill: float = 0.0) -> list[float]:
    array = np.asarray(values, dtype=np.float64).reshape(-1) if values is not None else np.asarray([])
    result = []
    for index in range(int(count)):
        result.append(float(array[index]) if index < array.size else float(fill))
    return result


def _quat_wxyz_to_rotvec(quaternion: Any) -> np.ndarray:
    """Convert a MuJoCo/Isaac wxyz quaternion to a rotation vector."""

    quat = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 0.0 or not np.isfinite(norm):
        return np.zeros(3, dtype=np.float32)
    quat = quat / norm
    if quat[0] < 0.0:
        quat = -quat
    w = float(np.clip(quat[0], -1.0, 1.0))
    xyz = quat[1:4]
    xyz_norm = float(np.linalg.norm(xyz))
    if xyz_norm < 1e-12:
        return np.zeros(3, dtype=np.float32)
    angle = 2.0 * np.arctan2(xyz_norm, w)
    return np.asarray((angle / xyz_norm) * xyz, dtype=np.float32)
