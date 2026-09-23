"""Kitchen success detection, acknowledged teleop boundaries and HDF5 episodes.

Isaac imports are lazy. HDF5 rows contain a post-step observation and the last
command applied before that step; image timestamps/frame IDs remain explicit.
"""

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

import h5py
import numpy as np

from gear_sonic.utils.teleop.episode_control import EpisodeControlClient


class PlacementDetector:
    def __init__(self, dwell=0.6):
        self.dwell = dwell
        self.since = None

    def update(self, sim_time, *, inside, upright, plate_contact, hand_contact,
               fruit_speed, plate_speed, fruit_angular_speed):
        valid = (inside and upright and plate_contact > 0.03 and hand_contact < 0.02
                 and fruit_speed < 0.05 and plate_speed < 0.05 and fruit_angular_speed < 0.5)
        if not valid:
            self.since = None
            return False
        if self.since is None:
            self.since = sim_time
        return sim_time - self.since >= self.dwell


class Hdf5Episode:
    """Append aligned rows; publish a final filename only after flush/fsync."""

    def __init__(self, directory, metadata):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.path = directory / f"episode_{stamp}_{uuid.uuid4().hex[:8]}.partial.hdf5"
        self.file = h5py.File(self.path, "x")
        self.file.attrs["schema_version"] = 1
        self.file.attrs["metadata_json"] = json.dumps(metadata)
        self.file.attrs["complete"] = False
        self.file.attrs["success"] = False
        self.count = 0
        self.datasets = {}

    def append(self, row):
        if self.count and set(row) != set(self.datasets):
            raise ValueError("episode row fields changed")
        for key, value in row.items():
            is_image = key == "observations/images/ego_jpeg"
            value = np.frombuffer(value, dtype=np.uint8) if is_image else np.asarray(value)
            if key not in self.datasets:
                if is_image:
                    dataset = self.file.create_dataset(key, (0,), maxshape=(None,),
                                                       dtype=h5py.vlen_dtype(np.dtype("uint8")))
                    dataset.attrs["encoding"] = "JPEG (decoded RGB)"
                else:
                    dataset = self.file.create_dataset(
                        key, (0, *value.shape), maxshape=(None, *value.shape),
                        dtype=value.dtype, chunks=True,
                    )
                self.datasets[key] = dataset
            dataset = self.datasets[key]
            dataset.resize(self.count + 1, axis=0)
            dataset[self.count] = value
        self.count += 1
        if self.count % 30 == 0:
            self.file.flush()

    def finish(self, outcome):
        self.file.attrs["outcome"] = outcome
        self.file.attrs["success"] = outcome == "success"
        self.file.attrs["num_frames"] = self.count
        self.file.attrs["complete"] = True
        self.file.flush()
        os.fsync(self.file.id.get_vfd_handle())
        self.file.close()
        destination = self.path.with_name(self.path.name.replace(".partial.hdf5", ".hdf5"))
        self.path.rename(destination)
        return destination

    def interrupt(self):
        if self.file.id.valid:
            self.file.attrs["outcome"] = "interrupted"
            self.file.attrs["num_frames"] = self.count
            self.file.flush()
            self.file.close()

    def discard(self):
        """Discard this attempt only; completed episodes are never removed."""
        self.file.close()
        self.path.unlink(missing_ok=True)


class EpisodeEgoCamera:
    """A separate RGB render product with the existing ego pose and optics."""

    def __init__(self, backend, intrinsics):
        from isaacsim.sensors.camera import Camera
        from gear_sonic.simulation_server.isaac_server import (
            IsaacImagePublisher, load_mujoco_camera_spec, mujoco_intrinsic_xyz_euler_to_quat_wxyz,
        )

        spec = load_mujoco_camera_spec()
        self.path = f"{backend.prim_path}/{spec.parent_body_name}/episode_ego_camera"
        self.camera = Camera(prim_path=self.path, name="episode_ego_camera", resolution=(640, 480))
        self.camera.set_local_pose(
            translation=np.asarray(spec.local_position),
            orientation=mujoco_intrinsic_xyz_euler_to_quat_wxyz(np.asarray(spec.local_euler_xyz)),
            camera_axes="usd",
        )
        self.camera.initialize()
        optics = IsaacImagePublisher(ego_camera_intrinsics=intrinsics)
        optics.camera = self.camera
        optics._mujoco_camera_spec = spec
        optics._configure_ego_camera_optics()
        self.metadata = {
            "prim_path": self.path, "resolution": [640, 480],
            "local_position": list(spec.local_position), "local_euler_xyz": list(spec.local_euler_xyz),
            "requested_intrinsics": list(intrinsics), "clipping_range": [0.02, 100.0],
        }
        for _ in range(8):
            backend.world.render()

    def read(self):
        import cv2

        frame = self.camera.get_current_frame()
        rgba = np.asarray(self.camera.get_rgba())
        valid = rgba.shape == (480, 640, 4) and bool(np.any(rgba[:, :, :3]))
        encoded = b""
        if valid:
            ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR),
                                    [cv2.IMWRITE_JPEG_QUALITY, 90])
            valid = bool(ok)
            if ok:
                encoded = jpeg.tobytes()
        return encoded, valid, int(frame.get("rendering_frame", -1)), float(frame.get("rendering_time", -1.0))


class KitchenEpisodeCollector:
    """One explicit handshake per episode; never synthesizes a toggle button."""

    EPISODE_TIMEOUT_SECONDS = 180.0

    def __init__(self, backend, directory, *, control_port=5564, hz=30.0, intrinsics=(620.8, 625.22, 320., 240.)):
        self.backend = backend
        self.directory = Path(directory)
        self.control_port = control_port
        self.hz = float(hz)
        if not 0 < self.hz <= 200:
            raise ValueError("episode frequency must be in (0, 200]")
        self.intrinsics = intrinsics
        self.client = None
        self.camera = None
        self.writer = None
        self.phase = "IDLE"
        self.status = None
        self.status_at = 0.0
        self.next_request = 0.0
        self.next_sample = 0.0
        self.recording_started_at = None
        self.manual_reset_pending = False
        self.token = None
        self.finished_epoch = -1
        self.session = None
        self.outcome = "success"
        self.detector = PlacementDetector()
        self.saved_paths = []
        self.resume_wall_time = 0.0
        self.contact_view = None
        self.last_image_frame = -1
        self.started_at = 0.0
        self.last_link_warning = 0.0

    def prepare_scene(self):
        """Called before the first World.reset so PhysX cooks contact sensors."""
        import omni.usd
        from pxr import Usd, UsdPhysics
        from isaacsim.core.prims import RigidPrim

        stage = omni.usd.get_context().get_stage()
        for path in ("/World/banana", "/World/plate"):
            if not stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(f"episode collection requires a rigid body at {path}")
        hand_paths = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(self.backend.prim_path))
                      if p.HasAPI(UsdPhysics.RigidBodyAPI)
                      and any(word in p.GetName().lower() for word in ("hand", "wrist"))]
        if not hand_paths:
            raise ValueError("cannot verify release: robot hand rigid bodies were not found")
        self.contact_view = self.backend.world.scene.add(RigidPrim(
            prim_paths_expr="/World/banana", name="episode_banana_contacts",
            contact_filter_prim_paths_expr=["/World/plate", *hand_paths],
            track_contact_forces=True, max_contact_count=256,
        ))
        self.hand_paths = hand_paths

    def start(self):
        self.camera = EpisodeEgoCamera(self.backend, self.intrinsics)
        self.client = EpisodeControlClient(self.control_port)
        self.started_at = time.monotonic()
        self.bodies = {body.prim_path: body for body in self.backend.scene_reset_bodies}
        print(f"[Episodes] waiting for manual A+X; HDF5 directory={self.directory}", flush=True)

    @property
    def frozen(self):
        stale = self.phase == "RECORDING" and time.monotonic() - self.status_at > 1.0
        return self.phase in ("STOPPING", "RELEASING", "WAITING", "RESTARTING", "ERROR") or stale or self.manual_reset_pending

    def before_step(self):
        # Keyboard callbacks can run during world.render/step. Perform the
        # actual reset at the next loop boundary, before sampling or physics.
        if self.manual_reset_pending and self.phase not in ("STOPPING", "RELEASING", "ERROR"):
            try:
                resume = self.phase in ("RECORDING", "RESTARTING")
                if self.writer is not None:
                    self.writer.discard()
                    self.writer = None
                self.backend.mujoco_backspace_reset()
                self.manual_reset_pending = False
                self.recording_started_at = None
                self.detector.since = None
                if resume:
                    self.phase = "RESTARTING"
                print("[Episodes] manual Backspace: attempt discarded; timer restarted", flush=True)
            except Exception as exc:
                self._error(f"manual reset failed: {exc!r}")
        if self.client is None:
            return
        now = time.monotonic()
        if now - max(self.status_at, self.started_at) > 2.0 and now - self.last_link_warning > 5.0:
            print(f"[Episodes] waiting for terminal 3 at 127.0.0.1:{self.control_port}; "
                  "restart the PICO manager with the updated code. No automatic reset without acknowledgement.", flush=True)
            self.last_link_warning = now
        result = self.client.poll()
        if result is not None:
            if not result.get("ok"):
                self._error(f"manager rejected {result.get('operation')}: {result.get('error')}")
            elif self.session is not None and result["session"] != self.session and self.phase != "IDLE":
                self._error("manager restarted during an episode; no automatic reset")
            else:
                self.status, self.status_at = result, now
                self.session = result["session"]
                mode = result["mode"]
                if self.phase == "STOPPING" and mode == "PLANNER" and result["hold_token"] == self.token and result["hold_ticks"] >= 2:
                    # The manager has stopped POSE output and published idle planner
                    # commands. Physics remains paused throughout save/reset/release.
                    try:
                        if self.manual_reset_pending:
                            self.writer.discard()
                            self.manual_reset_pending = False
                        else:
                            path = self.writer.finish(self.outcome)
                            self.saved_paths.append(path)
                            print(f"[Episodes] saved {path}", flush=True)
                        self.writer = None
                        self.backend.mujoco_backspace_reset()
                        self.finished_epoch = result["pose_epoch"]
                        self.phase = "RELEASING"
                    except Exception as exc:
                        self._error(f"save/reset failed; tracking remains held: {exc!r}")
                elif self.phase == "RELEASING" and result["operation"] == "release" and result.get("released_token") == self.token and result["hold_token"] is None:
                    self.phase = "WAITING"
                    print("[Episodes] reset complete; press A+X manually for the next episode", flush=True)
                    if mode == "POSE" and result["pose_epoch"] > self.finished_epoch:
                        self._try_begin()
                elif self.phase in ("IDLE", "WAITING") and mode == "POSE" and result["pose_epoch"] > self.finished_epoch:
                    self._try_begin()
                elif self.phase in ("RECORDING", "RESTARTING") and mode == "OFF":
                    if self.writer is not None:
                        self.saved_paths.append(self.writer.finish("policy_off"))
                        self.writer = None
                    self.phase = "IDLE"
        if self.phase == "RESTARTING":
            self._try_begin()
        if (self.phase == "RECORDING" and self.recording_started_at is not None
                and now - self.recording_started_at >= self.EPISODE_TIMEOUT_SECONDS):
            self.outcome = "timeout"
            self.phase = "STOPPING"
            self.next_request = 0.0
            print("[Episodes] 180s episode timeout; requesting PLANNER hold", flush=True)
        if now >= self.next_request and self.client.pending is None and self.phase != "ERROR":
            operation = "hold" if self.phase == "STOPPING" else "release" if self.phase == "RELEASING" else "status"
            self.client.request(operation, self.token)
            self.next_request = now + 0.05

    def _try_begin(self):
        try:
            self._begin()
        except Exception as exc:
            self._error(f"cannot begin recording: {exc!r}")

    def _begin(self):
        _, valid, _, _ = self.camera.read()
        if not valid:
            return  # Keep waiting for a real first-person frame, not black fallback data.
        self.token = uuid.uuid4().hex
        scene = Path(self.backend.scene_layer_path)
        self.writer = Hdf5Episode(self.directory, {
            "robot_model": self.backend.robot_model, "joint_names": self.backend.dof_names,
            "body_command_joint_names": list(self.backend.robot_asset.body_joint_names),
            "quaternion_order": "wxyz", "world_units": "metres", "frequency_hz": self.hz,
            "physics_dt": self.backend.physics_dt, "scene_path": str(scene),
            "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
            "scene_usda": scene.read_text(), "camera": self.camera.metadata,
            "action_alignment": "last command applied before the observed physics step",
            "manager_session": self.session, "pose_epoch": self.status["pose_epoch"],
            "control_mode": getattr(self.backend.dds_bridge, "control_mode", "position"),
            "success_dwell_seconds": self.detector.dwell,
            "episode_timeout_seconds": self.EPISODE_TIMEOUT_SECONDS,
        })
        bridge = self.backend.dds_bridge
        self.resume_wall_time = bridge.discard_episode_commands() if bridge is not None else time.time()
        self.next_sample = self.backend._sim_time
        self.detector.since = None
        self.last_image_frame = -1
        self.outcome = "success"
        self.recording_started_at = time.monotonic()
        self.phase = "RECORDING"
        print(f"[Episodes] recording {self.writer.path.name}", flush=True)

    def after_step(self, state, bridge):
        if self.phase != "RECORDING" or self.frozen or self.backend._sim_time + 1e-9 < self.next_sample:
            return
        self.next_sample = self.backend._sim_time + 1.0 / self.hz
        try:
            row, detection = self._sample(state, bridge)
            self.writer.append(row)
            if not row["observations/images/ego_valid"]:
                self.detector.since = None
                return
            if self.detector.update(self.backend._sim_time, **detection):
                self.outcome = "success"
                self.phase = "STOPPING"
                self.next_request = 0.0
                print("[Episodes] object released and settled in plate; requesting PLANNER hold", flush=True)
        except Exception as exc:
            self._error(f"recording failed; no reset: {exc!r}")

    def _sample(self, state, bridge):
        import omni.usd
        from pxr import Usd, UsdGeom

        banana, plate = self.bodies["/World/banana"], self.bodies["/World/plate"]
        bp, bq = banana.get_world_pose()
        pp, pq = plate.get_world_pose()
        bv, pv = banana.get_linear_velocity(), plate.get_linear_velocity()
        ba = banana.get_angular_velocity()
        forces = np.asarray(self.contact_view.get_contact_force_matrix(dt=self.backend.physics_dt))[0]
        plate_force = float(np.linalg.norm(forces[0]))
        hand_force = float(np.linalg.norm(forces[1:], axis=-1).sum())
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"], False, True).ComputeWorldBound(
            omni.usd.get_context().get_stage().GetPrimAtPath("/World/banana")
        ).ComputeAlignedRange()
        lo, hi = np.asarray(bounds.GetMin()), np.asarray(bounds.GetMax())
        farthest_xy = np.maximum(np.abs(lo[:2] - pp[:2]), np.abs(hi[:2] - pp[:2]))
        # Source plate's local Y is its normal (the authored pose rotates it up).
        w, x, y, z = pq
        upright = 2.0 * (y * z + w * x) > 0.9
        inside = float(np.linalg.norm(farthest_xy)) < 0.105 and 0.0 < bp[2] - pp[2] < 0.09
        jpeg, valid, image_frame, image_time = self.camera.read()
        fresh_image = valid and image_frame != self.last_image_frame
        self.last_image_frame = image_frame
        command = getattr(bridge, "last_applied_command", None)
        targets = getattr(bridge, "last_position_targets_by_isaac", None)
        effort = getattr(bridge, "last_applied_tau_by_isaac", None)
        command_valid = command is not None and command.received_time >= self.resume_wall_time
        row = {
            "time/simulation": np.float64(self.backend._sim_time), "time/wall": np.float64(time.time()),
            "observations/joint_position": np.asarray(state.joint_position, dtype=np.float32),
            "observations/joint_velocity": np.asarray(state.joint_velocity, dtype=np.float32),
            "observations/root_pose": np.asarray([*state.root_position, *state.root_quaternion], dtype=np.float32),
            "observations/root_velocity": np.asarray([*state.root_linear_velocity, *state.root_angular_velocity], dtype=np.float32),
            "objects/banana_pose": np.asarray([*bp, *bq], dtype=np.float32),
            "objects/plate_pose": np.asarray([*pp, *pq], dtype=np.float32),
            "objects/banana_velocity": np.asarray([*bv, *ba], dtype=np.float32),
            "objects/plate_velocity": np.asarray([*pv, *plate.get_angular_velocity()], dtype=np.float32),
            "actions/valid": np.bool_(command_valid),
            "actions/joint_position_target": np.asarray(targets if targets is not None else np.full(self.backend.num_dof, np.nan), dtype=np.float32),
            "actions/applied_effort": np.asarray(effort if effort is not None else np.full(self.backend.num_dof, np.nan), dtype=np.float32),
            "actions/received_wall_time": np.float64(command.received_time if command is not None else np.nan),
            "observations/images/ego_jpeg": jpeg,
            "observations/images/ego_valid": np.bool_(fresh_image),
            "observations/images/ego_render_frame": np.int64(image_frame),
            "observations/images/ego_render_time": np.float64(image_time),
            "task/plate_contact_force": np.float32(plate_force), "task/hand_contact_force": np.float32(hand_force),
            "manager/mode": np.int32({"OFF": 0, "POSE": 1, "PLANNER": 2, "PLANNER_FROZEN_UPPER_BODY": 3, "POSE_PAUSE": 4, "PLANNER_VR_3PT": 5}[self.status["mode"]]),
        }
        for field in ("q", "dq", "kp", "kd", "tau"):
            row[f"actions/lowcmd_{field}"] = np.asarray(getattr(command, field) if command is not None else np.full(29, np.nan), dtype=np.float32)
        detection = dict(inside=inside, upright=upright, plate_contact=plate_force, hand_contact=hand_force,
                         fruit_speed=float(np.linalg.norm(bv)), plate_speed=float(np.linalg.norm(pv)),
                         fruit_angular_speed=float(np.linalg.norm(ba)))
        return row, detection

    def request_manual_reset(self):
        if self.phase != "ERROR":
            self.manual_reset_pending = True
        return True

    def _error(self, message):
        self.phase = "ERROR"
        print(f"[Episodes] PAUSED: {message}. Partial HDF5 retained; restart after resolving the error.", flush=True)

    def close(self):
        if self.writer is not None:
            self.writer.interrupt()
        if self.client is not None:
            self.client.close()
