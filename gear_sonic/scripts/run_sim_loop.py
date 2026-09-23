"""Run the Isaac G1 simulation, DDS control bridge, and camera publisher.

The deploy relay optionally forwards keyboard commands and XRoboToolkit poses.
"""

import sys
from pathlib import Path
from dataclasses import MISSING, fields
from datetime import datetime
from typing import Any, Dict

REPO_ROOT = Path(__file__).resolve().parents[2]
repo_root_str = str(REPO_ROOT)
if repo_root_str in sys.path:
    sys.path.remove(repo_root_str)
sys.path.insert(0, repo_root_str)

try:
    import tyro
except ModuleNotFoundError:
    tyro = None

import gear_sonic
from gear_sonic.utils.isaac_sim.configs import SimLoopConfig

ArgsConfig = SimLoopConfig


def _parse_cli_fallback() -> ArgsConfig:
    """Small argparse fallback for Isaac Python environments without tyro."""

    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    config = ArgsConfig()
    for field_info in fields(ArgsConfig):
        name = field_info.name
        default = field_info.default if field_info.default is not MISSING else getattr(config, name)
        arg_name = "--" + name.replace("_", "-")
        if isinstance(default, bool):
            parser.add_argument(arg_name, dest=name, action="store_true", default=default)
            parser.add_argument("--no-" + name.replace("_", "-"), dest=name, action="store_false")
        elif isinstance(default, int) and not isinstance(default, bool):
            parser.add_argument(arg_name, type=int, default=default)
        elif isinstance(default, float):
            parser.add_argument(arg_name, type=float, default=default)
        elif default is None:
            parser.add_argument(arg_name, default=default)
        elif isinstance(default, str):
            parser.add_argument(arg_name, type=str, default=default)

    namespace = parser.parse_args()
    for key, value in vars(namespace).items():
        setattr(config, key, value)
    config.__post_init__()
    return config


def _sonic_to_isaac_positions(backend: Any, sonic_positions: Any) -> Any:
    import numpy as np

    current_isaac = np.asarray(backend.articulation.get_joint_positions(), dtype=np.float32)
    result = current_isaac.copy()
    for sonic_index, isaac_index in enumerate(backend.joint_mapping.sonic_body_to_isaac_dof):
        result[isaac_index] = float(sonic_positions[sonic_index])
    return result


def _resolve_isaac_experience_path(config: ArgsConfig) -> Path | None:
    if config.isaac_experience_path:
        return Path(config.isaac_experience_path).expanduser().resolve()
    if config.isaac_xr_mode is None:
        return None

    isaac_root = Path("/home/colin/Erwin/isaac-sim-4.5.0")
    experience_by_mode = {
        "vr": "isaacsim.exp.base.xr.vr.kit",
        "openxr": "isaacsim.exp.base.xr.openxr.kit",
    }
    try:
        experience_name = experience_by_mode[str(config.isaac_xr_mode)]
    except KeyError as exc:
        raise ValueError("--isaac-xr-mode must be one of: vr, openxr") from exc
    return (isaac_root / "apps" / experience_name).resolve()


def _set_startup_gui_perspective_zoom(
    backend: Any,
    *,
    zoom: float,
    right_offset: float = 0.0,
    up_offset: float = 0.0,
) -> None:
    """Dolly the default GUI Perspective toward G1 without changing scene cameras.

    This is deliberately limited to the GUI Perspective camera.  It does not
    move ``ego_view_camera`` or alter the active camera source on port 5555.
    """
    import numpy as np

    zoom = float(zoom)
    if zoom <= 0.0:
        raise ValueError("--isaac-gui-perspective-zoom must be positive")
    right_offset = float(right_offset)
    up_offset = float(up_offset)
    if (
        np.isclose(zoom, 1.0)
        and np.isclose(right_offset, 0.0)
        and np.isclose(up_offset, 0.0)
    ) or bool(getattr(backend, "headless", True)):
        return

    from omni.kit.viewport.utility import get_active_viewport
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import UsdGeom
    import omni.usd

    viewport = get_active_viewport()
    if viewport is None or str(getattr(viewport, "camera_path", "")) != "/OmniverseKit_Persp":
        print("[IsaacBackend] GUI Perspective zoom skipped: active viewport is not /OmniverseKit_Persp")
        return

    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath("/OmniverseKit_Persp")
    torso_prim = stage.GetPrimAtPath(f"{backend.prim_path.rstrip('/')}/torso_link")
    if not camera_prim or not camera_prim.IsValid() or not torso_prim or not torso_prim.IsValid():
        print("[IsaacBackend] GUI Perspective zoom skipped: camera or G1 torso prim is unavailable")
        return

    xform_cache = UsdGeom.XformCache()
    eye = np.asarray(xform_cache.GetLocalToWorldTransform(camera_prim).ExtractTranslation(), dtype=np.float64)
    target = np.asarray(xform_cache.GetLocalToWorldTransform(torso_prim).ExtractTranslation(), dtype=np.float64)
    offset = eye - target
    distance = float(np.linalg.norm(offset))
    if distance <= 1e-6:
        print("[IsaacBackend] GUI Perspective zoom skipped: camera is already at G1 torso")
        return

    view_direction = (target - eye) / distance
    screen_right = np.cross(view_direction, np.asarray([0.0, 0.0, 1.0]))
    screen_right_norm = float(np.linalg.norm(screen_right))
    if screen_right_norm <= 1e-6:
        print("[IsaacBackend] GUI Perspective offset skipped: view is vertical")
        return
    screen_right /= screen_right_norm
    new_eye = (
        target
        + offset / zoom
        + screen_right * right_offset
        + np.asarray([0.0, 0.0, up_offset])
    )
    set_camera_view(eye=new_eye, target=target, camera_prim_path="/OmniverseKit_Persp")
    print(
        "[IsaacBackend] GUI Perspective startup zoom applied "
        f"factor={zoom:g} target=G1/torso_link distance={distance:.3f}->{distance / zoom:.3f} m "
        f"right_offset={right_offset:.3f}m up_offset={up_offset:.3f}m"
    )


def _run_isaac_loop(config: ArgsConfig, wbc_config: Dict[str, Any]) -> None:
    """Run the simulator terminal in the XRoboToolkit/Sonic stack.

    Port ownership:

    - 5555: simulator publishes camera msgpack frames.
    - 5557: normally owned by ``g1_deploy_onnx_ref``; Isaac only provides an
      optional fallback publisher when explicitly requested.
    """

    import os
    import time
    import numpy as np

    from gear_sonic.robot_interface.isaac_robot_assets import DEFAULT_ROBOT_MODEL, get_robot_asset
    from gear_sonic.robot_interface.pd_mapping import SONIC_STANDING_POSE
    from gear_sonic.simulation_server.isaac_server import (
        DEFAULT_SONICSTAR_TASK_SCENE_LAYER_PATH,
        IsaacSimulationBackend,
    )

    robot_model_name = config.isaac_robot_model or DEFAULT_ROBOT_MODEL
    robot_asset = get_robot_asset(robot_model_name)
    log_dir = (
        REPO_ROOT
        / "work_dirs"
        / "isaac_sim_loop"
        / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    log_dir.mkdir(parents=True, exist_ok=True)

    experience_path = _resolve_isaac_experience_path(config)
    headless = False if experience_path is not None else not bool(config.enable_onscreen)
    physics_dt = 1.0 / float(config.sim_frequency)
    backend = IsaacSimulationBackend(
        robot_model=robot_model_name,
        prim_path=robot_asset.prim_path,
        articulation_root_path=robot_asset.articulation_root,
        headless=headless,
        physics_dt=physics_dt,
        rendering_dt=1.0 / 60.0,
        startup_steps=20,
        scene_layer_path=config.isaac_scene_layer_path or DEFAULT_SONICSTAR_TASK_SCENE_LAYER_PATH,
        collision_layer_path=robot_asset.collision_layer_path,
        experience_path=experience_path,
        forward_k_to_deploy=bool(config.isaac_forward_k_to_deploy),
    )
    backend.elastic_band.point = np.asarray(
        [config.isaac_initial_root_x, config.isaac_initial_root_y, config.isaac_elastic_band_anchor_z],
        dtype=np.float32,
    )
    initial_yaw = np.deg2rad(float(config.isaac_initial_root_yaw))
    initial_orientation = np.asarray(
        [np.cos(initial_yaw / 2.0), 0.0, 0.0, np.sin(initial_yaw / 2.0)], dtype=np.float32
    )
    backend.elastic_band.target_yaw = initial_yaw
    backend.elastic_band.enabled = bool(config.isaac_elastic_band_enabled)
    camera_image_dt = 1.0 / float(config.isaac_camera_fps)
    ego_camera_intrinsics = (
        config.isaac_ego_camera_fx,
        config.isaac_ego_camera_fy,
        config.isaac_ego_camera_cx,
        config.isaac_ego_camera_cy,
    )
    bridge = None

    if config.isaac_episode_directory:
        from gear_sonic.simulation_server.kitchen_episodes import KitchenEpisodeCollector

        if not config.isaac_forward_k_to_deploy or config.isaac_camera_only:
            raise ValueError("episode collection requires the deploy relay and full control loop")
        backend.episode_collector = KitchenEpisodeCollector(
            backend, config.isaac_episode_directory,
            control_port=config.isaac_episode_control_port, hz=config.isaac_episode_hz,
            intrinsics=ego_camera_intrinsics,
        )

    try:
        backend.start()
        if config.isaac_camera_source == "gui_perspective":
            _set_startup_gui_perspective_zoom(
                backend,
                zoom=float(config.isaac_gui_perspective_zoom),
                right_offset=float(config.isaac_gui_perspective_right_offset),
                up_offset=float(config.isaac_gui_perspective_up_offset),
            )
        if config.isaac_camera_only:
            if config.isaac_publish_camera:
                backend.start_image_publish_subprocess(
                    camera_port=config.camera_port,
                    camera_source=config.isaac_camera_source,
                    image_dt=camera_image_dt,
                    ego_camera_intrinsics=ego_camera_intrinsics,
                    local_translation=(
                        config.isaac_camera_local_x,
                        config.isaac_camera_local_y,
                        config.isaac_camera_local_z,
                    ),
                )

            print(
                "[run_sim_loop] Isaac camera-only backend ready: "
                f"camera_port={config.camera_port}, "
                f"camera_fps={float(config.isaac_camera_fps):g}, "
                f"camera_only={config.isaac_camera_only}, "
                f"log_dir={log_dir}",
                flush=True,
            )
            while True:
                backend.step(render=True)
                if backend.image_publish_process is not None:
                    backend.image_publish_process.maybe_publish(backend)
                time.sleep(max(0.0, 1.0 / float(config.sim_frequency)))
            return

        from gear_sonic.robot_interface.isaac_unitree_bridge import IsaacUnitreeBridge

        bridge = IsaacUnitreeBridge(
            domain_id=int(wbc_config.get("DOMAIN_ID", 0)),
            network_interface=wbc_config.get("INTERFACE", None),
            log_dir=log_dir,
            verbose=bool(config.verbose),
            control_mode=os.environ.get("ISAAC_CONTROL_MODE", "position"),
            robot_asset=robot_asset,
        )
        bridge.articulation = backend.articulation
        bridge.joint_mapping = backend.joint_mapping
        bridge.robot_asset = backend.robot_asset

        standing_pose_sonic = np.asarray(SONIC_STANDING_POSE, dtype=np.float32)
        standing_pose_isaac = _sonic_to_isaac_positions(backend, standing_pose_sonic)
        init_report = backend.initialize_articulation_state_for_control(
            joint_positions=standing_pose_isaac,
            root_position=np.asarray(
                [config.isaac_initial_root_x, config.isaac_initial_root_y, config.isaac_initial_root_height],
                dtype=np.float32,
            ),
            root_orientation=initial_orientation,
            configure_drives=lambda: bridge.configure_joint_drives(force=True),
            target_writer=bridge._apply_isaac_position_targets,
            reset_world=True,
            startup_warmup_steps=backend.startup_steps,
            state_commit_steps=20,
            target_commit_steps=20,
            render=not headless,
        )
        backend.set_elastic_band_enabled(bool(config.isaac_elastic_band_enabled))
        (log_dir / "canonical_control_init.log").write_text(
            f"{init_report}\n"
            f"elastic_band={backend.elastic_band.as_dict()}\n"
            f"isaac_initial_root_xy={[config.isaac_initial_root_x, config.isaac_initial_root_y]}\n"
            f"isaac_initial_root_yaw={float(config.isaac_initial_root_yaw)}\n"
            f"isaac_initial_root_height={float(config.isaac_initial_root_height)}\n",
            encoding="utf-8",
        )

        if config.isaac_publish_camera:
            backend.start_image_publish_subprocess(
                camera_port=config.camera_port,
                camera_source=config.isaac_camera_source,
                image_dt=camera_image_dt,
                ego_camera_intrinsics=ego_camera_intrinsics,
                local_translation=(
                    config.isaac_camera_local_x,
                    config.isaac_camera_local_y,
                    config.isaac_camera_local_z,
                ),
            )
        if config.isaac_publish_debug_state:
            backend.start_realtime_state_publisher(
                port=config.state_zmq_port,
                topic=config.state_zmq_topic,
            )
            print(
                "[run_sim_loop] warning: Isaac direct 5557 publisher is enabled. "
                "Do not run g1_deploy_onnx_ref on the same 5557 port."
            )

        print(
            "[run_sim_loop] Isaac backend ready: "
            f"DDS lowstate/lowcmd active after bridge.start(), camera_port={config.camera_port}, "
            f"camera_fps={float(config.isaac_camera_fps):g}, "
            f"debug_5557={'direct-isaac' if config.isaac_publish_debug_state else 'owned-by-g1_deploy_onnx_ref'}, "
            f"elastic_band_enabled={backend.elastic_band.enabled}, "
            f"elastic_band_anchor={backend.elastic_band.point.tolist()}, "
            f"initial_root_height={float(config.isaac_initial_root_height)}, "
            f"log_dir={log_dir}",
            flush=True,
        )
        if backend.episode_collector is not None:
            backend.episode_collector.start()
        backend.spin(
            frequency_hz=int(config.sim_frequency),
            dds_bridge=bridge,
            log_dir=log_dir,
        )
    except KeyboardInterrupt:
        print("+++++Isaac simulator interrupted by user.")
    except Exception:
        import traceback

        print("[run_sim_loop] Isaac backend fatal error:", flush=True)
        traceback.print_exc()
        raise
    finally:
        if bridge is not None:
            bridge.stop()
        backend.close()


def main(config: ArgsConfig):
    if config.isaac_camera_follow_hmd:
        raise ValueError(
            "--isaac-camera-follow-hmd has been removed; "
            "use the GUI Perspective or robot-mounted ego camera without this flag."
        )
    wbc_config = config.load_wbc_yaml()
    print(
        "[run_sim_loop] "
        f"script={Path(__file__).resolve()} "
        f"repo_root={REPO_ROOT} "
        f"gear_sonic={Path(gear_sonic.__file__).resolve()} "
        f"backend={config.simulator} "
        f"robot_scene={wbc_config.get('ROBOT_SCENE')}",
        flush=True,
    )
    wbc_config["ENV_NAME"] = config.env_name
    _run_isaac_loop(config=config, wbc_config=wbc_config)


if __name__ == "__main__":
    if tyro is not None:
        config = tyro.cli(ArgsConfig)
    else:
        print("[run_sim_loop] warning: tyro is not installed; using limited argparse fallback")
        config = _parse_cli_fallback()
    main(config)
