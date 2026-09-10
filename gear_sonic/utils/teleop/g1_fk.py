"""G1 key-frame forward kinematics used by Pico pose calibration."""

from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation as sRot


G1_LEFT_WRIST_FRAME = "left_wrist_yaw_link"
G1_RIGHT_WRIST_FRAME = "right_wrist_yaw_link"
G1_TORSO_FRAME = "torso_link"

# Local-frame offsets used by the Sonic VR tracking targets.
G1_KEY_FRAME_OFFSETS = {
    "left_wrist": np.array([0.18, -0.025, 0.0]),
    "right_wrist": np.array([0.18, 0.025, 0.0]),
    "torso": np.array([0.0, 0.0, 0.35]),
}

G1_FRAME_MAPPING = {
    "left_wrist": G1_LEFT_WRIST_FRAME,
    "right_wrist": G1_RIGHT_WRIST_FRAME,
    "torso": G1_TORSO_FRAME,
}


def get_g1_key_frame_poses(
    robot_model,
    q: np.ndarray = None,
    root_position: np.ndarray = None,
    apply_offset: bool = True,
) -> Dict[str, Dict[str, np.ndarray]]:
    """Get G1 wrist and torso poses using the supplied Pinocchio robot model.

    Positions include the local frame offsets by default. Both xyzw and wxyz
    quaternion formats are returned for the calibration callers.
    """
    if q is None:
        q = robot_model.default_body_pose
    if root_position is None:
        root_position = np.array([0.0, 0.0, 0.0])

    # Update forward kinematics
    robot_model.cache_forward_kinematics(q, auto_clip=False)

    result = {}
    for key, frame_name in G1_FRAME_MAPPING.items():
        # A missing frame indicates an incompatible robot asset.
        try:
            placement = robot_model.frame_placement(frame_name)
        except ValueError as e:
            raise RuntimeError(
                f"Cannot find frame '{frame_name}' (key='{key}') in robot model. "
                f"Ensure the URDF contains this frame. Original error: {e}"
            ) from e

        rotation_matrix = placement.rotation

        # Apply offset in local frame, then transform to world frame
        if apply_offset and key in G1_KEY_FRAME_OFFSETS:
            local_offset = G1_KEY_FRAME_OFFSETS[key]
            world_offset = rotation_matrix @ local_offset
            position = placement.translation + world_offset + root_position
        else:
            position = placement.translation + root_position

        # Convert rotation matrix to quaternion using scipy
        rot = sRot.from_matrix(rotation_matrix)
        quat_xyzw = rot.as_quat()  # scipy returns [qx, qy, qz, qw]
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

        result[key] = {
            "position": position.copy(),
            "orientation_xyzw": quat_xyzw.copy(),
            "orientation_wxyz": quat_wxyz.copy(),
        }

    return result
