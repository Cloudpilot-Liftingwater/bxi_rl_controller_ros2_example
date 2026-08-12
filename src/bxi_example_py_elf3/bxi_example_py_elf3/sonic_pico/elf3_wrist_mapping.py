"""ELF3 wrist features derived from the PICO SMPL pose stream.

The released step28800 tokenizer expects six wrist values in this order::

    [left_x, left_y, left_z, right_x, right_y, right_z]

The implementation intentionally preserves the arithmetic order used by the
validated SONIC runtime.  It has no dependency on another robot's URDF, FK,
joint limits, or mesh assets.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from gear_sonic.trl.utils.rotation_conversion import decompose_rotation_aa


ELF3_WRIST_LAYOUT_VERSION = 1
ELF3_WRIST_WIDTH = 6

# ELF3's authoritative positions in the 29-wide SONIC policy joint order.
ELF3_NATIVE_WRIST_INDICES: tuple[int, ...] = (19, 20, 21, 26, 27, 28)


def compute_elf3_wrist_features(smpl_body_pose: np.ndarray) -> np.ndarray:
    """Return ELF3 wrist x/y/z features for one or more SMPL body poses.

    Args:
        smpl_body_pose: Axis-angle SMPL body pose with shape ``(N, 21, 3)`` or
            ``(21, 3)``.  Indices 17/19 and 18/20 are the left/right elbow and
            wrist rotations used by the validated SONIC mapping.

    Returns:
        A float64 array with shape ``(N, 6)`` in
        ``left_x,left_y,left_z,right_x,right_y,right_z`` order.
    """
    body_pose = np.asarray(smpl_body_pose)
    if body_pose.shape == (21, 3):
        body_pose = body_pose[None, ...]
    if body_pose.ndim != 3 or body_pose.shape[1:] != (21, 3):
        raise ValueError(
            f"SMPL body pose must have shape (N,21,3) or (21,3), got {body_pose.shape}"
        )
    if not np.all(np.isfinite(body_pose)):
        raise ValueError("SMPL body pose contains non-finite values")

    smpl_l_elbow_aa = body_pose[:, 17]
    smpl_l_wrist_aa = body_pose[:, 19]
    smpl_r_elbow_aa = body_pose[:, 18]
    smpl_r_wrist_aa = body_pose[:, 20]

    elbow_axis = np.array([0, 1, 0])
    _l_twist, l_swing = decompose_rotation_aa(smpl_l_elbow_aa, elbow_axis)
    _r_twist, r_swing = decompose_rotation_aa(smpl_r_elbow_aa, elbow_axis)

    l_elbow_swing_euler = Rotation.from_quat(l_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    r_elbow_swing_euler = Rotation.from_quat(r_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    l_wrist_euler = Rotation.from_rotvec(smpl_l_wrist_aa).as_euler(
        "XYZ", degrees=False
    )
    r_wrist_euler = Rotation.from_rotvec(smpl_r_wrist_aa).as_euler(
        "XYZ", degrees=False
    )

    left_x = l_elbow_swing_euler[:, 0] + l_wrist_euler[:, 0]
    # Preserve the validated mapping's two sign operations exactly.
    left_y_intermediate = -l_wrist_euler[:, 1]
    left_y = -left_y_intermediate
    left_z = l_elbow_swing_euler[:, 2] + l_wrist_euler[:, 2]

    right_x = -(r_elbow_swing_euler[:, 0] + r_wrist_euler[:, 0])
    right_y = -r_wrist_euler[:, 1]
    right_z = r_elbow_swing_euler[:, 2] + r_wrist_euler[:, 2]

    return np.column_stack((left_x, left_y, left_z, right_x, right_y, right_z))


def pack_sonic_wrist_transport(wrist: np.ndarray, width: int = 29) -> np.ndarray:
    """Pack one native six-value wrist vector into the current manager field."""
    wrist = np.asarray(wrist)
    if wrist.shape != (ELF3_WRIST_WIDTH,):
        raise ValueError(f"ELF3 wrist must have shape (6,), got {wrist.shape}")
    if width <= max(ELF3_NATIVE_WRIST_INDICES):
        raise ValueError(f"transport width {width} cannot hold ELF3 wrist fields")

    joint_pos = np.zeros(width)
    for index, value in zip(ELF3_NATIVE_WRIST_INDICES, wrist):
        joint_pos[index] = value
    return joint_pos
