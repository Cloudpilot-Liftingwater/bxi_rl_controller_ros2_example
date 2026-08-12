from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from bxi_example_py_elf3.sonic_pico.elf3_fk_calibration import (
    ELF3_ANCHOR_FRAME,
    ELF3_CONTROLLED_JOINTS,
    Elf3FkCalibration,
)
from bxi_example_py_elf3.sonic_pico.elf3_wrist_mapping import (
    ELF3_NATIVE_WRIST_INDICES,
    compute_elf3_wrist_features,
    pack_sonic_wrist_transport,
)
from bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge import (
    _extract_wrist_frames,
)
from bxi_example_py_elf3.inference.sonic import JOINT_NAMES


def _parse_xyz(value: str | None) -> np.ndarray:
    if value is None:
        return np.zeros(3, dtype=np.float64)
    return np.fromstring(value, sep=" ", dtype=np.float64)


def _homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _urdf_tree_oracle(urdf: Path, values: dict[str, float]) -> dict[str, np.ndarray]:
    """Independent serial-tree FK using only URDF XML and SciPy rotations."""
    root = ET.parse(urdf).getroot()
    children: dict[str, list[tuple[str, str, np.ndarray, np.ndarray, np.ndarray]]] = {}
    child_links = set()
    all_links = {link.attrib["name"] for link in root.findall("link")}

    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        xyz = _parse_xyz(origin.attrib.get("xyz") if origin is not None else None)
        rpy = _parse_xyz(origin.attrib.get("rpy") if origin is not None else None)
        axis_node = joint.find("axis")
        axis = _parse_xyz(axis_node.attrib.get("xyz") if axis_node is not None else None)
        children.setdefault(parent, []).append(
            (joint.attrib["name"], child, xyz, rpy, axis)
        )
        child_links.add(child)

    base_links = all_links - child_links
    assert len(base_links) == 1
    base = next(iter(base_links))
    poses = {base: np.eye(4, dtype=np.float64)}
    stack = [base]
    while stack:
        parent = stack.pop()
        for joint_name, child, xyz, rpy, axis in children.get(parent, ()):
            origin_tf = _homogeneous(
                # URDF rpy is fixed-axis roll/pitch/yaw (extrinsic xyz).
                Rotation.from_euler("xyz", rpy).as_matrix(), xyz
            )
            angle = values.get(joint_name, 0.0)
            motion_tf = _homogeneous(
                Rotation.from_rotvec(axis * angle).as_matrix(), np.zeros(3)
            )
            poses[child] = poses[parent] @ origin_tf @ motion_tf
            stack.append(child)
    return poses


def _anchor_local(transform: np.ndarray, anchor: np.ndarray) -> np.ndarray:
    return np.linalg.inv(anchor) @ transform


def _quat_multiply_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = np.moveaxis(left, -1, 0)
    rw, rx, ry, rz = np.moveaxis(right, -1, 0)
    return np.stack(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ),
        axis=-1,
    )


def _swing_quaternion_wxyz(rotvec: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """Independent swing = inverse(twist) * rotation quaternion oracle."""
    quat_xyzw = Rotation.from_rotvec(rotvec).as_quat()
    quat = quat_xyzw[:, [3, 0, 1, 2]]
    vector = quat[:, 1:]
    projected = np.sum(vector * axis, axis=1, keepdims=True) * axis
    twist = np.concatenate((quat[:, :1], projected), axis=1)
    twist /= np.linalg.norm(twist, axis=1, keepdims=True)
    twist_inverse = twist * np.array([1.0, -1.0, -1.0, -1.0])
    return _quat_multiply_wxyz(twist_inverse, quat)


def _independent_wrist_oracle(body_pose: np.ndarray) -> np.ndarray:
    """Independent NumPy/SciPy oracle for the validated six wrist features."""
    body_pose = np.asarray(body_pose).reshape(-1, 21, 3)
    smpl_l_elbow_aa = body_pose[:, 17]
    smpl_l_wrist_aa = body_pose[:, 19]
    smpl_r_elbow_aa = body_pose[:, 18]
    smpl_r_wrist_aa = body_pose[:, 20]

    axis = np.array([0, 1, 0])
    l_swing = _swing_quaternion_wxyz(smpl_l_elbow_aa, axis)
    r_swing = _swing_quaternion_wxyz(smpl_r_elbow_aa, axis)
    l_elbow = Rotation.from_quat(l_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    r_elbow = Rotation.from_quat(r_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    l_wrist = Rotation.from_rotvec(smpl_l_wrist_aa).as_euler("XYZ", degrees=False)
    r_wrist = Rotation.from_rotvec(smpl_r_wrist_aa).as_euler("XYZ", degrees=False)

    l_roll = l_elbow[:, 0] + l_wrist[:, 0]
    l_pitch = -l_wrist[:, 1]
    l_yaw = l_elbow[:, 2] + l_wrist[:, 2]
    r_roll = -(r_elbow[:, 0] + r_wrist[:, 0])
    r_pitch = -r_wrist[:, 1]
    r_yaw = r_elbow[:, 2] + r_wrist[:, 2]

    return np.column_stack((l_roll, -l_pitch, l_yaw, r_roll, r_pitch, r_yaw))


@pytest.mark.parametrize("seed", [0, 7, 28800])
def test_native_wrist_transport_matches_independent_oracle(seed):
    rng = np.random.default_rng(seed)
    # Avoid Euler singularities while covering both signs and coupled rotations.
    body_pose = rng.uniform(-1.2, 1.2, size=(256, 21, 3))

    expected = np.asarray(_independent_wrist_oracle(body_pose), dtype=np.float32)
    production = compute_elf3_wrist_features(body_pose)
    packed = np.stack([pack_sonic_wrist_transport(row) for row in production])
    transported = _extract_wrist_frames(packed)

    assert np.array_equal(np.asarray(production, dtype=np.float32), expected)
    assert np.array_equal(transported, expected)


def test_native_wrist_transport_indices_are_the_elf3_policy_order():
    assert ELF3_NATIVE_WRIST_INDICES == (19, 20, 21, 26, 27, 28)
    expected_names = (
        "l_wrist_x_joint",
        "l_wrist_y_joint",
        "l_wrist_z_joint",
        "r_wrist_x_joint",
        "r_wrist_y_joint",
        "r_wrist_z_joint",
    )
    assert tuple(JOINT_NAMES[index] for index in ELF3_NATIVE_WRIST_INDICES) == expected_names
    marker = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    packed = pack_sonic_wrist_transport(marker)
    assert np.array_equal(_extract_wrist_frames(packed)[0], marker.astype(np.float32))
    assert np.count_nonzero(packed) == 6


def test_elf3_fk_is_anchor_local_and_uses_only_the_kinematic_model():
    fk = Elf3FkCalibration.from_default_urdf()
    poses = fk.key_frame_poses(np.zeros(29, dtype=np.float64))

    assert fk.model.nq == 29
    assert set(fk.joint_to_q_index) == set(ELF3_CONTROLLED_JOINTS)
    assert fk.model.existFrame(ELF3_ANCHOR_FRAME)
    assert set(poses) == {"left_wrist", "right_wrist", "torso", "anchor"}
    for key in ("torso", "anchor"):
        assert np.array_equal(poses[key]["position"], np.zeros(3))
        assert np.array_equal(
            poses[key]["orientation_wxyz"], np.array([1.0, 0.0, 0.0, 0.0])
        )

    # ``pin.Model`` has no visual/collision geometry collections; those exist
    # only when RobotWrapper/geometry builders are used.
    assert not hasattr(fk, "visual_model")
    assert not hasattr(fk, "collision_model")


def test_elf3_urdf_revolute_order_matches_policy_and_fk_contract():
    fk = Elf3FkCalibration.from_default_urdf()
    root = ET.parse(fk.urdf_path).getroot()
    urdf_revolute = tuple(
        joint.attrib["name"]
        for joint in root.findall("joint")
        if joint.attrib.get("type") in {"revolute", "continuous"}
    )

    assert urdf_revolute == ELF3_CONTROLLED_JOINTS
    assert urdf_revolute == JOINT_NAMES
    assert tuple(
        urdf_revolute[index] for index in ELF3_NATIVE_WRIST_INDICES
    ) == (
        "l_wrist_x_joint",
        "l_wrist_y_joint",
        "l_wrist_z_joint",
        "r_wrist_x_joint",
        "r_wrist_y_joint",
        "r_wrist_z_joint",
    )


def test_elf3_control_node_and_mujoco_actuators_match_policy_order():
    repo = Path(__file__).resolve().parents[3]
    package = repo / "src/bxi_example_py_elf3"

    demo_module = (
        package / "bxi_example_py_elf3/bxi_example_demo.py"
    ).read_text(encoding="utf-8")
    assignment = demo_module.split("joint_name = (", 1)[1].split(")", 1)[0]
    demo_names = tuple(
        line.split('"', 2)[1]
        for line in assignment.splitlines()
        if '"' in line
    )

    mujoco_root = ET.parse(
        package / "data/mujoco_simulation/elf3.xml"
    ).getroot()
    actuator = mujoco_root.find("actuator")
    assert actuator is not None
    actuator_names = tuple(item.attrib["joint"] for item in actuator)

    assert demo_names == JOINT_NAMES
    assert actuator_names == JOINT_NAMES


@pytest.mark.parametrize("seed", [0, 19, 28800])
def test_elf3_fk_matches_independent_urdf_tree_oracle(seed):
    fk = Elf3FkCalibration.from_default_urdf()
    rng = np.random.default_rng(seed)

    for _ in range(24):
        q29 = rng.uniform(-0.7, 0.7, size=29)
        values = dict(zip(ELF3_CONTROLLED_JOINTS, q29))
        oracle_world = _urdf_tree_oracle(fk.urdf_path, values)
        oracle_anchor = oracle_world[ELF3_ANCHOR_FRAME]
        actual = fk.key_frame_poses(q29, apply_offset=False)

        for key, link in (
            ("left_wrist", "l_wrist_z_link"),
            ("right_wrist", "r_wrist_z_link"),
            ("torso", ELF3_ANCHOR_FRAME),
            ("anchor", ELF3_ANCHOR_FRAME),
        ):
            expected = _anchor_local(oracle_world[link], oracle_anchor)
            np.testing.assert_allclose(
                actual[key]["position"], expected[:3, 3], atol=1e-10, rtol=1e-10
            )
            np.testing.assert_allclose(
                Rotation.from_quat(actual[key]["orientation_xyzw"]).as_matrix(),
                expected[:3, :3],
                atol=1e-10,
                rtol=1e-10,
            )


def test_manager_and_canonical_sources_have_no_g1_runtime_reference():
    repo = Path(__file__).resolve().parents[3]
    paths = (
        repo / "gear_sonic/scripts/pico_manager_thread_server.py",
        repo / "script/run_sonic_pico_sources.sh",
    )
    for path in paths:
        text = path.read_text()
        assert "instantiate_g1" not in text
        assert "pico_g1_legacy" not in text
        assert "g1_debug" not in text
