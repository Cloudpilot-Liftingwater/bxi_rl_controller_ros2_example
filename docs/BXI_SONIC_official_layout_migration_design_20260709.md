# BXI SONIC official-layout migration design

Date: 2026-07-09

Scope:

- Runtime being audited: `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/`
- Official baseline: `/data/downloads/bxi_rl_controller_ros2_example-main/`
- Goal: keep the official ELF3/BXI controller structure, add SONIC as a normal robot state/action module, and remove the temporary `gear_sonic_deploy/gear_sonic` sidecar dependency from the final BXI runtime.

This document is a design checklist only. No large files are moved here.

## 0. Organizing Principle

SONIC should be treated exactly like the existing official actions:

- policy code belongs under `src/bxi_example_py_elf3/bxi_example_py_elf3/`
- policy/runtime assets belong under `src/bxi_example_py_elf3/data/`
- launch files belong under `src/bxi_example_py_elf3/launch/`
- state machine config belongs under `src/bxi_example_py_elf3/config/`
- remote controller mapping belongs under `src/remote_controller/config/`
- operator/deployment helper scripts belong under `script/`

`gear_sonic_deploy/` and `gear_sonic/` should become transition/history material, not the final authority for the BXI runtime.

## 1. Current Facts

Official package behavior:

- `src/bxi_example_py_elf3/setup.py` already recursively installs:
  - `data/` to `install/share/bxi_example_py_elf3/data/`
  - `launch/` to `install/share/bxi_example_py_elf3/launch/`
  - `config/` to `install/share/bxi_example_py_elf3/config/`
- Therefore SONIC ONNX/reference/launch/config can be stored in the official package without adding a new install mechanism.

Current SONIC runtime facts:

- `src/bxi_example_py_elf3/bxi_example_py_elf3/inference/sonic.py` is already in the right Python package location.
- `sonic.py` and `bxi_example_demo.py` still default to:
  - `gear_sonic_deploy/policy/elf3_step28800_smpl/model_step_028800_smpl.onnx`
  - `gear_sonic_deploy/reference/elf3_step28800_idle_left_001_A019/stream_reference.npz`
- The current PICO input chain still imports from `gear_sonic.*`.
- The current FK calibration in PICO manager uses G1:
  - `instantiate_g1_robot_model()`
  - `get_g1_key_frame_poses()`
  - `left_wrist_yaw_link`
  - `right_wrist_yaw_link`
  - `torso_link`
  - `G1_KEY_FRAME_OFFSETS`
- The current bridge compensates for this with a legacy wrist index mapping:
  - `PICO_G1_LEGACY_WRIST_IDX = [23, 25, 27, 24, 26, 28]`
  - `ELF3_NATIVE_WRIST_IDX = [19, 20, 21, 26, 27, 28]`

ELF3-native training/deploy contract facts:

- Training config already uses ELF3 names:
  - joints: `l_wrist_x_joint`, `l_wrist_y_joint`, `l_wrist_z_joint`, `r_wrist_x_joint`, `r_wrist_y_joint`, `r_wrist_z_joint`
  - VR bodies: `l_wrist_z_link`, `r_wrist_z_link`, `torso_link`
  - anchor: `waist_z_link`
- `resources/elf3_dof29_hand/urdf/elf3.urdf` contains the needed ELF3 frames:
  - `torso_link`
  - `waist_z_link`
  - `l_wrist_z_link`
  - `r_wrist_z_link`
  - full 29 controlled joint names

## 2. Checklist A: G1 FK Calibration To ELF3 FK Calibration

### A1. Code To Replace

Current files with G1 FK semantics:

- `gear_sonic/scripts/pico_manager_thread_server.py`
  - imports `get_g1_key_frame_poses`
  - `ThreePointPose.__init__()` instantiates `instantiate_g1_robot_model()`
  - `_capture_calibration()` calls `get_g1_key_frame_poses()`
  - error messages and comments say G1
  - hard-coded wrist joint output uses G1 index names
- `gear_sonic/utils/teleop/vis/vr3pt_pose_visualizer.py`
  - defines `G1_LEFT_WRIST_FRAME = "left_wrist_yaw_link"`
  - defines `G1_RIGHT_WRIST_FRAME = "right_wrist_yaw_link"`
  - defines `G1_KEY_FRAME_OFFSETS`
  - implements `get_g1_key_frame_poses()`
- `gear_sonic/data/robot_model/instantiation/g1.py`
  - loads G1 URDF and G1 supplemental info
- `gear_sonic/data/robot_model/model_data/g1/...`
  - 51 MiB G1 model/mesh dependency

### A2. New ELF3 FK Helper

Create a small BXI-owned helper instead of moving the whole G1 robot-model stack:

```text
src/bxi_example_py_elf3/bxi_example_py_elf3/sonic_pico/
  __init__.py
  elf3_fk_calibration.py
```

Expected responsibilities:

- load ELF3 URDF with Pinocchio
- build `q` from the 29 ELF3 controlled joint order
- run forward kinematics
- return poses for:
  - `left_wrist`: frame `l_wrist_z_link`
  - `right_wrist`: frame `r_wrist_z_link`
  - `torso`: frame `torso_link`
  - optional `anchor`: frame `waist_z_link`
- apply ELF3-native offsets matching the training/deploy contract:
  - current native config uses wrist offsets `[[0.203, 0.0, 0.0], [0.203, 0.0, 0.0]]`
  - torso/VR point offset remains `[0.0, 0.0, 0.35]` unless A/B validation says otherwise

Recommended URDF source:

- primary: package an installable URDF under `src/bxi_example_py_elf3/data/sonic_robot_model/elf3_dof29_hand/urdf/elf3.urdf`
- source of truth to copy from: `resources/elf3_dof29_hand/urdf/elf3.urdf`
- start by testing whether Pinocchio can load URDF-only for FK. If meshes are required by the loader, copy the minimal `urdf/meshes/` directory as well. This is about 13 MiB, still much smaller and semantically correct compared with the 51 MiB G1 model directory.

Why not use `src/bxi_example_py_elf3/data/mujoco_simulation/elf3.xml` first:

- current FK helper path is Pinocchio/URDF-based
- using MJCF would require a separate MuJoCo FK implementation
- URDF migration is smaller and closer to the existing PICO manager code

### A3. PICO Manager Calibration Changes

Move/refactor the PICO manager into BXI package space:

```text
src/bxi_example_py_elf3/bxi_example_py_elf3/sonic_pico/pico_manager.py
```

Then replace:

- `with_g1_robot` -> `with_elf3_robot` or generic `with_robot`
- `instantiate_g1_robot_model()` -> `Elf3FkCalibration.from_package_data(...)`
- `get_g1_key_frame_poses()` -> `get_elf3_key_frame_poses()`
- `G1_*_WRIST_*_IDX` names -> `ELF3_*_WRIST_*_IDX`
- G1 comments/error text -> ELF3-neutral text

Important: do not only replace the URDF. Also replace the wrist joint convention.

Current wrist output path:

```text
PICO manager emits joint_pos using G1 legacy wrist slots
bridge extracts [23, 25, 27, 24, 26, 28]
bridge converts to ELF3 native wrist order
```

Target wrist output path:

```text
PICO manager emits joint_pos directly in ELF3 29-joint order
bridge uses --wrist-source elf3_native by default
legacy pico_g1_legacy remains only for comparison
```

ELF3 native wrist indices:

```text
l_wrist_x_joint -> 19
l_wrist_y_joint -> 20
l_wrist_z_joint -> 21
r_wrist_x_joint -> 26
r_wrist_y_joint -> 27
r_wrist_z_joint -> 28
```

### A4. Optional G1 Hand IK

Current PICO manager tries to load `G1GripperInverseKinematicsSolver`. If it is unavailable, it falls back to zero hand joints.

For the current ELF3 29DOF SONIC policy:

- hand/finger joints are not part of the action output
- G1 hand IK must not be a required runtime dependency
- keep this as optional or remove it from the default BXI PICO runtime
- if hand/finger data collection is needed later, add an ELF3-specific hand module under `sonic_pico/hand/`

### A5. A/B Validation Before Switching Default

Before making ELF3 FK the default:

1. Capture or reuse one PICO stream segment.
2. Generate `smpl_ref` with legacy G1 FK path.
3. Generate `smpl_ref` with new ELF3 FK path.
4. Compare:
   - wrist position time series
   - wrist quaternion continuity
   - torso/anchor orientation
   - frame latency and dropped frames
   - sim2sim teleop stability
5. Run sim2sim through the official BXI state machine:
   - pd_brake
   - normal
   - sonic_teleop
6. Only then change default from `pico_g1_legacy` to `elf3_native`.

### A6. Removal Criteria For G1 Model Directory

`gear_sonic/data/robot_model/model_data/g1/...` can stop being a BXI runtime dependency only after:

- `rg -n "instantiate_g1|get_g1|G1_|left_wrist_yaw_link|right_wrist_yaw_link" src/bxi_example_py_elf3 script src/remote_controller` has no active runtime hits
- PICO manager starts and publishes `smpl_ref` without importing `gear_sonic.data.robot_model.instantiation.g1`
- sim2sim passes with ELF3 FK
- model/reference defaults resolve from package share, not `gear_sonic_deploy`

## 3. Checklist B: Move SONIC Sidecar Into Official BXI Structure

### B1. Keep In Official BXI Package

Already correctly located:

```text
src/bxi_example_py_elf3/bxi_example_py_elf3/inference/sonic.py
```

Need to update:

- default model/ref paths should use package data:
  - `data/sonic_model/elf3_step28800_smpl/model_step_028800_smpl.onnx`
  - `data/sonic_reference/elf3_step28800_idle_left_001_A019/stream_reference.npz`
- environment variables remain overrides:
  - `BXI_SONIC_MODEL_ONNX`
  - `BXI_SONIC_STREAM_REFERENCE_NPZ`

### B2. Move SONIC Assets Into Official `data/`

Current:

```text
gear_sonic_deploy/policy/elf3_step28800_smpl/model_step_028800_smpl.onnx
gear_sonic_deploy/reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
gear_sonic_deploy/reference/elf3_smpl_Loop_Forward_Walk_001__A019/stream_reference.npz
```

Target:

```text
src/bxi_example_py_elf3/data/sonic_model/
  elf3_step28800_smpl/
    model_step_028800_smpl.onnx

src/bxi_example_py_elf3/data/sonic_reference/
  elf3_step28800_idle_left_001_A019/
    stream_reference.npz
  elf3_smpl_Loop_Forward_Walk_001__A019/
    stream_reference.npz        # optional/pending, if still useful
```

Notes:

- Do not move these large files until backup exists.
- Keep SHA256 before and after copying.
- The ONNX is about 60 MiB, so decide later whether it stays in Git or moves to ModelScope/runtime release with a fetch script. For runnable local runtime, package-data placement is correct.

### B3. Move PICO Runtime Code Into Official Python Package

Current:

```text
gear_sonic/scripts/pico_manager_thread_server.py
gear_sonic_deploy/pico_pose_to_smpl_ref_bridge.py
gear_sonic/utils/teleop/zmq/zmq_planner_sender.py
gear_sonic/utils/teleop/zmq/zmq_poller.py
gear_sonic/trl/utils/rotation_conversion.py
gear_sonic/trl/utils/torch_transform.py
gear_sonic/isaac_utils/rotations.py
```

Target:

```text
src/bxi_example_py_elf3/bxi_example_py_elf3/sonic_pico/
  __init__.py
  pico_manager.py
  pico_pose_to_smpl_ref_bridge.py
  elf3_fk_calibration.py
  zmq_messages.py
  zmq_poller.py
  smpl_transforms.py
  rotation_conversion.py
```

Setup change required:

```text
packages=[
  "bxi_example_py_elf3",
  "bxi_example_py_elf3.inference",
  "bxi_example_py_elf3.utils",
  "bxi_example_py_elf3.sonic_pico",
]
```

Add console scripts if preferred:

```text
sonic_pico_manager = bxi_example_py_elf3.sonic_pico.pico_manager:main
sonic_pico_bridge = bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge:main
```

This removes the need to set `PYTHONPATH=$REPO_ROOT` just to import `gear_sonic`.

### B4. Move SONIC Sim2sim Launch Into Official Launch Directory

Current:

```text
gear_sonic_deploy/launch/elf3_bxi_python_sim2sim.launch.py
gear_sonic_deploy/run_pico_live_ref_bxi_sim2sim.sh
gear_sonic_deploy/run_elf3_sim2sim_controller.sh
gear_sonic_deploy/run_pico_live_ref_sources.sh
```

Target:

```text
src/bxi_example_py_elf3/launch/example_sonic_sim2sim.launch.py
script/run_sonic_sim2sim_controller.sh
script/run_sonic_pico_sources.sh
script/run_sonic_bxi_sim2sim.sh
```

Behavior rule:

- official `example_demo.launch.py` should keep official default behavior
- SONIC-specific release gating belongs in `example_sonic_sim2sim.launch.py`
- true runtime flow remains:
  - T1 BXI simulation/demo node
  - T2 official remote_controller
  - T3 PICO manager + bridge
  - controller switches pd_brake -> normal -> sonic_teleop

### B5. Remote Controller Config

Current problem:

- runtime changed device from `/dev/input/jsBattleDragon` to `/dev/input/js0`

Final target:

```text
src/remote_controller/config/xbox_default.yaml
  device: /dev/input/jsBattleDragon
  keyboard 3 -> dance
  keyboard 6 -> sonic_teleop
  LB+X -> btn_5=1 -> dance
  RT+X -> btn_10=7 -> sonic_teleop
```

Optional debug-only config:

```text
src/remote_controller/config/xbox_sonic_js0_debug.yaml
  device: /dev/input/js0
```

The debug config must not replace the official default.

### B6. Restore Official Battle Dragon Device Rules

Restore from official baseline:

```text
script/bxi-battle-dragon-link
script/bxi-dev.rules
```

Final `bxi-dev.rules` should include:

- IMU symlink
- BAT symlink
- Battle Dragon add/change/remove rules

This keeps the official stable joystick path and avoids USB enumeration surprises.

### B7. State Machine And Robot State Placement

Keep:

```text
src/bxi_example_py_elf3/config/elf3_state_machine.yaml
src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py
```

Required final semantics:

- `DanceState` remains a separate state
- `SonicTeleopState` remains a separate state
- `dance_event` uses `btn_5=1`
- `sonic_teleop_event` uses `btn_10=7`
- normal -> sonic_teleop transition exists
- sonic_teleop -> normal / pd_brake / recover / zero_torque transitions exist

### B8. What Becomes History Or Pending

After backup and after equivalent official-package entries exist:

Move to history/pending, not delete immediately:

```text
gear_sonic_deploy/
gear_sonic/
pico_manager_thread_server.py
pico_pose_to_smpl_ref_bridge.py
run_pico_live_ref_sources.sh
script/pico_manager_thread_server.py
```

Suggested holding location:

```text
history/sonic_sidecar_legacy_20260709/
pending_assets/
```

But do this only after the BXI official-package path passes sim2sim.

## 4. Backup Gate Before Moving Files

Before any file move/copy cleanup:

```text
/data/ros2_ws/备份/
  bxi_rl_controller_ros2_example-main_before_sonic_layout_YYYYMMDD_HHMMSS/
  GR00T-WholeBodyControl_before_sonic_layout_YYYYMMDD_HHMMSS/
```

Backup only the two local directories. Cloud repositories rely on their own history and should not be copied here unless separately requested.

## 5. Recommended Implementation Order

1. Backup local BXI runtime and GR00T repo.
2. Restore official input-device behavior:
   - `/dev/input/jsBattleDragon`
   - `bxi-battle-dragon-link`
   - Battle Dragon udev rules
   - keep SONIC mappings.
3. Add `bxi_example_py_elf3.sonic_pico` package skeleton.
4. Move/copy small Python PICO/bridge utilities into `sonic_pico/`; remove `gear_sonic.*` imports from the new path.
5. Add `elf3_fk_calibration.py` and test ELF3 URDF loading.
6. Add ELF3-native wrist index output path, keep `pico_g1_legacy` as comparison.
7. Add official SONIC launch/scripts under `launch/` and `script/`.
8. Copy ONNX/reference into `src/bxi_example_py_elf3/data/sonic_model` and `data/sonic_reference` after checksum recording.
9. Update `sonic.py` and `bxi_example_demo.py` defaults to package data paths.
10. Build and run sim2sim through the official controller state machine.
11. Only after successful sim2sim, mark `gear_sonic_deploy/gear_sonic` as legacy/pending.

## 6. Validation Commands

After implementation, these checks should pass:

```bash
cd /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main

rg -n "gear_sonic_deploy|gear_sonic\\.data\\.robot_model|instantiate_g1|get_g1|G1_|left_wrist_yaw_link|right_wrist_yaw_link" \
  src/bxi_example_py_elf3 script src/remote_controller

rg -n "/dev/input/js0|JS_EVENT_INIT" src/remote_controller script

colcon build --symlink-install

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
import inspect
import bxi_example_py_elf3.inference.sonic as sonic
print(inspect.getfile(sonic))
print(sonic.DEFAULT_MODEL_ONNX)
print(sonic.DEFAULT_STREAM_REFERENCE)
PY

ros2 launch bxi_example_py_elf3 example_sonic_sim2sim.launch.py
```

Expected direction:

- `rg gear_sonic...` should have no active runtime hits in final BXI package code.
- `/dev/input/js0` should appear only in an explicitly named debug config, if kept.
- `sonic.DEFAULT_MODEL_ONNX` should resolve under `install/share/bxi_example_py_elf3/data/sonic_model/...`.
- `sonic.DEFAULT_STREAM_REFERENCE` should resolve under `install/share/bxi_example_py_elf3/data/sonic_reference/...`.

## 7. Open Decisions For User Review

1. Use `elf3_dof29_hand` as FK reference by default.
   - Recommended because policy action dimension is 29 and head joints are locked.
2. Whether to package only `elf3.urdf` or full `urdf/meshes`.
   - Recommended: test URDF-only first; package meshes only if Pinocchio/runtime requires them.
3. Whether to keep the old loop-forward reference.
   - `elf3_smpl_Loop_Forward_Walk_001__A019/stream_reference.npz` should be pending until confirmed useful.
4. Whether ONNX/reference go into Git or only runtime release/ModelScope.
   - Runtime layout should still be under `data/sonic_model` and `data/sonic_reference`; storage backend can be decided later.
5. Whether G1 hand IK is needed for data collection.
   - Recommended default: not required for ELF3 29DOF SONIC runtime.
