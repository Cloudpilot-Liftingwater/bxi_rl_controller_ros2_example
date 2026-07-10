# BXI SONIC official-layout migration report

Date: 2026-07-09

Runtime root:

```text
/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/
```

Pre-migration backup:

```text
/data/ros2_ws/备份/local_before_sonic_layout_20260709_152946/
```

## 1. What Changed

### 1.1 Official controller input behavior restored

Restored the official Battle Dragon stable joystick path while keeping SONIC mappings.

Changed:

```text
src/remote_controller/config/xbox_default.yaml
src/remote_controller/include/remote_controller/config.hpp
src/remote_controller/src/input_driver.cpp
script/bxi-dev.rules
script/bxi-battle-dragon-link
```

Final mapping:

```text
keyboard 3  -> dance
keyboard 6  -> sonic_teleop
LB + X      -> btn_5=1  -> dance
RT + X      -> btn_10=7 -> sonic_teleop
device      -> /dev/input/jsBattleDragon
```

`JS_EVENT_INIT` is skipped again, matching the official input driver behavior.

### 1.2 SONIC assets moved into official package data

Copied SONIC model/reference into the official package `data/` tree:

```text
src/bxi_example_py_elf3/data/sonic_model/
  elf3_step28800_smpl/model_step_028800_smpl.onnx

src/bxi_example_py_elf3/data/sonic_reference/
  elf3_step28800_idle_left_001_A019/stream_reference.npz
  elf3_smpl_Loop_Forward_Walk_001__A019/stream_reference.npz
```

Checksums:

```text
6c0f1fabb80e96fed2a9400ec85c2f7f56f45ab048988b67628db56546a77337  model_step_028800_smpl.onnx
c0816049c33de6e0259fe6498c672e311a3113c24a21ed0753d726b03437304e  elf3_step28800_idle_left_001_A019/stream_reference.npz
64cf729639f711573ac9d5b996a0fcebcf4bfb67c53ad2e22f6a2a46ffff0919  elf3_smpl_Loop_Forward_Walk_001__A019/stream_reference.npz
```

After build, installed defaults resolve to:

```text
install/share/bxi_example_py_elf3/data/sonic_model/elf3_step28800_smpl/model_step_028800_smpl.onnx
install/share/bxi_example_py_elf3/data/sonic_reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
```

Environment overrides are still supported:

```text
BXI_SONIC_MODEL_ONNX
BXI_SONIC_STREAM_REFERENCE_NPZ
```

### 1.3 Official launch behavior separated from SONIC launch behavior

Restored:

```text
src/bxi_example_py_elf3/launch/example_demo.launch.py
```

This file no longer carries SONIC-specific startup release gating.

Added:

```text
src/bxi_example_py_elf3/launch/example_sonic_sim2sim.launch.py
```

This launch keeps the verified SONIC sim2sim gating flow:

```text
pd_brake -> normal -> sonic_teleop
release only after allowed state normal
```

Also changed `bxi_example_demo.py` so the default `startup_release_allowed_states` is empty. That preserves the official default release behavior unless a SONIC-specific launch passes an allowed state list.

### 1.4 SONIC PICO bridge moved into official Python package

Added:

```text
src/bxi_example_py_elf3/bxi_example_py_elf3/sonic_pico/
  __init__.py
  pico_pose_to_smpl_ref_bridge.py
  zmq_messages.py
  pico_manager_legacy.py
  elf3_fk_calibration.py
```

Added console scripts:

```text
sonic_pico_bridge
sonic_pico_manager_legacy
```

`sonic_pico_bridge --help` starts successfully after build.

### 1.5 ELF3 FK reference prepared

Copied ELF3 FK reference model into package data:

```text
src/bxi_example_py_elf3/data/sonic_robot_model/elf3_dof29_hand/urdf/
```

Added:

```text
src/bxi_example_py_elf3/bxi_example_py_elf3/sonic_pico/elf3_fk_calibration.py
```

The helper loads the ELF3 URDF without mesh geometry and resolves these frames:

```text
left_wrist  -> l_wrist_z_link
right_wrist -> r_wrist_z_link
torso       -> torso_link
anchor      -> waist_z_link
```

Zero-pose FK smoke result:

```text
left_wrist  [0.459, 0.178, -0.169]
right_wrist [0.459, -0.178, -0.169]
torso       [0.0, 0.0, 0.35]
anchor      [0.0, 0.0, -0.2265]
```

Important: ELF3 FK is prepared but not yet switched into the live PICO manager default. That switch needs A/B validation.

### 1.6 Official scripts added

Added:

```text
script/run_sonic_bxi_sim2sim.sh
script/run_sonic_sim2sim_controller.sh
script/run_sonic_pico_sources.sh
```

New sim2sim startup:

```bash
cd /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main

# T1
bash script/run_sonic_bxi_sim2sim.sh

# T2
bash script/run_sonic_sim2sim_controller.sh

# T3
bash script/run_sonic_pico_sources.sh
```

T2 keyboard sequence:

```text
! -> pd_brake
1 -> normal
6 -> sonic_teleop
3 -> dance
```

### 1.7 Old duplicate entry files archived

Moved old duplicate root/script files to history instead of deleting them:

```text
history/sonic_sidecar_legacy_20260709/duplicates/root/pico_manager_thread_server.py
history/sonic_sidecar_legacy_20260709/duplicates/root/pico_pose_to_smpl_ref_bridge.py
history/sonic_sidecar_legacy_20260709/duplicates/root/run_pico_live_ref_sources.sh
history/sonic_sidecar_legacy_20260709/duplicates/script/pico_manager_thread_server.py
```

## 2. What Is Still Transitional

These directories still remain:

```text
gear_sonic/
gear_sonic_deploy/
```

Reason:

- `gear_sonic/` is still used by the legacy PICO manager wrapper.
- `gear_sonic_deploy/` is retained as transition/history material and for comparison, but active SONIC model/reference defaults no longer point there.

The current T3 script starts:

```text
bxi_example_py_elf3.sonic_pico.pico_manager_legacy
```

and this wrapper still runs:

```text
gear_sonic/scripts/pico_manager_thread_server.py
```

This is intentional for this migration step. The live PICO manager must be switched to ELF3 FK only after A/B validation.

## 3. Build And Verification

Build command that passed:

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg-main/setup.bash  # if present
cd /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main
PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
PYTHONNOUSERSITE=1 \
colcon build --merge-install
```

Why this exact command:

- the workspace uses merged install layout, so `--merge-install` is required
- local `~/.local/bin/cmake` and user-site setuptools caused build-tool conflicts
- using system `cmake` first and `PYTHONNOUSERSITE=1` made the ROS Humble build stack consistent

Passed checks:

```text
colcon build --merge-install: 4 packages finished
sonic.DEFAULT_MODEL_ONNX points to install/share/.../data/sonic_model/...
sonic.DEFAULT_STREAM_REFERENCE points to install/share/.../data/sonic_reference/...
remote_controller installed config uses /dev/input/jsBattleDragon
remote_controller installed config contains btn_10=7 for sonic_teleop
sonic_pico_bridge --help works
ELF3 FK helper resolves installed URDF and runs zero-pose FK
bash -n passed for new scripts and bxi-battle-dragon-link
```

## 4. Next Required Step

Do not delete `gear_sonic/` yet.

Next work item should be a focused PICO calibration migration:

1. add ELF3 FK path inside the live PICO manager
2. emit wrist joint positions directly in ELF3 native order
3. run same PICO recording through:
   - legacy G1 calibration
   - new ELF3 calibration
4. compare `smpl_ref` wrist/root/torso windows
5. run sim2sim A/B before changing default from `pico_g1_legacy` to `elf3_native`

Only after this passes should `gear_sonic/data/robot_model/model_data/g1/` stop being a runtime dependency.
