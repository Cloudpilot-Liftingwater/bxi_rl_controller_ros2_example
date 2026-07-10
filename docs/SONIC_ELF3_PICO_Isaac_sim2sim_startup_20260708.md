# SONIC ELF3 PICO Isaac/MuJoCo sim2sim 启动指南

日期：2026-07-08

用途：给同事在 Isaac/仿真工作站部署 ELF3 SONIC PICO 遥操 sim2sim，并用于后续任务数据采集。

## 1. 运行原则

本流程使用与真机一致的 BXI Python runtime：

```text
bxi_example_py_elf3_demo
  -> RobotStateMachine
  -> SonicTeleopState
  -> inference/sonic.py
```

不要把旧 C++ `elf3_deploy_onnx_ref` 当作主入口。它只适合对照实验。

## 2. 目录要求

推荐目录：

```text
/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main
```

关键文件应存在：

```text
install/setup.bash
gear_sonic/scripts/pico_manager_thread_server.py
gear_sonic_deploy/pico_pose_to_smpl_ref_bridge.py
gear_sonic_deploy/config/xbox_sonic.yaml
gear_sonic_deploy/policy/elf3_step28800_smpl/model_step_028800_smpl.onnx
gear_sonic_deploy/reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
gear_sonic_deploy/launch/elf3_bxi_python_sim2sim.launch.py
gear_sonic_deploy/run_pico_live_ref_bxi_sim2sim.sh
gear_sonic_deploy/run_elf3_sim2sim_controller.sh
gear_sonic_deploy/run_pico_live_ref_sources.sh
```

## 3. Python 依赖

至少需要：

```text
numpy
scipy
onnx
onnxruntime
pyzmq
rclpy
```

检查命令：

```bash
python3 - <<'PY'
import sys
print("python", sys.executable)
for m in ["numpy", "scipy", "onnx", "onnxruntime", "zmq", "rclpy"]:
    mod = __import__(m)
    print(m, getattr(mod, "__version__", "?"), getattr(mod, "__file__", "?"))
PY
```

如果 `onnx` 已安装在 `~/.local`，不要设置 `PYTHONNOUSERSITE=1`。当前 sim2sim T1 脚本默认会自动取消这个变量。

## 4. 终端 T1：仿真主控

```bash
cd /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main

bash gear_sonic_deploy/run_pico_live_ref_bxi_sim2sim.sh
```

正常启动时会打印：

```text
[elf3-bxi-python-sim2sim] using BXI Python SONIC runtime
demo: .../bxi_example_demo.py
sonic: .../inference/sonic.py
```

T1 会启动：

```text
simulation_mujoco
bxi_example_py_elf3_demo
```

## 5. 终端 T2：控制器

键盘模式：

```bash
cd /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main

bash gear_sonic_deploy/run_elf3_sim2sim_controller.sh
```

键盘映射：

```text
! -> pd_brake
1 -> normal
6 -> sonic_teleop
3 -> dance
ESC -> exit
```

手柄模式：

```bash
cd /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main

REMOTE_KEYBOARD=0 bash gear_sonic_deploy/run_elf3_sim2sim_controller.sh
```

手柄映射：

```text
RB+B -> pd_brake
RB+X -> normal
RT+X -> sonic_teleop
LB+X -> dance
```

推荐状态切换顺序：

```text
!  -> pd_brake
1  -> normal
6  -> sonic_teleop
```

## 6. 终端 T3：PICO 输入

```bash
cd /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main

PICO_ENABLE_VIS=1 BRIDGE_LOG_EVERY=1 \
  bash gear_sonic_deploy/run_pico_live_ref_sources.sh
```

PICO 端操作：

```text
A+B+X+Y -> CALIB_FULL / PLANNER
A+X     -> POSE
```

进入 `sonic_teleop` 前，应确认 T3 bridge 没有持续 `STALE_INPUT`。

## 7. 推荐启动顺序

1. 启动 T1。
2. 启动 T2。
3. T2 按 `!` 进入 `pd_brake`。
4. T2 按 `1` 进入 `normal`，等待仿真释放并站稳。
5. 启动 T3。
6. PICO 完成校准，人体和仿真机器人朝向对齐。
7. T2 按 `6` 进入 `sonic_teleop`。
8. PICO 按 `A+X` 进入 POSE。
9. 采集任务数据。

退出 SONIC：

```text
T2 按 1 -> normal
```

## 8. 检查命令

查看 ROS topic：

```bash
source /opt/ros/humble/setup.bash
source /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/install/setup.bash

ros2 node list
ros2 topic info /motion_commands -v
ros2 topic info /simulation/actuator_states -v
ros2 topic info /simulation/imu_data -v
ros2 topic echo /simulation/state_machine_info --once
```

关键预期：

```text
/motion_commands publisher: remote_controller
/motion_commands subscriber: bxi_example_py_elf3_demo
/simulation/actuator_states publisher: simulation_mujoco
/simulation/state_machine_info current.name: normal 或 sonic_teleop
```

## 9. 常见问题

### ModuleNotFoundError: No module named 'onnx'

原因通常不是模型文件缺失，而是 Python 包 `onnx` 不在当前解释器可见路径里。

检查：

```bash
python3 - <<'PY'
import onnx
print(onnx.__version__, onnx.__file__)
PY
```

如果 `onnx` 在 `~/.local`，不要设置：

```bash
PYTHONNOUSERSITE=1
```

### T2 按键无效

检查 `/motion_commands` 是否有 demo subscriber：

```bash
ros2 topic info /motion_commands -v
```

如果没有 `bxi_example_py_elf3_demo` subscription，说明 T1 demo 没活起来。

### PICO 输入旧或断流

T3 会打印：

```text
STALE_INPUT
```

持续出现时不要进入 SONIC；如果已经进入，T2 按 `1` 回 `normal`。

### 不要使用的主流程

以下只作历史/对照：

```text
run_pico_live_ref_deploy_sim2sim.sh  # C++ deploy path
run_pico_live_ref_sim2sim.sh         # old all-in-one
run_elf3_suspension_toggle.sh        # manual RobotReset probe
bxi_motion_command_sequence.py       # non-interactive regression helper
```

## 10. 采集建议

- 每次采集记录 T1/T2/T3 终端日志。
- 记录 PICO bridge 的 `received/sent/skipped/input_age_ms`。
- 记录进入 `sonic_teleop` 前后的 `/simulation/state_machine_info`。
- 若要比较 PICO 数据质量，优先保留 `smpl_ref` ZMQ 或 bridge 日志。

