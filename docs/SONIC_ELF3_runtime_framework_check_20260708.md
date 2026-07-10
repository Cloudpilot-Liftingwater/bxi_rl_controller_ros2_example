# SONIC ELF3 runtime 框架确认与必需文件清单

日期：2026-07-08

## 1. 结论

真机成功运行时使用的是 BXI 官方 ROS2 Python 主控框架，不是外部 C++ deploy 框架。

核心链路是：

```text
remote_controller / robot_gateway
  -> /motion_commands
  -> bxi_example_py_elf3_demo
  -> RobotStateMachine
  -> SonicTeleopState
  -> bxi_example_py_elf3.inference.sonic.SonicTeleopPolicy
  -> hardware_elf3
```

也就是说，SONIC 在最终真机 runtime 中是 BXI state machine 里的一个新增运动状态 `sonic_teleop`，不是用 SONIC 替换 dance，也不是单独开一个外部高优先级控制链路。

## 2. 真机框架证据

真机启动指南记录的成功流程使用：

```bash
source /home/bxi/bxi_rl_controller_ros2_example-main/install/setup.bash
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py
ros2 run remote_controller remote_controller --config /home/bxi/bxi_rl_controller_ros2_example-main/gear_sonic_deploy/config/xbox_sonic.yaml
```

成功检查中，ROS topic 关系为：

```text
/motion_commands publisher: ros_bridge_node, COM_publisher
/motion_commands subscriber: bxi_example_py_elf3_demo
/hardware/actuator_states publisher: hardware_elf3
/hardware/imu_data publisher: hardware_elf3
/hardware/state_machine_info current.name = normal
```

键盘配置中同时存在：

```text
3 : keyboard.dance
6 : keyboard.sonic_teleop
```

这说明真机当时已经按“保留 dance、新增 sonic_teleop”的状态机框架启动并验证过。需要注意的是，后来机器人被拿去检修，最新机器人磁盘上的 runtime 包没有再次完整回传；因此当前本地仓库是按真机成功指南和快照重建出的等价 runtime，并已在 sim2sim 通过。

## 3. 本地 runtime 当前状态

本地路径：

```text
/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main
```

当前校验结果：

```text
robot_states.py: src/build/install 三层一致
bxi_example_demo.py: src/build/install 三层一致
inference/sonic.py: src/build/install 三层一致
elf3_state_machine.yaml: src/install 一致
xbox_default.yaml / xbox_sonic.yaml / install remote config 一致
```

`source install/setup.bash` 后，Python import 指向：

```text
bxi_example_demo.py -> install/lib/python3.10/site-packages/bxi_example_py_elf3/bxi_example_demo.py
robot_states.py    -> install/lib/python3.10/site-packages/bxi_example_py_elf3/robot_states.py
sonic.py           -> install/lib/python3.10/site-packages/bxi_example_py_elf3/inference/sonic.py
```

policy 和 clean reference 也已在本地库内：

```text
gear_sonic_deploy/policy/elf3_step28800_smpl/model_step_028800_smpl.onnx
gear_sonic_deploy/reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
```

## 4. 状态机要求

必须满足：

```text
remote_events:
  dance_event -> btn_5=1
  sonic_teleop_event -> btn_10=7

states:
  dance -> DanceState
  sonic_teleop -> SonicTeleopState
```

从 `normal` 可切到：

```text
dance_event -> dance
sonic_teleop_event -> sonic_teleop
```

从 `sonic_teleop` 必须至少能切回：

```text
normal_event -> normal
pd_brake_event -> pd_brake
recover_event -> recover
zero_torque_event -> zero_torque
```

## 5. 当前按键映射

键盘：

```text
! -> pd_brake
1 -> normal
3 -> dance
6 -> sonic_teleop
```

手柄：

```text
RB+B -> pd_brake
RB+X -> normal
LB+X -> dance
RT+X -> sonic_teleop
```

其中 SONIC 输出为：

```text
btn_10=7
```

dance 保持原输出：

```text
btn_5=1
```

## 6. 运行必需文件

真机和 sim2sim 共用的核心 runtime 文件：

```text
src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py
src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py
src/bxi_example_py_elf3/bxi_example_py_elf3/inference/sonic.py
src/bxi_example_py_elf3/config/elf3_state_machine.yaml
src/remote_controller/config/xbox_default.yaml
gear_sonic_deploy/config/xbox_sonic.yaml
gear_sonic_deploy/pico_pose_to_smpl_ref_bridge.py
gear_sonic/scripts/pico_manager_thread_server.py
gear_sonic_deploy/policy/elf3_step28800_smpl/model_step_028800_smpl.onnx
gear_sonic_deploy/reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
```

sim2sim 新增入口文件：

```text
gear_sonic_deploy/launch/elf3_bxi_python_sim2sim.launch.py
gear_sonic_deploy/run_pico_live_ref_bxi_sim2sim.sh
gear_sonic_deploy/run_elf3_sim2sim_controller.sh
gear_sonic_deploy/run_pico_live_ref_sources.sh
```

运行文档：

```text
docs/SONIC_ELF3_PICO_Isaac_sim2sim_startup_20260708.md
docs/SONIC_ELF3_PICO_sim2sim_runtime_report_20260708.md
docs/SONIC_ELF3_runtime_framework_check_20260708.md
```

## 7. 真机能否直接按新框架启动

结论分两层：

1. 历史真机成功记录：已经按该框架成功启动过，且 T1/T2/T3/T4/T5 都有反馈，SONIC 遥操和 dance/normal 等模式切换都成功。
2. 当前机器人磁盘状态：由于最新机器人 runtime 包没有再次完整回传，不能仅凭本地文件断言机器人当前磁盘仍是完全相同版本。下次拿到机器人后，应先执行 import/path/topic/state machine 检查，再上电进入 SONIC。

最低检查命令：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_rc_ros2/setup.bash
source /home/bxi/bxi_rl_controller_ros2_example-main/install/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash

ros2 pkg prefix bxi_example_py_elf3

python3 - <<'PY'
import inspect
import bxi_example_py_elf3.bxi_example_demo as demo
import bxi_example_py_elf3.robot_states as states
import bxi_example_py_elf3.inference.sonic as sonic
print("demo:", inspect.getfile(demo))
print("states:", inspect.getfile(states))
print("sonic:", inspect.getfile(sonic))
print("model:", getattr(sonic, "DEFAULT_MODEL_ONNX", None))
print("ref:", getattr(sonic, "DEFAULT_STREAM_REFERENCE", None))
PY

grep -n -Ei 'dance_event|sonic_teleop_event|to: dance|to: sonic_teleop|behavior: DanceState|behavior: SonicTeleopState|slot: btn_5|slot: btn_10|value: 7' \
  /home/bxi/bxi_rl_controller_ros2_example-main/install/share/bxi_example_py_elf3/config/elf3_state_machine.yaml

grep -n -Ei 'keyboard.dance|keyboard.sonic_teleop|btn_5=1|btn_10=7' \
  /home/bxi/bxi_rl_controller_ros2_example-main/gear_sonic_deploy/config/xbox_sonic.yaml
```

只有确认 import 指向 `/home/bxi/bxi_rl_controller_ros2_example-main/install`，状态机同时保留 `DanceState` 和 `SonicTeleopState`，并且 T2 显示 `3 : keyboard.dance`、`6 : keyboard.sonic_teleop` 后，才进入真机 SONIC。
