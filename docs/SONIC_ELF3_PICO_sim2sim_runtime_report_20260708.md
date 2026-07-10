# SONIC ELF3 PICO sim2sim 真机一致链路报告

日期：2026-07-08

## 1. 结论

本轮 sim2sim 已从早期外部 C++ deploy 路线，修正为与真机最终运行组件一致的 BXI Python 控制链路：

```text
remote_controller
  -> /motion_commands
  -> bxi_example_py_elf3_demo
  -> RobotStateMachine
  -> SonicTeleopState
  -> bxi_example_py_elf3.inference.sonic.SonicTeleopPolicy
  -> simulation/actuators_cmds
  -> BXI MuJoCo simulation
```

这条链路和真机运行方式保持一致：SONIC 不是外部独立控制器，而是 BXI 官方状态机中的一个 `sonic_teleop` 运动状态。

## 2. 关键纠偏

早期 sim2sim 使用过：

```text
gear_sonic_deploy/target/release/elf3_deploy_onnx_ref
```

这是 C++ deploy 路径，主要用于验证 ONNX、观测拼装、外部 `ActuatorCmds` 发布和 state gate。它可以作为对照工具，但不能代表真机最终 runtime。

真机最终使用的是：

```text
bxi_example_py_elf3_demo
bxi_example_py_elf3/robot_states.py::SonicTeleopState
bxi_example_py_elf3/inference/sonic.py::SonicTeleopPolicy
```

因此，后续用于任务数据采集和真机前验证的 sim2sim，应优先使用 Python-BXI 路径。

## 3. 当前新增文件

本轮新增主流程文件：

```text
gear_sonic_deploy/launch/elf3_bxi_python_sim2sim.launch.py
gear_sonic_deploy/run_pico_live_ref_bxi_sim2sim.sh
gear_sonic_deploy/run_elf3_sim2sim_controller.sh
```

本轮新增文档已经落在 BXI 官方改造库根目录的 `docs/` 下：

```text
docs/SONIC_ELF3_PICO_sim2sim_runtime_report_20260708.md
docs/SONIC_ELF3_PICO_Isaac_sim2sim_startup_20260708.md
docs/SONIC_ELF3_runtime_framework_check_20260708.md
```

本轮修改但不作为主入口的历史/辅助文件：

```text
gear_sonic_deploy/run_pico_live_ref_deploy_sim2sim.sh
gear_sonic_deploy/run_pico_live_ref_sim2sim.sh
gear_sonic_deploy/run_elf3_suspension_toggle.sh
gear_sonic_deploy/bxi_motion_command_sequence.py
```

这些修改的目的主要是防止误用旧 C++ deploy、旧 all-in-one、手动 suspension toggle 或自动状态序列。

## 4. 目标 BXI 库打包位置

同步到 BXI 官方改造库时，文件放置如下：

```text
/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/
  gear_sonic_deploy/
    launch/
      elf3_bxi_python_sim2sim.launch.py
    run_pico_live_ref_bxi_sim2sim.sh
    run_elf3_sim2sim_controller.sh
    run_pico_live_ref_sources.sh
    pico_pose_to_smpl_ref_bridge.py
    config/xbox_sonic.yaml
    policy/elf3_step28800_smpl/model_step_028800_smpl.onnx
    reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
  docs/
    SONIC_ELF3_PICO_Isaac_sim2sim_startup_20260708.md
    SONIC_ELF3_PICO_sim2sim_runtime_report_20260708.md
    SONIC_ELF3_runtime_framework_check_20260708.md
```

其中 `pico_pose_to_smpl_ref_bridge.py`、`xbox_sonic.yaml`、policy 和 reference 在目标库中已存在；本轮主要补齐启动脚本、launch 和文档。

## 5. 正确启动流程

终端顺序应与真机启动指南一致：

1. T1：启动 ELF3 MuJoCo + BXI Python 主控。
2. T2：启动 `remote_controller`。
3. T3：启动 PICO manager + `smpl_ref` bridge。
4. 在 T2 用控制器切 `pd_brake -> normal -> sonic_teleop`。
5. 在 PICO 端完成校准并进入 POSE。

默认键盘映射：

```text
! -> pd_brake
1 -> normal
6 -> sonic_teleop
3 -> dance
```

默认手柄映射：

```text
RB+B -> pd_brake
RB+X -> normal
RT+X -> sonic_teleop
LB+X -> dance
```

## 6. 放绳逻辑

Python-BXI sim2sim 使用 BXI demo 内置 reset 流程：

```text
robot_reset step1 release=false
等待允许状态
进入 normal 后 robot_reset step2 release=true
再从 normal 切 sonic_teleop
```

这与真机指南的操作顺序一致：先 `pd_brake`，再 `normal`，确认机器人/仿真处于可控站立状态后，再进入 SONIC。

不要再用 `run_elf3_suspension_toggle.sh` 作为主 release 路径。它只保留为历史 RobotReset service 探针。

## 7. PICO 输入路径

当前 PICO live reference 仍走：

```text
gear_sonic/scripts/pico_manager_thread_server.py
  -> ZMQ pose topic, default tcp://127.0.0.1:5556 topic=pose
gear_sonic_deploy/pico_pose_to_smpl_ref_bridge.py
  -> ZMQ smpl_ref topic, default tcp://127.0.0.1:5557 topic=smpl_ref
bxi_example_py_elf3.inference.sonic.SonicTeleopPolicy
  -> subscribe smpl_ref
```

`sonic.py` 默认要求 live reference：

```text
BXI_SONIC_USE_SMPL_REF_ZMQ=1
BXI_SONIC_REQUIRE_LIVE_REFERENCE=1
```

如果 bridge 持续出现 `STALE_INPUT`，不要进入 `sonic_teleop`，或立即从 T2 切回 `normal`。

## 8. 当前验证

已完成：

- 新 launch 文件 Python 编译检查通过。
- 新 shell 脚本 `bash -n` 通过。
- `ros2 launch ... --show-args` 可解析。
- 本机已确认 import 到目标 runtime：

```text
bxi_example_py_elf3.bxi_example_demo
bxi_example_py_elf3.inference.sonic
onnx
```

注意：本机 `onnx` 安装在 `~/.local/lib/python3.10/site-packages`，所以 sim2sim 脚本默认会 `unset PYTHONNOUSERSITE`。真机 root 环境是否需要 `PYTHONNOUSERSITE=1` 仍按真机指南处理。

## 9. 风险与后续

- C++ deploy 路径仍可作为 ONNX / obs / action scale 对照，但不要再作为主 sim2sim 结论。
- PICO 输入质量仍是后续任务数据质量的关键变量，应记录 bridge 的 `received/sent/skipped/input_age_ms/STALE_INPUT`。
- 采集任务数据前，应确认每次都是通过 T2 控制器切状态，而不是脚本自动切状态。
- 如果要在 Isaac 环境中复现，应先保证 BXI ROS2 package、MuJoCo simulation、PICO dependencies 和 `onnx/onnxruntime/pyzmq` 都在同一 Python/ROS 环境中可见。
