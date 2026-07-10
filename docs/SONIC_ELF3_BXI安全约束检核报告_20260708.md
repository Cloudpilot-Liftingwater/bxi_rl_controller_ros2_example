# SONIC ELF3 BXI 安全约束检核报告

日期：2026-07-08

## 结论

本次检核在 `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/` 内找到了能够解释现象的明确代码路径：

`SONIC` 嵌入 ELF3 官方 BXI Python 控制框架后，`SonicTeleopState.on_update()` 会检查机身姿态。如果当前 IMU 推出的 roll 或 pitch 超过 `pi / 3`，约 60 度，代码会主动请求切换到 `zero_torque`。`ZeroTorqueState` 随后向电机发送 `kp=0, kd=0` 的命令帧。因此在仿真或真机上表现为机器人失去支撑、瘫倒，用户感知上类似“电机自动断电”。

这不是 `sonic.py` 推理模型自身对大幅弯腰动作做的限制；`sonic.py` 只返回目标关节位姿和固定 PD 增益，不负责状态机切换、reset 或电机发布。

和旧的 SONIC-only 独立 sim2sim 链路相比，关键差异是：旧 C++ deploy 链路也有 roll/pitch 姿态阈值，但当时启动脚本把 `orientation_safety_mode` 设为 `warn`，超过阈值时只报警并继续发 PD 命令；而 BXI Python 官方框架在 `SonicTeleopState` 中直接切 `zero_torque`。

## 已确认的触发链路

1. BXI demo 加载 SONIC policy 和状态机

文件：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py`

证据：

- 第 41 行导入 `SonicTeleopPolicy`。
- 第 177-190 行根据状态机配置构建 `RobotStateMachine`。
- 第 306-321 行创建 `self.sonic_teleop = SonicTeleopPolicy(...)`。
- 第 340-342 行创建 `<topic_prefix>actuators_cmds` 发布器，因此 BXI demo 是官方框架中的电机命令发布者。

2. 状态机中 SONIC 是正式 state

文件：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/config/elf3_state_machine.yaml`

证据：

- 第 20-22 行：`sonic_teleop_event` 映射到 `btn_10 = 7`。
- 第 121-123 行：`normal` 可切到 `sonic_teleop`。
- 第 215-223 行：`sonic_teleop` 使用 `SonicTeleopState`。
- 第 226-237 行：`sonic_teleop` 可切回 `normal`、`pd_brake`、`recover`、`zero_torque`。

这说明当前本地 BXI runtime 已经不是“用 SONIC 替换 dance”，而是把 SONIC 作为新增运动模式嵌入状态机。

3. SONIC 状态每帧检查姿态，超阈值切零力矩

文件：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py`

证据：

- 第 91-99 行：`SonicTeleopState.on_update()` 中先调用 `ctx.is_orientation_unsafe(ctx.current_quat_xyzw)`。
- 第 92-95 行：如果 unsafe，打印 `sonic teleop orientation unsafe, zero_torque!`，随后 `ctx.request_state("zero_torque", trigger="safety")`，并且本帧不再发 SONIC motor target。
- 同文件第 108-123 行：`ZeroTorqueState` 的 motor frame 为 `ctx.joint_nominal_pos`，但 `kp` 和 `kd` 都是 `np.zeros(ctx.dof_num)`。

4. 姿态阈值是 roll/pitch 约 60 度

文件：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py`

证据：

- 第 641-646 行：

```python
def is_orientation_unsafe(self, quat_xyzw):
    eu_ang = quaternion_to_euler_array(quat_xyzw)
    eu_ang[eu_ang > math.pi] -= 2 * math.pi
    return (np.abs(eu_ang[0]) > (math.pi / 3.0)) or (
        np.abs(eu_ang[1]) > (math.pi / 3.0)
    )
```

也就是 roll 或 pitch 绝对值超过 `math.pi / 3.0`，约 1.047 rad / 60 deg，即判为 unsafe。

5. `request_state("zero_torque")` 会真正切换状态

文件：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/utils/state_machine.py`

证据：

- 第 275-297 行：`request_transition()` 会立即 `_begin_transition()`，除非显式 delay。
- 第 335-360 行：`_begin_transition()` 打印 `switch current -> target ...`，执行 `on_exit()`、`on_prepare_enter()`，并在 instant transition 下立即 `_finish_active_transition()`。
- 第 393-403 行：`_finish_active_transition()` 将 `self.current` 改成目标状态并执行 `on_transition_commit()`。

所以 `SonicTeleopState` 里的 `ctx.request_state("zero_torque", trigger="safety")` 不是日志提示，而是实际状态切换。

6. 零力矩帧会被发到 actuators command

文件：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py`

证据：

- 第 429-443 行：每个 timer 周期更新状态机，如果有 `self.motor_target`，会调用 `send_to_motor(qpos, kp, kd)`。
- 第 448-458 行：`send_to_motor()` 将 `pos/kp/kd` 写进 `communication.msg.ActuatorCmds` 并 publish。

结合 `ZeroTorqueState` 的 `kp=0,kd=0`，可以解释为什么切入 `zero_torque` 后机器人会失去姿态保持。

## 与 SONIC-only 独立链路的差异

参照文件：

- `/data/ros2_ws/GR00T-WholeBodyControl/gear_sonic_deploy/run_pico_live_ref_deploy_sim2sim.sh`
- `/data/ros2_ws/GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/elf3_deploy_onnx_ref.cpp`

旧独立 C++ deploy 链路中：

- C++ 默认也有 `orientation_limit_rad = M_PI / 3.0`，见 `elf3_deploy_onnx_ref.cpp` 第 130-132 行。
- 如果 `orientation_safety_mode == Suppress`，第 1907-1919 行会在 unsafe 时 suppress command。
- 如果不是 suppress，第 1921-1925 行会打印 warning 并继续 command。
- 旧启动脚本第 149 行显式传入 `orientation_safety_mode:=warn`。

因此，旧链路在大幅弯腰超过 60 度时仍继续发布 PD 命令；BXI Python 官方框架则直接切入 `zero_torque`。这是目前最能解释两条链路行为差异的代码证据。

## `sonic.py` 自身检查

文件：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/inference/sonic.py`

确认内容：

- 第 392 行起 `SonicTeleopPolicy` 明确是 policy wrapper。
- 文件头第 3-6 行说明该模块不发布 `ActuatorCmds`、不调用 reset、也不拥有状态机。
- 第 669-675 行：如果没有 live SMPL reference，则返回默认姿态，并设置 `last_status = "waiting_for_live_smpl_ref"`。
- 第 682-684 行：模型输出 action 只做 `np.clip(raw_action, -20, 20)`，再转成 `default_dof_pos + action * action_scale`。
- 第 685-689 行：policy active 时返回目标关节位姿。

没有发现 `sonic.py` 会因为大幅弯腰主动切 `zero_torque` 或关闭电机。

## 其他保护逻辑

BXI runtime README 说明了底层硬件/仿真还有保护：

- `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/README.md` 第 82 行：控制命令丢失超过 100 ms 会触发 out-of-control protection，电机会 disabled。
- 第 90-95 行：硬件节点还有 torque、overspeed、position protection，error count 达到 1000 后电机会退出 enabled 状态。

但这些保护实现不在本次审计的 `/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/` 源码中，底层 `hardware_elf3` 主要来自外部 `/opt/bxi/bxi_ros2_pkg`。因此，本报告不能断言真机上的“电机断电”完全只由 BXI Python `zero_torque` 造成；只能确认当前文件夹内存在一条充分解释现象的主动切零力矩路径。

## 建议验证

下次 sim2sim 或真机复现时，重点抓以下信号：

1. T1 控制器日志是否出现：

```text
sonic teleop orientation unsafe, zero_torque!
switch sonic_teleop -> zero_torque via instant (safety)
```

2. 同时 echo 状态机：

```bash
ros2 topic echo /simulation/state_machine_info --once
# 真机用：
ros2 topic echo /hardware/state_machine_info --once
```

看 `current.name` 是否从 `sonic_teleop` 变成 `zero_torque`。

3. 记录 IMU roll/pitch，在接近 1.047 rad / 60 deg 时观察是否触发。

4. 如果真机日志还出现 `motor power off`、`motor_timeout`、`overspeed`、`torque overrun`，再单独审计 `/opt/bxi/bxi_ros2_pkg` 的 `hardware_elf3` 或让同事提供硬件保护日志。

## 后续修改方向

不建议直接删除安全逻辑。更稳妥的改法：

1. 把 `is_orientation_unsafe()` 阈值参数化，例如 `orientation_limit_rad`。
2. 给 `SonicTeleopState` 增加独立策略：`zero_torque` / `warn` / `hold_last` / `pd_brake`，默认仍保守。
3. 在 sim2sim 先验证 `warn` 模式是否能复现旧 SONIC-only 大幅弯腰能力。
4. 真机前单独讨论安全策略，因为大幅弯腰时直接 `zero_torque` 会摔，但完全放开也可能越过硬件保护或机械安全边界。

当前最小结论：限制 SONIC 大幅弯腰的直接可见约束在 BXI 官方 Python 控制框架的 `SonicTeleopState -> is_orientation_unsafe -> zero_torque` 路径，而不是 SONIC policy 本身。
