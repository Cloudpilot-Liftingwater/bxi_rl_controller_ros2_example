# SONIC ELF3 BXI 姿态安全判断逻辑汇总

日期：2026-07-08

## 一句话结论

ELF3 官方 BXI Python 控制框架中，确实有代码会检测当前机器人姿态。当 IMU 姿态换算出的 roll 或 pitch 超过 `pi / 3`，也就是约 60 度时，框架会把当前运动状态切到 `zero_torque`。`zero_torque` 状态发出的电机命令是 `kp=0, kd=0`，所以机器人会失去支撑，表现上接近“电机掉电/瘫倒”。

严格说，当前代码层面确认的是“切到零力矩”，不是直接调用硬件驱动“power off”。如果真机日志里还有 `motor power off`，那可能是底层 `hardware_elf3` 保护进一步触发，相关代码不在这个 BXI runtime 文件夹内。

## 1. 姿态 unsafe 的核心判断函数

文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py`

代码位置：第 641-646 行

```python
def is_orientation_unsafe(self, quat_xyzw):
    eu_ang = quaternion_to_euler_array(quat_xyzw)
    eu_ang[eu_ang > math.pi] -= 2 * math.pi
    return (np.abs(eu_ang[0]) > (math.pi / 3.0)) or (
        np.abs(eu_ang[1]) > (math.pi / 3.0)
    )
```

判断逻辑：

- 输入：`quat_xyzw`，来自当前 IMU 姿态。
- 处理：`quaternion_to_euler_array(quat_xyzw)` 把四元数转换成欧拉角。
- 阈值：`math.pi / 3.0`，约 `1.047 rad = 60 deg`。
- 判定条件：`abs(eu_ang[0]) > 60 deg` 或 `abs(eu_ang[1]) > 60 deg`。
- 也就是 roll 或 pitch 任意一个超过约 60 度，就返回 `True`。

这就是“当前机器人姿态到达某个阈值后被判断为不安全”的核心函数。

## 2. SONIC 状态中的触发点

文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py`

代码位置：第 91-95 行

```python
def on_update(self, ctx: BxiExample, dt: float) -> None:
    if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
        print("sonic teleop orientation unsafe, zero_torque!")
        ctx.request_state("zero_torque", trigger="safety")
        return
```

判断逻辑：

- `SonicTeleopState` 每个控制周期都会进入 `on_update()`。
- 它先调用 `ctx.is_orientation_unsafe(ctx.current_quat_xyzw)`。
- 如果 unsafe：
  - 打印 `sonic teleop orientation unsafe, zero_torque!`
  - 调用 `ctx.request_state("zero_torque", trigger="safety")`
  - 直接 `return`，本周期不再执行 SONIC policy 的正常电机目标输出。

这就是 SONIC 嵌入官方 BXI 框架后，大幅弯腰到阈值时自动退出 SONIC 控制并切零力矩的直接代码。

## 3. 其他运动状态也复用了同一姿态安全判断

同一文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py`

复用同一判断函数并切 `zero_torque` 的状态包括：

- `NormalState`：第 51-55 行

```python
if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
    print("check safe error, zero_torque!")
    ctx.request_state("zero_torque", trigger="safety")
    return
```

- `SonicTeleopState`：第 91-95 行

```python
if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
    print("sonic teleop orientation unsafe, zero_torque!")
    ctx.request_state("zero_torque", trigger="safety")
    return
```

- `DanceState`：第 212-215 行

```python
if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
    print("check safe error, zero_torque!")
    ctx.request_state("zero_torque", trigger="safety")
    return
```

- `HandPlayBackState` / `ApplauseState`：第 390-393 行

```python
if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
    ctx.request_state("zero_torque", trigger="safety")
    return
```

- `HelloState`：第 470-473 行

```python
if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
    ctx.request_state("zero_torque", trigger="safety")
    return
```

- `AmpRunState`：第 640-644 行

```python
if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
    print("check safe error, zero_torque!")
    ctx.request_state("zero_torque", trigger="safety")
    return
```

- `NormalRunState`：第 696-700 行

```python
if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
    print("check safe error, zero_torque!")
    ctx.request_state("zero_torque", trigger="safety")
    return
```

所以这不是 SONIC 独有的设计，而是 BXI 官方 Python 状态机里多个运动状态共同使用的姿态保护逻辑。SONIC 接入这个框架后继承了同一套安全门槛。

## 4. `request_state("zero_torque")` 怎么真正切状态

文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py`

代码位置：第 634-639 行

```python
def request_state(
    self, state_name, trigger="code", transition="instant", delay=0.0
):
    self.state_machine.request_transition(
        state_name, trigger=trigger, transition=transition, delay=delay
    )
```

文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/utils/state_machine.py`

关键逻辑：

- 第 275-297 行：`request_transition()` 解析目标状态；如果 `delay <= 0`，立即 `_begin_transition()`。
- 第 335-360 行：`_begin_transition()` 打印状态切换日志，并准备切换到目标状态。
- 第 393-403 行：`_finish_active_transition()` 把 `self.current` 改成目标状态。

也就是说，`ctx.request_state("zero_torque", trigger="safety")` 是真正改变状态机当前状态，不只是打印一条日志。

典型日志应类似：

```text
sonic teleop orientation unsafe, zero_torque!
switch sonic_teleop -> zero_torque via instant (safety)
```

## 5. `ZeroTorqueState` 发出的电机命令是什么

文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py`

代码位置：第 108-123 行

```python
class ZeroTorqueState(RobotControlState):
    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )
```

判断后的动作：

- 目标位置：`ctx.joint_nominal_pos`
- `kp`：全 0
- `kd`：全 0

这意味着进入 `zero_torque` 后，控制器不再给关节提供 PD 支撑。仿真或真机上就会表现为机器人失去支撑、瘫倒。

## 6. 零力矩帧如何发布到电机命令

文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py`

代码位置：第 429-443 行

```python
self.motor_target = None
transition_active = self.state_machine.update(self.dt, events)
self.state = self.state_machine.current_state_id

if not transition_active:
    self.state_machine.update_current_state(self.dt)
    self.state = self.state_machine.current_state_id

if self.motor_target is not None:
    qpos, kp, kd = self.motor_target
    self.pos_last = qpos
    self.kp_last = kp
    self.kd_last = kd
    self.check_inference_frame_timeout()
    self.send_to_motor(qpos, kp, kd)
```

代码位置：第 448-458 行

```python
def send_to_motor(self, dof_pos_target, joint_kp, joint_kd):
    msg = bxiMsg.ActuatorCmds()
    msg.header.frame_id = robot_name
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.actuators_name = joint_name
    msg.pos = dof_pos_target.tolist()
    msg.vel = np.zeros(dof_num, dtype=np.float32).tolist()
    msg.torque = np.zeros(dof_num, dtype=np.float32).tolist()
    msg.kp = joint_kp.tolist()
    msg.kd = joint_kd.tolist()
    self.act_pub.publish(msg)
```

因此，进入 `ZeroTorqueState` 后生成的 `kp=0,kd=0` 会被写入 `ActuatorCmds` 并发布到：

- sim2sim：`simulation/actuators_cmds`
- 真机：`hardware/actuators_cmds`

具体 topic 取决于启动时传入的 `/topic_prefix`。

## 7. 状态机配置中 `zero_torque` 是正式状态

文件：

`/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/config/elf3_state_machine.yaml`

关键位置：

- 第 1 行：`initial_state: zero_torque`
- 第 139-145 行：定义 `zero_torque` 状态，行为类是 `ZeroTorqueState`

```yaml
zero_torque:
  manifest:
    label: 零力矩
    index: 1
    group: Base
    icon: do_not_disturb_on
  behavior: ZeroTorqueState
```

这说明 `zero_torque` 不是异常分支，而是 BXI 官方框架内的一个正式运行状态。

## 8. 和 SONIC-only 旧独立链路的关键区别

旧 SONIC-only C++ deploy 路径在：

`/data/ros2_ws/GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/elf3_deploy_onnx_ref.cpp`

它也有类似姿态阈值：

- 第 130 行：`orientation_limit_rad = M_PI / 3.0`
- 第 1907-1909 行：roll/pitch 超过阈值则 `orientation_unsafe = true`

但旧启动脚本：

`/data/ros2_ws/GR00T-WholeBodyControl/gear_sonic_deploy/run_pico_live_ref_deploy_sim2sim.sh`

第 149 行传入：

```bash
orientation_safety_mode:=warn
```

所以旧链路超过 60 度时是 warning 模式，会继续发 PD 命令；新 BXI Python 框架是直接切 `zero_torque`。这就是为什么同样大幅弯腰动作，在旧 SONIC-only 链路能继续控制，而嵌入 BXI 官方框架后会瘫倒。

## 最小证据链

可以把整条逻辑压缩成下面这条链：

```text
IMU quat_xyzw
  -> bxi_example_demo.py:is_orientation_unsafe()
  -> abs(roll) > pi/3 or abs(pitch) > pi/3
  -> robot_states.py:SonicTeleopState.on_update()
  -> ctx.request_state("zero_torque", trigger="safety")
  -> state_machine.py request_transition()
  -> current state = ZeroTorqueState
  -> ZeroTorqueState.get_motor_frame()
  -> kp = 0, kd = 0
  -> bxi_example_demo.py:send_to_motor()
  -> ActuatorCmds publishes zero-gain command
  -> robot loses support / appears motor dropped
```

## 复现时建议观察的日志

启动后在 T1 控制器终端观察：

```text
sonic teleop orientation unsafe, zero_torque!
switch sonic_teleop -> zero_torque via instant (safety)
```

同时观察状态机 topic：

```bash
ros2 topic echo /simulation/state_machine_info --once
```

真机则用：

```bash
ros2 topic echo /hardware/state_machine_info --once
```

如果 `current.name` 从 `sonic_teleop` 变成 `zero_torque`，就证明触发了这条 BXI 姿态安全链路。
