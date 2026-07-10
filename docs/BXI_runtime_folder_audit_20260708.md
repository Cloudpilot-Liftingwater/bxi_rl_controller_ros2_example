# bxi_rl_controller_ros2_example-main 文件夹审计报告

日期：2026-07-08

审计对象：

```text
/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main
```

## 1. 总体结论

这个目录当前不是一个干净的源码 Git 工作树，而是一个可运行 runtime 快照：

```text
源码 src/
构建产物 build/
安装产物 install/
运行日志 log/
SONIC/PICO 运行脚本 gear_sonic_deploy/
PICO/teleop 支撑代码 gear_sonic/
ELF3 资源 resources/
运行文档 docs/
```

总大小约 309M。其中：

```text
resources              120M
gear_sonic_deploy       59M
gear_sonic              52M
src                     38M
install                 37M
build                  5.2M
docs                   236K
log                    282K
```

当前适合作为“本地可运行 runtime 基准”；不适合原样作为“干净代码仓库”上传。

## 2. 关键 runtime 结论

当前本地 runtime 已经是“保留 dance，新增 sonic_teleop”的结构。

已确认一致：

```text
src/bxi_example_py_elf3/config/elf3_state_machine.yaml
install/share/bxi_example_py_elf3/config/elf3_state_machine.yaml

src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py
build/bxi_example_py_elf3/build/lib/bxi_example_py_elf3/robot_states.py
install/lib/python3.10/site-packages/bxi_example_py_elf3/robot_states.py

src/bxi_example_py_elf3/bxi_example_py_elf3/bxi_example_demo.py
build/bxi_example_py_elf3/build/lib/bxi_example_py_elf3/bxi_example_demo.py
install/lib/python3.10/site-packages/bxi_example_py_elf3/bxi_example_demo.py

src/bxi_example_py_elf3/bxi_example_py_elf3/inference/sonic.py
build/bxi_example_py_elf3/build/lib/bxi_example_py_elf3/inference/sonic.py
install/lib/python3.10/site-packages/bxi_example_py_elf3/inference/sonic.py

src/remote_controller/config/xbox_default.yaml
gear_sonic_deploy/config/xbox_sonic.yaml
install/share/remote_controller/config/xbox_default.yaml
```

`source install/setup.bash` 后，ROS/Python 均解析到当前目录：

```text
ros2 pkg prefix bxi_example_py_elf3 -> /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/install
ros2 pkg prefix remote_controller -> /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/install

bxi_example_demo.py -> install/lib/python3.10/site-packages/bxi_example_py_elf3/bxi_example_demo.py
robot_states.py    -> install/lib/python3.10/site-packages/bxi_example_py_elf3/robot_states.py
sonic.py           -> install/lib/python3.10/site-packages/bxi_example_py_elf3/inference/sonic.py
```

## 3. 真机/仿真主链路

真机主控框架：

```text
remote_controller / robot_gateway
  -> /motion_commands
  -> bxi_example_py_elf3_demo
  -> RobotStateMachine
  -> SonicTeleopState
  -> bxi_example_py_elf3.inference.sonic.SonicTeleopPolicy
  -> hardware_elf3
```

sim2sim 主控框架：

```text
remote_controller
  -> /motion_commands
  -> bxi_example_py_elf3_demo
  -> RobotStateMachine
  -> SonicTeleopState
  -> bxi_example_py_elf3.inference.sonic.SonicTeleopPolicy
  -> simulation/actuators_cmds
  -> mujoco simulation
```

推荐 sim2sim 入口：

```text
gear_sonic_deploy/run_pico_live_ref_bxi_sim2sim.sh
gear_sonic_deploy/run_elf3_sim2sim_controller.sh
gear_sonic_deploy/run_pico_live_ref_sources.sh
```

不要把旧 C++ deploy 当作当前主入口。

## 4. 状态机和按键映射

状态机中应同时存在：

```text
dance_event -> btn_5=1 -> DanceState
sonic_teleop_event -> btn_10=7 -> SonicTeleopState
```

当前映射：

```text
键盘:
  ! -> pd_brake
  1 -> normal
  3 -> dance
  6 -> sonic_teleop

手柄:
  RB+B -> pd_brake
  RB+X -> normal
  LB+X -> dance
  RT+X -> sonic_teleop
```

这个映射没有发现 dance/sonic 冲突。

## 5. 运行必需资产

SONIC/PICO 真机和 sim2sim 共用核心文件：

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

当前 policy/reference 存在：

```text
gear_sonic_deploy/policy/elf3_step28800_smpl/model_step_028800_smpl.onnx             59.7M
gear_sonic_deploy/reference/elf3_step28800_idle_left_001_A019/stream_reference.npz  1.3M
```

`gear_sonic_deploy/reference/elf3_smpl_Loop_Forward_Walk_001__A019/stream_reference.npz` 也存在，但不是当前主流程默认 reference。

## 6. 主要风险

### R1. 不是 Git 工作树

目录内没有 `.git/`。当前不能直接用 `git status`、`git diff`、`git push` 管理。

建议：

```text
先把它当 runtime 快照保存；
再整理出一个干净源码仓库；
最后再决定 build/install/model/reference 哪些走代码库，哪些走模型/数据集存储。
```

### R2. build/install/log 混在目录内

`.gitignore` 已经排除了：

```text
build/
install/
log/
__pycache__/
MUJOCO_LOG.TXT
```

但当前实际目录中包含这些内容。它们是运行有用的快照，不是干净源码资产。

建议：

```text
源码库: 保留 src/, gear_sonic_deploy/脚本, docs/, 必要 gear_sonic 子集
runtime包: 另行打包 install/, policy/, reference/, 启动脚本
构建产物: build/log/__pycache__ 不进正式代码库
```

### R3. remote_controller 的 system.start 仍可能拉起旧路径

当前以下文件一致：

```text
src/remote_controller/config/xbox_default.yaml
gear_sonic_deploy/config/xbox_sonic.yaml
install/share/remote_controller/config/xbox_default.yaml
```

其中 `system.start` 内容是：

```text
ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py ...
ros2 launch bxi_example_bms bms.launch.py ...
su bxi -c "cd /home/bxi/bxi_ws/bxi_voice && ./bin/bxi-voice"
```

手动 T1/T2/T3 启动不会走这个 `system.start`，但如果有人通过手柄/远程 start event 触发它，可能拉起不符合当前指南的旧环境或旧路径。

建议后续专门处理：

```text
要么禁用 system.start；
要么改成 source /home/bxi/bxi_rl_controller_ros2_example-main/install/setup.bash；
要么把它参数化，避免写死机器人旧工作区路径。
```

### R4. 根目录存在旧 PICO 启动副本

重复文件：

```text
pico_manager_thread_server.py
gear_sonic/scripts/pico_manager_thread_server.py
script/pico_manager_thread_server.py
```

这三个内容一致。

重复文件：

```text
pico_pose_to_smpl_ref_bridge.py
gear_sonic_deploy/pico_pose_to_smpl_ref_bridge.py
```

这两个内容一致。

但这两个不同：

```text
run_pico_live_ref_sources.sh
gear_sonic_deploy/run_pico_live_ref_sources.sh
```

根目录版本是旧流程，仍含“手动 release/suspension”语义；主流程应使用 `gear_sonic_deploy/run_pico_live_ref_sources.sh`。

建议：

```text
根目录旧启动脚本移动到 archive 或删除；
保留 gear_sonic_deploy/ 下的新版主入口。
```

### R5. install/setup.* 带本机绝对路径

install 目录中存在：

```text
/home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/install
/data/ros2_ws/bxi_rl_controller_ros2_example-main/install
/opt/bxi/bxi_ros2_pkg-main
```

这对本机 sim2sim 是正常的，但对机器人部署不是可移植安装包。

建议：

```text
机器人端应在目标路径重新 colcon build/install；
或者保持和指南完全一致的 /home/bxi/bxi_rl_controller_ros2_example-main/install。
```

### R6. 文件权限污染

当前 executable 文件约 866 个，其中包括 `.md/.yaml/.xml/.STL/.onnx/.npz/.png` 等普通资产。

这通常来自 zip/tar 或跨系统复制，不影响运行，但不适合代码库。

建议：

```text
.sh 保持 0755；
必要 Python CLI 可 0755；
普通源码、文档、配置、模型、mesh 设为 0644。
```

### R7. 缓存和备份文件

当前发现：

```text
__pycache__ dirs: 24
*.pyc files: 60
*.bak files: 1
```

其中：

```text
gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml.bak
```

建议：

```text
缓存不进代码库；
.bak 若确有价值，改名放 docs/archive 或明确说明来源。
```

## 7. 未发现的问题

初步秘密扫描未发现真实 token、私钥、access key。命中的 `token` 主要是代码中的 `token_state` 和 `smpl_tokenizer` 变量名。

脚本/launch 轻量校验通过：

```text
bash -n gear_sonic_deploy/run_pico_live_ref_bxi_sim2sim.sh
bash -n gear_sonic_deploy/run_elf3_sim2sim_controller.sh
bash -n gear_sonic_deploy/run_pico_live_ref_sources.sh
compile gear_sonic_deploy/launch/elf3_bxi_python_sim2sim.launch.py
compile src/bxi_example_py_elf3/launch/example_demo_hw.launch.py
compile src/bxi_example_py_elf3/launch/example_demo.launch.py
```

## 8. 建议整理方式

建议拆成四类资产：

### A. 官方 BXI 改造源码库

应进入 Git：

```text
src/
gear_sonic_deploy/config/
gear_sonic_deploy/launch/
gear_sonic_deploy/*.sh
gear_sonic_deploy/pico_pose_to_smpl_ref_bridge.py
docs/
README.md
build.sh
script/ 中仍需要的 systemd/udev 文件
```

谨慎进入 Git：

```text
gear_sonic/scripts/pico_manager_thread_server.py
gear_sonic/utils/
gear_sonic/isaac_utils/
gear_sonic/trl/
gear_sonic/data/
resources/
```

其中 `gear_sonic` 和 `resources` 最好确认是否有上游来源，避免把 GR00T/teleop 支撑代码和 BXI 官方库边界混太深。

### B. Runtime 发布包

用于机器人/同事部署：

```text
install/
gear_sonic_deploy/policy/elf3_step28800_smpl/model_step_028800_smpl.onnx
gear_sonic_deploy/reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
gear_sonic_deploy/run_*.sh
gear_sonic_deploy/config/xbox_sonic.yaml
docs/启动指南
```

### C. 模型/数据资产

建议放魔塔或对象存储：

```text
*.onnx
*.npz
*.pkl
large STL/mesh
PICO reference recordings
```

### D. 本地临时产物

不进入 Git，不作为发布包核心：

```text
build/
log/
__pycache__/
*.pyc
MUJOCO_LOG.TXT
*.bak
```

## 9. 下步建议

1. 先决定 `system.start` 是否要修成当前 SONIC runtime 路径，或彻底禁用。
2. 清理根目录旧副本，统一只保留 `gear_sonic_deploy/` 主入口。
3. 用当前目录导出一份“源码清单”和一份“runtime 发布清单”。
4. 等官方仓库地址确定后，把源码清单导入干净 Git repo；不要把当前目录原样初始化成仓库。
5. 模型、reference、mesh 大文件单独纳入魔塔/数据集版本，而不是混在代码仓库里。
