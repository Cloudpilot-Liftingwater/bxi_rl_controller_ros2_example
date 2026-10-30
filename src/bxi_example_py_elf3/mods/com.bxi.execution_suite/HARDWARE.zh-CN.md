# ELF3统一真机测试流程

真机和仿真都用 `remote_controller/config/execution_suite_gamepad.yaml`。
这套流程复用原B测试的关节行程、300秒振动和跑步，在同一控制器后面增加三个模型环节。
手柄只发出状态请求，不启动第二个电机指令发布者。

## 操作表

启动新版套件、收到有效反馈并完成PD启动后，按以下顺序执行：

| 步骤 | 手柄操作 | 完成条件 |
| --- | --- | --- |
| 1 | LB＋B | 进入悬挂待测；回中准备完成后输出 READY |
| 2 | 单按B后松开 | 自动完成关节行程→回中→300秒振动→回中→持续跑步 |
| 3 | RB＋Y | 跑步已进入playback后，受控回中、落地准备、进入normal并保持 |
| 4 | LB＋A | 完整执行CXK load2，自动回normal等待 |
| 5 | LB＋X | 完整执行SONIC06，自动回normal等待 |
| 6 | LB＋Y | 完整执行SONIC11，自动回normal等待 |

仿真第3步自动释放虚拟挂绳；真机的物理放绳由测试者判断和操作，代码不发送虚拟挂绳释放请求。
三个模型各自需要人工启动，每项结束后不自动开始下一项。
模型执行、资源准备、入场和退场过程中可用RB＋X撤回并保持normal；中止记录为cancelled。
View/Back单键发送零力矩退出，其优先级高于normal和模型请求；它是软件退出键，不替代硬件急停。
普通操作之间先松开按键；均不要求长按，也不增加0.5秒确认。

View/Back的覆盖范围是输入和控制器已就绪的正常测试流程：PD、待测、原B三阶段、
回中/落地准备、normal，以及三个模型的资源准备、入场、执行和退场。
它通过真实零力矩状态将KP、KD、速度和前馈力矩置零；不会等待动作播完或按键全部松开。
初始化完成前的按键仅同步状态，需在就绪后重新按下；原IMU锁定保护屏蔽遥控事件，
不会被View/Back解除，也不会由它从低刚度保护切到零力矩。
手柄连接、ROS通信或控制线程失效时，这个软件按键也不提供独立停机通道。

## 部署准备

在安排好的现场窗口，将此robot_test工作副本的补丁部署到现场实际使用的控制工作区，
构建 `bxi_example_py_elf3` 和 `remote_controller`，再source新版 `install/setup.bash`。
使用匹配的eval 0.4.0；它的 `elf3_eval` Python包必须能由控制器所用的Python导入。
若使用源码部署，可以在现场ROS启动环境增加：

```bash
export PYTHONPATH=/path/to/elf3-model-execution-eval/src${PYTHONPATH:+:$PYTHONPATH}
export BXI_CXK_ROOT=/path/to/com.bxi.cxk
export BXI_SONIC_ROOT=/path/to/com.bxi.sonic
export ELF3_EVAL_SOCKET=elf3_model_eval_v1
```

CXK目录需有 `assets/cxkv5_15k.onnx`、`assets/cxkv5.npz`；SONIC目录需有
`policy.py`、`assets/sonic_action_token.onnx`、`assets/stream_reference.npz` 及原策略依赖。
两段SONIC参考随本Mod安装在 `assets/sonic06.npz`、`assets/sonic11.npz`。
新硬件入口先核对上述文件和eval导入，再包含原硬件启动入口。

保持现场ROS_DOMAIN_ID、ROS环境、硬件参数和IMU/BMS安排；本入口不会设置仿真隔离域。
同一测试只保留一个遥控输入进程和一个关节控制器。
新入口包含原硬件入口，因此用它替换原 `example_demo_hw.launch.py` 的启动，不能同时再起一份。
`full_sim2sim.sh` 继续专用于仿真。

## 记录与启动

先在终端一录制，`ELF3_ROBOT_ID`填本台机器人编号，记录路径每次新建：

```bash
export ELF3_ROBOT_ID=ELF3-001
mkdir -p "$HOME/elf3-test-runs"
export ELF3_RUN_DIR="$(mktemp -d "$HOME/elf3-test-runs/run-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
printf '%s\n' "$ELF3_RUN_DIR"
python3 -m elf3_eval record --seconds 0 --robot-id "$ELF3_ROBOT_ID" \
  --address "$ELF3_EVAL_SOCKET" --output "$ELF3_RUN_DIR"
```

确认录制进程保持运行、输出目录已出现 `events.jsonl.gz` 后，
在现场已source相同环境的终端二启动：

```bash
ros2 launch bxi_example_py_elf3 example_launch_execution_suite_hw.launch.py
```

该命令调用原 `example_demo_hw.launch.py` 启动硬件和唯一的控制器，
启用 `execution_suite=1`、`execution_eval=1`、manual模式，并启动同一份手柄配置。
`robot_config_file`、`enable_head`、`enable_imu`、`imu_runtime_timeout_sec`
继续作为启动参数传给原入口；默认IMU选择沿用机器人配置。
现场使用独立IMU时传 `enable_imu:=false`，并由既有IMU进程提供反馈。
使用独立手柄进程时传 `start_remote_controller:=false`，手柄命令为：

```bash
ros2 run remote_controller remote_controller --driver joystick --disable-system-actions \
  --config "$(ros2 pkg prefix remote_controller)/share/remote_controller/config/execution_suite_gamepad.yaml"
```

手柄日志应显示 `accepting commands: gamepad`；按操作表完成六步。
`/hardware/actuators_cmds` 应只有一个发布者；记录器只订阅数据。
最终三个模型都完整结束并返回normal时，输出 `EXECUTION_SUITE playback_complete`。
这是流程完成标记；精度由eval另行计算。

也可沿用现场原硬件启动命令，显式添加 `execution_suite:=1 execution_eval:=1`，
同时选择上面的手柄配置；此方式和新包装入口只选一个。

## 停录与报告

六步结束后保持机器人处于现场确定的状态，在终端一Ctrl+C停止记录器，
确认 `capture_result.json` 已写入；机器人停机按现场流程操作。
随后离线生成报告：

```bash
python3 -m elf3_eval import "$ELF3_RUN_DIR" --output "$ELF3_RUN_DIR/input.sqlite"
python3 -m elf3_eval analyze "$ELF3_RUN_DIR/input.sqlite" \
  --output "$ELF3_RUN_DIR/report" --acceptance-profile elf3-dynamic-screening-v1
```

检查三个模型各自的 `q_actual-q_model`，并结合 `q_actual-q_command` 和
`q_command-q_model`定位差异；原B阶段只有指令目标，不能冒充模型输出。
完整播放、人工中止、故障、数据完整性和精度分开报告。
温度沿用原B测试的75°C/85°C停止值和0.5秒新鲜度检查；模型温度保护不会因仿真验证而在真机停用。
精度初筛阈值是工程配置，后续按健康真机基准校准。

## 验收记录

离线回归使用真实C++手柄映射、原B状态和真实状态机，验证同一组按键走完六步；
测试中的反馈由内存目标生成，原300秒振动使用虚拟时钟完整推进。
模型推理/动力学仍以已有sim2sim结果和现场记录分别验收。
部署记录应注明机器人编号、控制器/模型版本、每个模型完成或中止情况、故障和eval结果。
