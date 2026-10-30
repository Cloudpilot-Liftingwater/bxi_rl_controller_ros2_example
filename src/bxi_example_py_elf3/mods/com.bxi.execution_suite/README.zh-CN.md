# BXI 完整测试与人工模型测试（2026-10-30 更新）

真机和仿真现在使用**同一份手柄配置、同一套六步流程**，详见
[统一真机启动与验收](HARDWARE.zh-CN.md)。真机入口复用原 `example_demo_hw.launch.py`，
同时显式启用套件和 manual 模式；LB+B 可直接从启动后的 PD 进入测试待机。
原版默认入口未启用套件时，仍保留其原来的进入规则和键位。

| 步骤 | 两边相同的手柄操作 | 等待结果 |
| --- | --- | --- |
| 1 | LB＋B | 悬挂待测，等 READY |
| 2 | 单按 B 后松开 | 自动完成关节行程→300秒振动→持续跑步 |
| 3 | RB＋Y | 结束跑步、回中、进入normal；仿真自动释放虚拟挂绳，真机由测试者放绳 |
| 4 | LB＋A | CXK完成后自动回normal等待 |
| 5 | LB＋X | SONIC06完成后自动回normal等待 |
| 6 | LB＋Y | SONIC11完成后自动回normal等待 |

模型执行时 RB＋X 可主动中止并回normal；View/Back 单键仍为零力矩退出。
所有组合键均无长按等待；保持已有按键释放互锁和阶段隔离。

源码部署并构建后，可以直接使用安装包入口运行仿真，不依赖下文的本地任务辅助脚本：

```bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
export BXI_CXK_ROOT=/path/to/com.bxi.cxk
export BXI_SONIC_ROOT=/path/to/com.bxi.sonic
ros2 launch bxi_example_py_elf3 example_launch_execution_suite.launch.py
```

另一终端使用相同环境和ROS域，运行下面的手柄命令；真机入口及记录器命令见
[统一真机启动与验收](HARDWARE.zh-CN.md)。配套评测器为
[elf3-model-execution-eval 0.4.0](https://github.com/Cloudpilot-Liftingwater/elf3-model-execution-eval)。

默认改为**人工逐项启动**：原 B 关节行程→300 秒振动→跑步保持不变，
在跑步阶段按 C，受控回中位并进入 normal；随后保持 normal，等待测试者选择动作。
真机放绳仍由测试者判断和操作，下面的键盘操作不控制物理挂绳。

## 当前直接启动流程

先关闭旧仿真及旧键盘进程。终端一：

```bash
bash /home/huangchenwei/Documents/Codex/2026-10-26/elf3-model-execution-eval/work/full_sim2sim.sh run
```

终端二：

```bash
bash /home/huangchenwei/Documents/Codex/2026-10-26/elf3-model-execution-eval/work/full_sim2sim.sh keyboard
```

| 键 | 当前作用 |
| --- | --- |
| P → T → 等 READY → B | 原 B 三阶段顺序测试；振动完整 300 秒 |
| C | 跑步进入 playback 后，结束跑步，受控回中并进入 normal；不自动开始舞蹈 |
| D | 从 normal 启动选定 CXK `_load2`，约 109.08 秒 |
| F | 从 normal 单独启动 SONIC 06，约 5.62 秒 |
| G | 从 normal 单独启动 SONIC 11，约 8.70 秒 |
| N | 从模型动作主动返回 normal；也能撤回正在准备资源或进入模型的过渡 |
| Q | 原有零力矩退出；不是站立恢复键 |

每个动作完整结束后自动回 normal 并等待；支持任意顺序、重复同一动作。
N 在动作执行或退场过渡中使用短 0.1 秒混合，冻结上一条已下发目标并立即采样 normal，
不继续推进被中止模型。入场过渡/资源准备时的 N 使用 normal 自路由撤回。
提前退出记录 `cancelled`，完整播放记录 `playback_complete`；两者均不等于精度合格。
既有 IMU 锁定和故障保护不因 N 而解除，回 normal 也不是对任意倾倒状态的恢复保证。

2026-10-30 现场仿真回归修复：真实 Humble 日志器不允许同一调用位置切换日志级别。
原 Session/模型的动态 INFO→WARN 调用会在 RB＋X 时抛异常并停控制线程；
现采用固定级别的独立调用位置，并隔离日志后端异常，保留旁路结构化事件。
CXK、SONIC06、SONIC11均复用原 BXI running blend；仿真入场为0.3秒，
硬件入场仍为原2秒，三个模型的人工回normal仍为0.1秒；模型输出未改。

保留仿真自动模式：

```bash
ELF3_EXECUTION_MODE=auto bash /home/huangchenwei/Documents/Codex/2026-10-26/elf3-model-execution-eval/work/full_sim2sim.sh run
```

auto 中 N 中止会停止后续自动推进。真机只接受 manual，不能用 auto 连测。
仿真原有 normal 后释放虚拟悬挂的行为保持不变。

## 手柄组合键（2026-10-30 简化版，无长按等待）

使用 `remote_controller/config/execution_suite_gamepad.yaml`，不再用默认
`xbox_default.yaml` 启动测试。LB/RB 是左/右肩键，RT 在此配置中不参与操作。
所有组合键只需按下，不要求保持0.5秒；View/Back 不要求保持2秒。

| 手柄操作 | 对应键盘 | 作用 |
| --- | --- | --- |
| RB＋B | P | 启动/零力矩状态进入 pd_brake |
| LB＋B | T | 进入悬挂测试 idle，等待 READY |
| 单按 B 后松开 | B | 原关节行程→300 秒振动→持续跑步；原B执行中再次单按仍受控取消 |
| **RB＋Y** | C | 人工结束顺序测试的跑步，受控回中并进入 normal |
| **LB＋A** | D | 从 normal 启动 CXK load2 |
| **LB＋X** | F | 从 normal 启动 SONIC 06 |
| **LB＋Y** | G | 从 normal 启动 SONIC 11 |
| **RB＋X** | N | 主动回 normal，或撤回资源准备/入场/退场；不用先松开 LB |
| **View/Back 单键（SELECT）** | Q | 按下即发送零力矩退出，优先于normal和模型请求；其他键未松开也能响应 |

本次只读检查已连接 PowerA 的 Linux 映射：`js.button.10 = BTN_SELECT (0x13a)`，
对应 View/Back 功能键；设备仍为 `/dev/input/js0`。不同手柄先核对实际编号。
该键没有软件长按等待，实际生效仍经过ROS和控制周期，不替代硬件急停。

每次连接后先全部松开；普通动作提交后，松开肩键和面键再发下一条。
同一组按住期间只提交一次，两个键的按下和松开顺序可变。
B 只有从头到尾始终单按并松开才发出原B事件，组合过程中经过 B 不会变成单按。
RB＋X 会丢弃尚未发布的模型请求；View/Back 会丢弃normal及模型请求，
不受重新松键、连接初始化或落地回中时的松键互锁阻挡。
零力矩后的残留按键不会自动重新启动normal，仍需全部松开后重新操作。

### 各阶段允许的遥控指令

| 当前阶段 | 允许执行的指令 |
| --- | --- |
| 启动/零力矩 | RB＋B 进入PD；LB＋B 进入悬挂待测；View/Back 零力矩 |
| pd_brake | LB＋B 进入悬挂待测；RB＋X 进入normal；View/Back 零力矩 |
| 悬挂待测 idle | B 启动原全流程；View/Back 退出；原键盘A/X/Y仍可独立测关节/跑步/振动 |
| 顺序测试的关节行程、振动 | 自动完成后进入下一阶段；B受控取消回待测；View/Back 退出 |
| 顺序测试的跑步 | RB＋Y结束跑步并准备进入normal；B受控取消回待测；View/Back 退出 |
| 回中/落地准备 | 自动完成后进入normal；View/Back 退出 |
| normal | LB＋A/X/Y选择三个模型；RB＋X保持normal/撤回资源准备；View/Back 退出 |
| 模型执行 | RB＋X中止并回normal；View/Back 退出；完整播放后自动回normal |

独立的旧键盘单项测试仍按原按键受控停止；其内部测试算法未改。
躺下、起身、旧独立舞蹈、前后走、零位和软件启停不在测试配置中。
控制器还会按阶段过滤其他入口的遥控事件：PD不打断落地模型测试，
执行中不直接切到另一个模型，C只在顺序测试的跑步阶段接收。
摇杆不生成行走速度；温度、IMU和故障保护仍执行原逻辑，锁定保护不由普通遥控解除。

在 MuJoCo 中用以下命令替换终端二的 keyboard 命令：

```bash
bash /home/huangchenwei/Documents/Codex/2026-10-26/elf3-model-execution-eval/work/full_sim2sim.sh gamepad
```

真机部署后选择同一测试配置，控制节点设置
`BXI_EXECUTION_SUITE=1`、`BXI_EXECUTION_SUITE_MODE=manual`，其余部署条件见下节。
推荐直接使用 `example_launch_execution_suite_hw.launch.py`，它将这些设置传入原硬件入口，
默认启动同一份手柄配置；不再需要为真机重新记一套按键。
现场已经单独启动硬件、IMU和控制节点，手柄只负责输入时使用：

```bash
ros2 run remote_controller remote_controller --driver joystick --disable-system-actions \
  --config "$(ros2 pkg prefix remote_controller)/share/remote_controller/config/execution_suite_gamepad.yaml"
```

同一测试只运行一个遥控输入进程。任务脚本使用独立的 `work/unified-suite-install`；
`ELF3_SUITE_INSTALL` 可指定其他安装目录。旧窗口仍是旧映射，重启后才采用新版本。

验证目标 `remote_controller_confirmed_gamepad_test` 覆盖24种两键按下×松开顺序、
44种多面键状态、View在64种其他键保持状态下立即退出、无长按确认、
残留B不重放、连接时按键未松开、源丢失，以及normal/零力矩优先级。
`test/test_confirmed_gamepad.py` 将真实映射送入原事件解析器、两个Mod互锁与状态机，
验证按阶段隔离、三个动作的主动退出，以及回中松键互锁不阻挡零力矩。
此前默认手柄、Linux `js_event` 驱动及键盘测试继续回归。
这些是离线验证，实体手柄与完整GUI流程仍需使用新安装目录复核。

## 温度、故障和评测

三个新模型状态以及独立 dance_test 都复用 `com.bxi.suspended_tests/config` 和原
`temperature_fault`：温度反馈缺失、非法、0.5 秒过期或达到型号停止值时，
写 ERROR、故障码和具体原因，并执行原故障退出；当前停止值为 75°C/85°C。
头部存在时按原消息名称检查温度，不截取前29项。
温度检查覆盖模型采样和入/退场采样；仿真温度标记为不适用，不假造真机读数。
驱动温度保留记录，不擅自设置缺少规格的驱动器温度阈值。

旁路 eval 新增温度原始数组、各关节温度范围、故障分类、级别、故障码及详细值。
结束仿真后自动生成 HTML、JSON、CSV 和对齐数组。
默认加载 `elf3-dynamic-screening-v1` 工程初筛：逐关节 MAE≤3°、RMSE≤5°、P95≤10°、
|偏差|≤3°、峰值≤20°，至少2秒/50次有效独立模型事件/95%有效时间覆盖。
这是待健康真机基准校准的初筛，不是产品规格；精度超限不触发停机。
`ELF3_EVAL_PROFILE=/path/to/profile.json` 可替换配置；完成、中止、数据完整性和精度分开报告。

## 现场人工部署范围

本任务只修改独立工作副本，没有安装到机器人，也没有启停硬件或发布硬件命令。
现场窗口由测试者将本次补丁/源码部署到实际使用的 robot_test 工作区并构建，
配置真实存在的 `BXI_CXK_ROOT`、`BXI_SONIC_ROOT`，使用本项目匹配的 eval 版本。
在既有硬件启动环境显式设置 `BXI_EXECUTION_SUITE=1`、`BXI_EXECUTION_SUITE_MODE=manual`；
原硬件入口和硬件/IMU启动方式沿用现场配置，不另起第二个关节控制器。
模型观察通过 `ELF3_EVAL_ENABLE=1` 和双方相同的 `ELF3_EVAL_SOCKET` 启用。
不设置 BXI_EXECUTION_SUITE 时不能进入新增套件，普通原版操作保持可用。

使用现场相同的 ROS_DOMAIN_ID 和 ROS 环境，单独运行原 C++ remote_controller 的
`--keyboard --disable-system-actions --config <安装目录>/share/remote_controller/config/execution_suite_keyboard.yaml`。
该配置只发布人工输入，不负责启动硬件；与现场现有遥控输入协调选择一个输入源。
仿真脚本固定本机隔离域，不能直接当作真机入口。

新键盘的协议是：D=`btn_6=3`、F=`btn_6=4`、G=`btn_6=5`、N=`btn_1=1`、C=`btn_9=3`。
手柄复用同一协议，组合键和启动方式见上节。
记录端使用 `elf3-eval record --seconds 0 --robot-id <机器人编号> --output <新目录>`，
默认订阅 `/hardware/actuator_states`、`/hardware/actuators_cmds` 和状态话题。

先核对31维关节名/温度，确认原B三阶段正常，RB＋Y回normal后由测试者放绳，
分别启动 D/F/G（手柄 LB＋A/X/Y）；每项都验证中途 N（RB＋X）、
完整结束回normal、温度和故障记录，再看三组角度误差。
故障注入仅用离线测试，不能为验证超温保护而故意加热真机。

---

以下为原自动 sim2sim 的实现及现场问题历史；当前默认行为以上述人工流程为准。

# 原自动 BXI sim2sim 测试（历史）

这套入口在 `robot_test` 原 B 顺序测试之后追加 CXK 舞蹈、SONIC 06、SONIC 11。
使用官方 BXI `mujoco/simulation`、原 `bxi_example_py_elf3_demo` 状态机、原
`remote_controller --keyboard`，全程只有一个关节指令发布者。
不使用独立 Python 物理循环替代 ROS 测试，也不启动旧的独立悬挂控制节点。

## 启动

本任务副本已构建，先关掉此前的仿真。终端一：

```bash
bash /home/huangchenwei/Documents/Codex/2026-10-26/elf3-model-execution-eval/work/full_sim2sim.sh run
```

终端二，启动官方键盘驱动：

```bash
bash /home/huangchenwei/Documents/Codex/2026-10-26/elf3-model-execution-eval/work/full_sim2sim.sh keyboard
```

脚本加载 ROS2/BXI/本任务 overlay，分别执行：

```bash
ros2 launch bxi_example_py_elf3 example_launch_execution_suite.launch.py
ros2 run remote_controller remote_controller --keyboard --disable-system-actions --config /path/to/install/share/remote_controller/config/execution_suite_keyboard.yaml
```

终端二需要是普通交互终端，点击终端后按键；无需手柄。

## 完整顺序

| 环节 | 操作和完成条件 |
| --- | --- |
| 初始化 | 先完成原 BXI `step1`，保持虚拟悬挂并启动控制，初始状态 `pd_brake`；此时保留 `step2` 不发送 |
| 确认与准备 | 按 **P** 确认 PD，再按 **T** 进入测试待机，等日志 `preparation complete; X/Y/A ready` |
| 1. 关节行程 | 按 **B** 开始原 B 顺序测试；原 A 全关节轨迹、反馈校验和受控回中位保持不变 |
| 2. 扫频振动 | 自动执行原 **300 秒**振动，结束后回中位，不缩短此阶段 |
| 3. 悬挂跑步 | 自动进入原 X 跑步轨迹；原流程没有自动结束时间。观察完、进入 playback 后按 **C** 结束跑步并继续 |
| 落地准备 | 原停止动作先回中位并检查实际反馈，然后将目标平滑转到 normal 默认姿态，仍保持悬挂 |
| normal 与释放 | 控制器进入原 normal；正常控制至少 0.5 秒后才首次发送原 BXI `step2/release=true`，等待服务确认，随后正常站立 1 秒 |
| 4. CXK 舞蹈 | 自动运行 `_load2`：`cxkv5_15k.onnx + cxkv5.npz`，V3、帧 0、fixed_pos=True，完整约 109.08 秒，然后回 normal |
| 5. SONIC 06 | 自动运行 `dance_hiphop_roof_top_slides_R_001__A313.pkl`，281 帧、50 Hz、约 5.62 秒，然后回 normal |
| 6. SONIC 11 | 自动运行 `dance_i_aint_afraid_002__A467.pkl`，435 帧、50 Hz、约 8.70 秒，然后回 normal |
| 结束 | 日志 `EXECUTION_SUITE playback_complete` 只在三个模型阶段均完整结束且返回 normal 后出现 |

因此按键主线是 **P → T → 等待准备完成 → B → 等到跑步并观察 → C**，后面自动继续。
原 A/Y/X 单项键仍可单独测试，B 再按一次仍是原取消行为；要继续新增环节，必须在跑步阶段按 **C**。
在关节行程或振动阶段按 C 不会跳过当前阶段；跑步的准备/回位阶段也不能提前切到模型测试。
**Q** 退出测试至零力矩，或在终端一 Ctrl+C 关闭此次仿真；故障/中断不会自动进入后续模型。
旧 `mujoco_dance.sh` 已转到本入口，不再使用上一次 Python 键盘服务释放流程。

## eval 记录与判断

`run` 先启动旁路记录器，再启动仿真和控制器；默认持续记录直到本次仿真结束，不再于 1800 秒自动停录。
可用 `run 2400` 显式设置时限；录制到时或异常结束时，此入口会关闭自己启动的仿真并收尾，避免继续执行未录制的测试。
文件默认在 `/data/tmp/elf3_execution_suite/run-*`，减少根分区占用。
所有源均为本机 domain 73、`simulation/*`，不会启动硬件、CAN、PICO、相机或夹爪。
运行结束后在终端一 Ctrl+C，记录器正常收尾并生成 `report/report.html`、JSON 和对齐数组。
若只需要启动官方 ROS 仿真链路、不需要录制，使用 `full_sim2sim.sh sim`。

查看每个模型段的 `actual_minus_model`，同时比较 `actual_minus_command` 和 `command_minus_model`。
前三个传统测试没有神经网络角度目标，报告为指令—反馈误差，不冒充模型跟踪。
模型切换和进出混合单独标记；SONIC 源混合也标为 blend。原始数据不按失败或缺失补零。

本 Mod 的 SONIC 文件输入使用原策略的 gather/infer/advance 顺序和现有 NumPy SMPL/手腕转换结果，
执行真实 SONIC 模型。通过 BXI 控制器下发，反馈来自官方 MuJoCo 的 ROS 话题。
不启动 PICO/ZMQ 采样端，末尾未来窗口保持末帧；不会把 SMPL 参考角直接当作 `q_model`。
SONIC 原动作限幅、缩放、偏置及源混合沿用原策略，没有为缩小误差新增目标角裁剪。
`assets/selection.json` 保存两段的原始文件名、帧率、帧数和指纹。

MuJoCo 没有电机热模型，因此仅在显式 `BXI_EXECUTION_SUITE=1` 且话题前缀为 `simulation/` 时，
热测试标记为 `not_assessed`；不虚构温度，硬件缺温度保护保持不变。
全套流程完成与角度精度合格分开报告，不预设统一精度阈值。

## 2026-10-29 释放悬挂修复

现场日志表明 C 已完成跑步交接并进入 normal，但模拟器随后报告 `error reset step: 2`。
旧代码在启动时发送了 `step2/release=false`，到 normal 后又发一次 `step2/release=true`。
原模拟器接受的顺序是 `step1 → step2`，重复 step2 会拒绝并重置协议状态；这不是时序 WARN 导致的。
现已复用官方 sim2sim 的顺序：完成 step1 后开始悬挂控制，直到 normal 才发一次 step2。
控制器的内部 `step=2` 表示可接收按键/控制，不再表示已发送模拟器的 step2；
状态话题增加 `simulation_release.confirmed/failed/step2_sent` 供核对。
旧进程需要关闭后重新启动；不能靠继续按 C 修复已被旧请求推进的复位状态。

## 本轮验证

已检查原悬挂状态、取消/故障、阶段交接、SONIC 真实推理与观察输出、模型资源加载和官方 C++ 键盘映射。
完整 ROS/DDS/GUI 运行需要工作站允许套接字，当前任务环境没有该权限；构建与离线结果不能代替实际跑完本流程。
