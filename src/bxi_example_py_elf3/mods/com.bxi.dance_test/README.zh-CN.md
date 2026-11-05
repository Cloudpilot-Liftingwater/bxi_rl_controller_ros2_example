# 舞蹈模型执行测试

完整官方 BXI sim2sim 测试流程现使用配套 `com.bxi.execution_suite`：
原关节行程、300 秒振动、跑步之后，再运行本舞蹈和 SONIC 06/11。
见 [完整流程](../com.bxi.execution_suite/README.zh-CN.md)。本 Mod 仍保留独立舞蹈状态。

此 Mod 新增独立舞蹈环节，复用 `policies/beyondmimic.py` 的真实动作解码，不另写一套控制器。
仅从 `com.bxi.basic_actions/normal` 进入，沿用基础控制周期（当前 50 Hz）。
现有悬挂 A/B/X/Y、B 顺序测试和单一 actuator 发布者不变。

## 动作和模型

默认读取安装包 `data/dance.npz` 与 `data/dance.onnx`。资源按需加载；缺失或不兼容时拒绝进入舞蹈，
不阻塞基础控制启动。在启动控制程序之前可显式指定：

```bash
export BXI_DANCE_MOTION=/path/to/verified/dance.npz
export BXI_DANCE_MODEL=/path/to/verified/dance.onnx
export BXI_DANCE_VARIANT=auto
export BXI_DANCE_START_FRAME=0
export BXI_DANCE_FIXED_POS=false
```

`auto` 只识别已知的 154/157 维旧策略合同。历史窗口/残差模型不能仅凭维度猜解码方式，
必须根据训练导出合同显式选择 `gravity_isaaclab_v2`（参考角加残差）或
`gravity_isaaclab_v3`（默认角加动作）。其他值：`mjlab`、`gravity_mjlab`、`gravity_isaaclab`。
关节顺序仍必须与对应策略类和模型 metadata 完全一致；不使用 SONIC 的 scale/default。

本任务 2026-10-28 的 MuJoCo 入口默认已改为用户指定的 `com.bxi.cxk/plugin.py::_load2`：
`cxkv5.npz` + `cxkv5_15k.onnx`，`gravity_isaaclab_v3`、`start_frame=0`、`fixed_pos=true`。
动作 5454 帧、50 Hz，完整播放约 109.08 秒，不裁剪、不循环。
`fixed_pos=true` 沿用原策略进入时对齐朝向的处理；不代表冻结全部位移或取消动作自身转向。
通用 Mod 默认仍可读取配套 `data/dance.*`；需显式给控制进程传入选定资产与上述环境变量，
本任务脚本已经设置。环境变量只对新启动的进程生效。

本版舞蹈加载器显式使用 ONNX Runtime，报告记录 backend。尚未验证 RKNN/OpenVINO 的此接入。
动作 NPZ 必须有 `joint_pos`、`joint_vel`、`body_pos_w`、`body_quat_w`；前两者为 N×29，
参考帧速率沿用现有策略的 50 Hz。若 NPZ 有 `joint_names`，会核对模型；没有时记录
`motion_names_observed=false` 并提示其数组顺序沿用原策略合同，现场必须核实动作与模型配套。

## 控制流程

升级配套遥控映射后，在 `normal` 状态按 **LB+X**（RB 松开）开始；再次按同组合键取消并回到 normal。
新事件复用消息中已有的 `btn_6=2`，不会覆盖原来的 LB+A（`btn_6=1`）。进入前后都有平滑过渡。
RB+A 保留零力矩出口，RB+B 可进入 PD 制动。遥控系统 Stop、原 IMU 保护仍有效。

完整播完最后一帧记录 `playback_complete`，以 1 秒过渡回 normal。
人工取消记 `cancelled`；意外离开记 `interrupted`；非有限输出、负增益、软件角度越界、反馈超过
0.2 秒不更新或倾角异常记 `failed` 并请求零力矩。不会裁剪模型输出以掩盖超限。
退出混合阶段保持最后一个舞蹈目标，不再向模型传入超出动作结尾的 time_step。

`playback_complete` 表示动作帧已播放完，不是精度通过，也不保证达到某个反馈误差门槛。
舞蹈没有串入 B 顺序测试；也没有自动把落地舞蹈当作悬挂动作执行。

## 用 elf3-eval 评测

需要安装包含结构化故障事件和温度记录的 **elf3-model-execution-eval 0.4.0**。
本状态支持 normal 事件提前返回，温度保护复用原悬挂测试配置；完整人工选取
CXK/SONIC06/SONIC11 及键盘流程见相邻 `com.bxi.execution_suite/README.zh-CN.md`。
旧版评测器缺少桥接时控制继续运行，但模型输出不会被记录，不能据此出模型误差结论。

```bash
# 在机器人已有 ROS 环境中安装审核后的 eval wheel，然后先启动旁路记录器。
elf3-eval record --seconds 120 --output /path/to/new-dance-run

# 下面的变量必须在启动控制程序之前设置并传入控制进程；不是只设置给记录器。
export ELF3_EVAL_ENABLE=1
export ELF3_EVAL_SOCKET=elf3_model_eval_v1
# 在已安排的现场窗口，用原经验证的入口启动本分支控制程序，再手动 LB+X。
# 若使用本分支硬件入口，可显式把开关传给控制节点：
# ros2 launch bxi_example_py_elf3 example_demo_hw.launch.py execution_eval:=1
```

模型策略和记录器必须在同一主机；当前 Unix datagram 使用该主机的单调时钟。
ROS 记录仍订阅 `/hardware/actuators_cmds`、`/hardware/actuator_states` 和状态话题。
观察错误不改变模型返回值；无记录器时不构造/序列化模型事件。
已在后台运行的遥控服务不会继承另一个终端新 export 的变量；使用上述 launch 参数，
或在现场安排的重启中将变量设置给实际启动控制器的父进程。不要再启动第二套硬件/控制器。

```bash
# 将原始记录带回工作站离线分析；这些命令不会连接机器人。
elf3-eval import /path/to/new-dance-run --output /path/to/new-dance.sqlite
elf3-eval analyze /path/to/new-dance.sqlite --output /path/to/new-dance-report
```

报告中的 `actual_minus_model` 就是 `q_actual - q_model`。
`q_model` 是现有策略 step 返回、已经真实缩放/偏置/残差解码的物理目标角；
`q_command` 取最终 override 处理后的 ROS 消息；`q_actual` 取真实 actuator 编码器反馈。
最终消息的源时间戳关联同一控制周期的模型序号；没有本周期模型的指令明确标记无模型关联。
预热、混合、保持默认不进入 active 模型评分。报告保留缺项、覆盖率及时间阈值，误差不补零。

模型和动作指纹、解码规则、scale/default、动作帧号保存在原始模型事件。
`report.json.test_events` 保留整个记录的生命周期标记（不随分析时间窗筛除），不当作精度合格标准。
只评价编码器跟随；同一闭环模型在不同机器人上产生的目标仍可能不同。

## 离线验证与现场范围

新测试 `test/test_dance_execution.py` 使用生成的 ONNX/NPZ 验证资源合同、真实策略解码、
完成/取消/故障、最终 31 维乱序发布和 eval 已知误差，**生成资产不是可部署舞蹈**。
验证额外需要 `pytest`、`onnx`、`onnxruntime`、`numpy`、`PyYAML`，无需 conda。

现场前核实实际资产 SHA、配套策略、初始姿态、动作时长和输出边界，在 MuJoCo
验证同一资产与控制流程。现场安排好原有安全操作后再录制，核查正常回 normal、记录器计数、
模型覆盖与原始误差。若要下“精度通过”结论，先给出对应动作和关节的验收阈值。
本次实现和离线验证没有启动硬件、遥操或电机；不能称为真机验收通过。
