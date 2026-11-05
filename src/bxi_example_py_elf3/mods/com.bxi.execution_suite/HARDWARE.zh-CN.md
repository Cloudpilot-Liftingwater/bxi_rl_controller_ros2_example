# ELF3模型执行一致性测试

这是独立于原悬挂 B 测试的模型执行测试。测试者先进入已有的
`normal` 走路状态，再逐个选择模型动作；也支持已有 `forward_back` 自动前后行走。
本 Mod 不修改 `com.bxi.suspended_tests`，不接管原 B 流程。

| 操作 | 结果 |
|---|---|
| 进入已有走路状态 | 完成原初始化后，`RB+X` 进入 `normal` |
| 自动前后行走（可选） | `LB+RB+X`，对应 `btn_10=9` |
| `LB+RT+A` | `btn_10=19`，启动 CXK |
| `LB+RT+X` | `btn_10=21`，启动 SONIC 06 |
| `LB+RT+Y` | `btn_10=22`，启动 SONIC 11 |
| 每个动作完成 | 自动回 `normal`，等待下一次人工选择 |
| RB+X | 中止资源准备、入场、动作或退场，返回 normal |
| RB+A | 原有零力矩；可中止准备、入场、执行和退场 |
| Back/View | 沿用官方停止机器人程序，不增加其他映射 |

采用同事最新 [`remote_controller/config/xbox_default.yaml`](../../../remote_controller/config/xbox_default.yaml) 原文件，不修改内容，只在本测试Mod中接收其预留按键值。
不新增遥控器配置文件，不启动第二个关节指令发布器。

模型阶段的日志直接进入既有 `*_elf.log`：

```text
MODEL_TEST ... PASS paired=... targets=... coverage=... worst_joint_deg=...
MODEL_TEST ... FAIL paired=... targets=... coverage=...
MODEL_TEST ... joint=l_knee_y_joint rmse=6.20>5
MODEL_TEST ... INSUFFICIENT_DATA ... reason=...
MODEL_TEST ... ABORTED ...
```

主指标是 `q_actual - q_model`。当前初筛门槛为 MAE 3°、RMSE 5°、P95 10°、最大误差 20°；这些是工程筛查值，不是厂家认证标准。

真机部署时使用普通硬件入口和普通遥控器配置，先在仿真验证，再由现场人员按既有流程启动硬件。模型资源通过 `BXI_CXK_ROOT`、`BXI_SONIC_ROOT` 提供；评测器仍旁路记录完整曲线和 JSON/HTML 报告。

在线摘要按控制周期取样，仅使用已成功发布且不晚于反馈接收时间的模型目标；不复用未来目标，重复反馈不增加样本数，混合阶段不评分。有效时长至少2秒、独立目标至少50帧、时间覆盖率至少95%；样本缓冲上限20000，超过时标为数据不足，不能判通过。百分位在后台线程计算。

实际单点误差超过20°时终止该动作并回normal；MAE/RMSE/P95在动作结束后统计。缺失反馈和不安全姿态保留零力矩保护，原IMU锁定保护优先。完整的1000Hz反馈与三组角度曲线仍以eval离线报告为准。

对齐的官方版本为 `f3ba2c47be3cd7f6be97d4387f8a4a602b1bc246`；原有Mod与整个遥控器包均采用该版本，不新增业务映射或遥控配置文件。
