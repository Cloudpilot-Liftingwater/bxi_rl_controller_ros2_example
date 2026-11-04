# ELF3模型执行一致性测试

这是独立于原悬挂 B 测试的模型执行测试。测试者先进入已有的
`com.bxi.basic_actions/forward_back` 走路模式，再逐个选择模型动作。
本 Mod 不修改 `com.bxi.suspended_tests`，不接管原 B 流程。

| 操作 | 结果 |
|---|---|
| 进入已有走路模式 | `LB+RB+X`，对应 `btn_5=1` |
| `LB+RT+A` | `btn_6=3`，启动 CXK |
| `LB+RT+X` | `btn_6=4`，启动 SONIC 06 |
| `LB+RT+Y` | `btn_6=5`，启动 SONIC 11 |
| 每个动作完成 | 自动回 `normal`，等待下一次人工选择 |
| View/Back | 走现有零力矩安全路径 |

只修改普通入口 [`remote_controller/config/xbox_default.yaml`](../../../remote_controller/config/xbox_default.yaml)。
不新增遥控器配置文件，不启动第二个关节指令发布器。

模型阶段的日志直接进入既有 `test_*_elf.log`：

```text
MODEL_TEST ... ACCURACY_PASS: paired=... mae=... p95=... max=...
MODEL_TEST ... ACCURACY_FAIL: ... bad=joint(...)
MODEL_TEST ... ACCURACY_INSUFFICIENT: paired=... required=50
```

主指标是 `q_actual - q_model`。当前初筛门槛为 MAE 3°、RMSE 5°、P95 10°、最大误差 20°；这些是工程筛查值，不是厂家认证标准。

真机部署时使用普通硬件入口和普通遥控器配置，先在仿真验证，再由现场人员按既有流程启动硬件。模型资源通过 `BXI_CXK_ROOT`、`BXI_SONIC_ROOT` 提供；评测器仍旁路记录完整曲线和 JSON/HTML 报告。
