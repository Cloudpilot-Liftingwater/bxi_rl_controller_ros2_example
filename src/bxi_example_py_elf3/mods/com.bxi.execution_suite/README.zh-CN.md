# ELF3独立模型执行测试

本 Mod 只负责 CXK、SONIC 06、SONIC 11 的逐项执行和在线精度初筛。
它从已有 `com.bxi.basic_actions/forward_back` 走路模式进入，不接入原悬挂 B 流程。

## 手柄流程

1. 按原有按键进入 `forward_back` 走路模式（`btn_5=1`）。
2. `LB+RT+A`：控制程序收到 `btn_6=3`，执行 CXK。
3. `LB+RT+X`：控制程序收到 `btn_6=4`，执行 SONIC 06。
4. `LB+RT+Y`：控制程序收到 `btn_6=5`，执行 SONIC 11。
5. 每个动作完成后自动回 `normal`，下一项由测试者单独选择。

按键只在普通入口 [`remote_controller/config/xbox_default.yaml`](../../../remote_controller/config/xbox_default.yaml) 中配置，不新增遥控器 YAML；遥控器 C++ 包不承担这三组映射逻辑。View/Back 仍走已有零力矩安全路径。

## 日志和精度

日志直接进入已有 `test_*_elf.log`，不创建独立日志文件。模型 Mod 输出：

```text
MODEL_TEST ... ACCURACY_PASS: paired=... mae=... p95=... max=...
MODEL_TEST ... ACCURACY_FAIL: ... bad=joint(...)
MODEL_TEST ... ACCURACY_INSUFFICIENT: paired=... required=50
```

比较对象是 `q_actual - q_model`。当前初筛门槛为 MAE≤3°、RMSE≤5°、P95≤10°、最大误差≤20°；完整曲线和三组角度仍由 `elf3-eval` 离线报告输出。

## 代码边界

- `com.bxi.execution_suite`：测试状态、路由和模型资源。
- `com.bxi.dance_test`：CXK decoder 和在线误差统计。
- `sonic_file.py`：SONIC 文件参考适配。
- `com.bxi.suspended_tests`：保持官方原状。

部署前先用官方 MuJoCo 仿真验证，再由现场人员使用普通硬件入口部署；模型资源通过 `BXI_CXK_ROOT`、`BXI_SONIC_ROOT` 提供。
