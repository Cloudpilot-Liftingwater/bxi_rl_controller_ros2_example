# ELF3独立模型执行测试

本 Mod 只负责 CXK、SONIC 06、SONIC 11 的逐项执行和在线精度初筛。
它从已有 `normal` 走路状态（也支持 `forward_back` 自动前后行走）进入，不接入原悬挂 B 流程。

## 手柄流程

1. 完成原启动流程，按 `RB+X` 进入 `normal` 走路状态；不需要运行悬挂 B 测试。
   如需自动前后行走，原 `LB+RB+X` 对应 `btn_5=1`。
2. `LB+RT+A`：控制程序收到 `btn_6=3`，执行 CXK。
3. `LB+RT+X`：控制程序收到 `btn_6=4`，执行 SONIC 06。
4. `LB+RT+Y`：控制程序收到 `btn_6=5`，执行 SONIC 11。
5. 每个动作完成后自动回 `normal`，下一项由测试者单独选择。

按键只在普通入口 [`remote_controller/config/xbox_default.yaml`](../../../remote_controller/config/xbox_default.yaml) 中配置，不新增遥控器 YAML；遥控器 C++ 包不承担这三组映射逻辑。View/Back（实测 `js.button.10`）在同一普通 YAML 中映射为零力矩请求；`RB+X` 随时中止并回 normal。

## 日志和精度

日志直接进入已有 `test_*_elf.log`，不创建独立日志文件。模型 Mod 输出：

```text
MODEL_TEST ... PASS paired=... targets=... coverage=... worst_joint_deg=...
MODEL_TEST ... FAIL paired=... targets=... coverage=...
MODEL_TEST ... joint=l_knee_y_joint rmse=6.20>5
MODEL_TEST ... INSUFFICIENT_DATA ... reason=...
MODEL_TEST ... ABORTED ...
```

比较对象是 `q_actual - q_model`。当前初筛门槛为 MAE≤3°、RMSE≤5°、P95≤10°、最大误差≤20°；完整曲线和三组角度仍由 `elf3-eval` 离线报告输出。

## 代码边界

- `com.bxi.execution_suite`：测试状态、路由和模型资源。
- `com.bxi.dance_test`：CXK decoder 和在线误差统计。
- `sonic_file.py`：SONIC 文件参考适配。
- `com.bxi.suspended_tests`：保持官方原状。

部署前先用官方 MuJoCo 仿真验证，再由现场人员使用普通硬件入口部署；模型资源通过 `BXI_CXK_ROOT`、`BXI_SONIC_ROOT` 提供。

在线摘要按控制周期取样，仅使用已成功发布且不晚于反馈接收时间的模型目标；不复用未来目标，重复反馈不增加样本数，混合阶段不评分。有效时长至少2秒、独立目标至少50帧、时间覆盖率至少95%；样本缓冲上限20000，超过时标为数据不足，不能判通过。百分位在后台线程计算。

实际单点误差超过20°时终止该动作并回normal；MAE/RMSE/P95在动作结束后统计。缺失反馈和不安全姿态保留零力矩保护，原IMU锁定保护优先。完整的1000Hz反馈与三组角度曲线仍以eval离线报告为准。

## 本轮验证（2026-11-04）

- Python回归218项通过：含三组组合键全部按下/松开顺序、走路入口、逐项等待、入场/执行/退场中止、真实SONIC解码、已知角差、缺项/重复/时钟异常和温度保护。
- 原遥控器两项C++行为测试及cppcheck通过；两个ROS包构建成功。
- 原有躺下动作资产测试1项未运行：本地部分克隆未带 `lie_down.onnx`；xmllint在线XSD校验受网络限制。未连接、部署或操控真机。
- `git diff e1d412f -- src/bxi_example_py_elf3/mods/com.bxi.basic_actions src/bxi_example_py_elf3/mods/com.bxi.suspended_tests` 为空；`remote_controller` 仅普通 `xbox_default.yaml` 有差异。
