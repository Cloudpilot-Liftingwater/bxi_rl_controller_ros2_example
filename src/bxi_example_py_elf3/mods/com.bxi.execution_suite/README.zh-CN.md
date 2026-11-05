# ELF3独立模型执行测试

本 Mod 只负责 CXK、SONIC 06、SONIC 11 的逐项执行和在线精度初筛。
它从已有 `normal` 走路状态（也支持 `forward_back` 自动前后行走）进入，不接入原悬挂 B 流程。

## 手柄流程

1. 完成原启动流程，按 `RB+X` 进入 `normal` 走路状态；不需要运行悬挂 B 测试。
   如需自动前后行走，原 `LB+RB+X` 对应 `btn_10=9`。
2. `LT+RT+A`：控制程序收到 `btn_10=31`，执行 CXK。
3. `LT+RT+X`：控制程序收到 `btn_10=33`，执行 SONIC 06。
4. `LT+RT+Y`：控制程序收到 `btn_10=34`，执行 SONIC 11。
5. 每个动作完成后自动回 `normal`，下一项由测试者单独选择。

先按住两个扳机，再点按对应字母键。先松字母键，再松扳机。连续松开至少 150 毫秒后才接受下一次测试。显式启用独立测试入口时，走路状态只接收模型测试、走路和原安全操作，防止组合键半按时启动其他动作。

复用同事 `f3ba2c4` 的 [`remote_controller/config/xbox_default.yaml`](../../../remote_controller/config/xbox_default.yaml)，文件保持逐字节一致，控制器只认领预留的 `btn_10=31/33/34`。`RB+A` 沿用原零力矩，`RB+X` 中止并回 `normal`，Back/View 沿用官方停止程序行为。

## 日志和精度

日志直接进入已有 `*_elf.log`，不创建独立日志文件。模型 Mod 输出：

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

对齐的官方版本为 `f3ba2c47be3cd7f6be97d4387f8a4a602b1bc246`；原有Mod与整个遥控器包均采用该版本，不新增业务映射或遥控配置文件。

## 本轮验证（2026-11-05）

- 247项Python回归通过；原有躺下动作资产测试1项因本地缺文件未运行。
- 两个ROS包构建成功，4项官方遥控器行为测试通过（含模拟进程停止测试，未停止真实进程）。
- 验证有线手柄与CRSF两条输入路径，覆盖三组动作按下/松开顺序、RB+X返回、RB+A在资源准备/入场/执行/退场时进入真实零力矩状态。
- 与官方 `f3ba2c4` 比较，`src/remote_controller`（含YAML）和原有 `basic_actions`、`suspended_tests` 均无差异；本测试未创建新的物理按键组合。
- 未连接或部署真机。
