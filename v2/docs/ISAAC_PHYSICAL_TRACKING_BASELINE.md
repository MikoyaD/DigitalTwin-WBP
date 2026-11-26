# 2026-11-25 正式版补充

物理执行默认 `warehouse_physics_self_overlay.usda`；历史简化场景通过 `--stage .../warehouse_color_overlay.usda` 选择。接触事件见运行手册；场景对齐与碰撞边界见遗留问题清单。原有评价字段和历史基线保持不变。

# Isaac P1 物理跟踪基线（2026-11-24）

阶段：`PHYSICAL_TRACKING_BASELINE: ACCEPTED`（用户认可跟踪效果）。已归档分层结论：`FULL_P1_EXECUTION: PASS`、`TRACKING_PERFORMANCE: PASS`、`PHYSICAL_CONSTRAINTS: FAIL`、`JOINT_FEEDBACK_CONSISTENCY: FAIL`、`WAREHOUSE_COLLISION_VALIDATION: NOT_VERIFIED`。实际加速度超出规划评价基准及位置速度积分残差保留为后续执行性能研究问题，不隐去原结论；物理执行不表示实机或安全验证通过。

真实 P1 0–55.084676660 s，240 Hz，PGS；一次 headless 及 GUI 的逐帧数据相同，XY RMSE 0.003004 m，最大 0.007005 m，yaw RMSE 0.007130 rad，六关节最大位置误差 0.019767 rad。逐步 API 速度差分关节加速度最大 12.688 rad/s²，六关节位置变化与 API 速度积分残差最大 0.078738 rad；完整实测与假设见实验室外部证据 `/home/robot/digitaltwin/research_management/evidence/ISAAC_P1_FULL_PHYSICAL_20261124_01/REPORT.md`。

## 控制架构

一个 Isaac 仿真时间插值 P1 CSV 的 q 和 v，yaw 先 unwrap，位置分段线性插值；该插值位置连续但斜率在采样点可变，不保证连续速度或加速度。CSV a 不作为控制输入。底盘 q/v 和臂关节 q 使用相同参考时间，关节 v 仅记录；实际执行并非统一全身动力学或力矩优化。

记参考位姿 (xr,yr,θr)，实际 (x,y,θ)：`ex=cosθ(xr−x)+sinθ(yr−y)`，`ey=−sinθ(xr−x)+cosθ(yr−y)`，`eθ=wrap(θr−θ)`。参考前向速度 `vr=cosθr vxr+sinθr vyr`，`ωr=vθr`；自编反馈为 `v=vr+1.0 ex`、`ω=ωr+1.5 eθ+1.5 sign(vr) ey`（vr=0 取正号）。其结构是参考速度前馈加车体坐标误差反馈，不能因形式相似直接称为某个特定命名控制算法。

按当前实际航向计算 v 的允许范围，使 world x/y 命令分量各不超过 0.5 m/s，相邻 world 命令分量变化不超过 1 m/s²；ω 限 ±0.5 rad/s、命令变化率限 1 rad/s²。Isaac `DifferentialController` 依据半径 r=0.06 m、轮距 b=0.395 m 计算 `ωL=(2v−bω)/(2r)`、`ωR=(2v+bω)/(2r)`。左右轮由 `ArticulationController.apply_action(ArticulationAction(joint_velocities=...))` 驱动，kp=0、kd=5、实验力矩上限 10。

机械臂使用相同 ArticulationController 发送六关节位置目标；关节 2 kp=2000，其余 kp=500，kd=50，保留原始 maxForce（此前 USD 探测为各 1000）。位置驱动属于基于关节位置及速度误差的 PhysX 反馈，不等于开环；脚本不显式发送臂参考速度，初始化速度目标为零；没有额外机械臂上层反馈、显式重力补偿或逆动力学。没有调用 Gazebo 控制器、MoveIt 执行控制器或旧 Isaac 项目控制器。

## 运行边界

输入真实 P1 和独立 Stage 可通过脚本参数传入；默认本机路径见脚本。用 `--mode feedback --duration 55.0846766603433 --output <新目录>` 运行 headless；加 `--gui --hold-open` 后保持末态。默认使用重力、临时平面、实验圆柱轮碰撞代理、低摩擦支撑轮、PGS。原始 USD、外部机器人/仓储资源与场景快照及历史轨迹均不包含在本 Git 基线中，见 `LAB_NOETIC_BASELINE.md`。只有初始化时设置根和关节状态，跟踪时仅通过轮和关节驱动。

## 2026-11：正式执行输出约定

正式 `isaac_execute.py` 默认 120 Hz，控制律和接触实验条件未变。执行目录不再使用随机后缀，默认不生成图；详见 `V2_PRP_RUNBOOK.md` 与执行评价文档。240 Hz 历史基线及其物理约束、反馈一致性和仓储碰撞结论保持原样。
