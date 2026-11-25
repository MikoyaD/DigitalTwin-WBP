# V2 P1 物理执行数据字典

正式任务默认保存在 `v2/outputs/<任务ID>/`，任务的推荐轨迹为 `Parameterization/P1/recommended/trajectory.csv`；每次执行在该任务的 `Execution/` 创建唯一目录。历史目录保留原格式。`execution_metadata.json` 包含相对任务的参考轨迹路径、时间范围、完成状态、步长、模型和接触边界。执行与分析的公开入口是 `/usr/bin/python3 -m v2.scripts.isaac_execute --task <任务目录>`；追加 `--gui` 启动 GUI，关闭窗口后分析数据。

## states.csv：实际状态和控制命令

每行是一个物理状态采样，均由 Isaac 状态接口读取。`sim_time_s` 为物理时间（s），`trajectory_time_s` 为同一行的参考查询时间（s）。底盘 `x_actual,y_actual` 是 world 位置（m），`yaw_actual` 是 world 航向（rad）；`vx_world_actual,vy_world_actual` 是 world 线速度（m/s），`yaw_velocity_actual` 是 world Z 角速度（rad/s）。`v_cmd,omega_cmd` 是车体纵向和角速度命令（m/s、rad/s）。`wheel_left_cmd,wheel_right_cmd` 是上一物理步所施加的左右轮角速度目标，`wheel_left_actual,wheel_right_actual` 是读回角速度（均 rad/s）。`joint1_actual` 至 `joint6_actual` 是关节角（rad），`joint1_vactual` 至 `joint6_vactual` 是 Isaac API 速度（rad/s）。`link6_x/y/z` 是 Isaac Link6 原点的 world 坐标（m），`link6_qw/qx/qy/qz` 为其 world 姿态四元数（w,x,y,z）。参考数据只保存在规划 CSV；不重复存储到 states。

## derived.csv：对齐和派生量

轨迹按 `trajectory_time_s` 对原始非等间隔采样做分段线性插值，yaw 插值前 unwrap；其余角度保持原值。实际 yaw 与参考 yaw 的差归一化到 [-π,π]。每一维使用 `world_car_x,world_car_y,world_car_theta,joint1..joint6` 后缀：`position_error_`、`velocity_error_`、`actual_acceleration_`、`acceleration_error_`、`actual_jerk_`；误差均为实际减参考。首两维以 m、m/s、m/s²、m/s³ 表示，其余维度分别以 rad、rad/s、rad/s²、rad/s³ 表示。`lateral_velocity` 为 m/s，`wheel_left_error,wheel_right_error` 为 rad/s。`link6_position_error_x/y/z` 为实际减参考（world，m），`link6_position_error_norm` 为其长度，`link6_orientation_error_rad` 为相对旋转角（rad）。

实际加速度对未滤波 Isaac API 速度按真实 `sim_time_s` 使用 `numpy.gradient` 一阶差分，内部点采用中心差分，端点一阶差分；jerk 再以相同方法对加速度差分。它们是仿真反馈的派生估计，而非直接传感器测量。参考加速度和 jerk 从 P1 对应原字段插值；参考 Link6 用 URDF/KDL 计算相同 Link6 坐标系，z 加本实验初始化高度 0.034224 m。实际底盘的 roll、pitch 和高度变化体现在实际 Link6 姿态，不人为覆盖。该高度是当前实验视觉/物理平面基准，不是仓储地图的正式高度标定。

## 2026-11：命名、绘图与多目标事件

正式执行目录使用时间戳、`P1-recommended` 或 `MULTI` 和步频命名；同名追加递增数字。默认不生成 figures；可用 `python3 -m v2.scripts.evaluate_execution <任务> <执行目录> --figures` 单独绘图。若多目标 `result.json` 保存 `task.events[].motion_end_s`，`metrics.json` 的 `tracking.goal_events` 记录目标编号、参考事件时刻、最近物理样本时刻及时间偏差、对应 Link6 位置和姿态轨迹跟踪误差。它不声称为实际末端相对任务位姿的目标达成误差。

## 2026-11：派生数据按需重建

新物理执行默认保存原始 120 Hz `states.csv`、配置和 `metrics.json`，不永久生成 `derived.csv`；原始数据与规划参考、机器人 URDF、有效配置及对应版本的评价代码共同构成重建输入。需要派生数据时，执行 `/usr/bin/python3 -m v2.scripts.evaluate_execution <任务目录> <执行目录> --save-derived`；若 derived 已存在会拒绝覆盖。重建会重新计算 metrics，已有 metrics 仅核对、不覆盖；若不一致则报错并保留原文件。真实多目标样本已验证逐字节重建 derived 和指标一致。旧实验不变。
