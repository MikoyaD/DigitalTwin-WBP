# V2-PRP 项目交接（2026-11-22）

## 接手时先看什么

在本机打开 `/home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit`。先看工作区 `/home/robot/digitaltwin/AGENTS.md` 和正式 `项目报告.md` 的最新维护日志，再看本文、`v2/docs/V2_RELEASES.md`、`v2/docs/V2_PRP_RUNBOOK.md`，最后读取具体任务相关源码和原始证据。工作区外主报告及 evidence 不随本仓库克隆，缺失时不要将本文概述当作原始证据。GUI 视觉、画面效果和人工交互由用户观察；AI 可核查代码、ROS 数据与日志。维护原始数据及未提交工作，修改需要当前任务授权。论文既有单目标版本与后来五目标研究版本分开，勿自动把后者宣称为论文原实验。

证据记号：下文“存档”为实际历史数据，“源码”为现有实现，“用户观察”为人工使用反馈；设计、推断与未验不冒充实测。

## 2026-11-22 零 NPZ 续跑结果（当前）

已验 attempt_03 首段复用，新第2段真实 P→R→P0→P1→原密度碰撞 PASS，耗时 17.749873 s、98035 状态；第3段六次预算中三次碰撞、一次相邻 MOVE 不支持、两次 MoveIt -2，任务 PARTIAL，第4/5段未执行。两次本轮目录新 NPZ 数量均为零；R 与 P0 参数保存在 `.arrays`，高密度 `.npy` 临时数组供碰撞和拼接，P0/P1 CSV 紧凑保存，新组分解析事件保存在 JSON。第2段的冻结几何和 P0 时间律已从最终结果重读验证。完整五段拼接、任务 CSV 和完成后密集数组清理尚未实测；部分任务保留 `.dense` 供续跑和诊断。报告：`research_management/evidence/V2_PRP_ZERO_NPZ_FIVE_RESUME_20261122/REPORT.md`。

## 研究目标与现状

对象是差速移动底盘加六自由度机械臂的**统一全身**规划，九维 `q=[world x, world y, yaw, joint1…joint6]`，不是先底盘后机械臂两套独立规划。底盘非完整约束 `-sin(theta)*xdot+cos(theta)*ydot=0`；前进、倒车与固定 XY 的 ADJUST 是实际运动模式。目标是从 MoveIt 规划、几何重建和时间参数化获得可验收参考轨迹，探索数字孪生展示；此为研究/仿真阶段，尚非实机或差速动力学闭环。

## 环境与代码地图

本机 Ubuntu 20.04.6、ROS Noetic、Gazebo 11，ROS1 MoveIt/OMPL；系统 Python `/usr/bin/python3`，依赖 NumPy、SciPy、CVXPY、ECOS/CLARABEL、PyYAML、ROS 消息及 MoveIt。其具体版本依现场环境，不能假定新机一致。包在 `zzs_ws/src/cr5_car_moveit`；`source /opt/ros/noetic/setup.bash` 后 source 该工作区 `devel/setup.bash --extend`。当前规划环境 11411，联合 Gazebo 环境独立 11511；不得关闭别人的节点。旧的显式非 YAML 展示环境仍含 `/home/robot/ws_moveit` 等历史路径，换机需核实；无参数展示菜单与 YAML 走包相对工作区及可选 overlay。模型来自相邻 `robot_description`，仓储 Gazebo world 来自 `aws_robomaker_world`，固定场景 `map/warehouse_cropped.bt`；小型 `.bt` 在仓库，mesh/USD/大型环境由各机保留。规划 PlanningScene 与 Gazebo world 是两个对象，不能因文件名一致就认定一致。

|用途|真实入口与数据|
|---|---|
|正式规划|`v2/scripts/plan_v2.sh --config v2/configs/global.yaml`；`prp_global.py` 分发；`prp_multi.py` 多目标、续算/重试；`stage_v2.py`、`prp_from_p.py`、`prp_from_r.py` 分阶段。`--from-p`、`--from-r` 保留。|
|P 与目标|`v2/adapters/capture_plan.py`、`moveit_input.py`、`v2/scripts/joint_ik.py`；MoveIt RRTstar 与 post-TOTG 父状态，联合 IK、P 和 P1 端点各自按 Link6 FK 验证。|
|R|`v2/geometry/signed_reconstruction.py`、`signed_frozen.py`、`signed_recovery.py`；保存几何系数，P0/P1 消费冻结几何。|
|P0/P1|`v2/scripts/g0_signed_ac.py`、`v2/timing/g0_ac_a.py`、`g0_ac_c.py`、`nested_time.py`；`v2/scripts/g1_general.py`、`v2/timing/g1_general.py`、`g1_signed_output.py`。|
|验收与输出|几何复核及事件感知时间数值检查，`g1_signed_output.py` 数值输出；最终完整机器人 PlanningScene **离散**碰撞与 FK，`prp_compact.py` 输出管理。|
|显示|`view_v2.sh`、`view_entry.py`、`v2/adapters/trajectory_viewer.py`、`run_discovery.py`；联合模式 `gazebo_interactive.py`→`gazebo_csv_replay.py`，launch 和 `replay_rviz.py`/`replay_scene_loader.py`。|
|画图|`v2/scripts/plot_trajectory.py` 接标准 CSV；`v2/outputs/plots/plot_index.csv` 索引本机实际结果，不提交完整输出。|

实际调用、手动三终端和环境限制详见运行手册。`global.yaml` 当前为五目标配置；改变 single/multi、input stage、目标须核对文件，不假设另一次实验的配置仍生效。

## 当前 P→R→P0→P1 数学和合同

- **P**：MoveIt/OMPL 产生全身九维 post-TOTG 状态，记录 `parent_path.json`，参考路径不是物理执行。单目标曾将九维 JointConstraint 收紧为 0.0005，仍必须对实际 Link6 做 0.01 m / 0.01 rad 后验；不能以 IK 解或 MoveIt 成功替代。
- **R**：从真实 P 拟合航向与六关节 profile，并求 MOVE 非负 `rho` 五次 B 样条：`[x_s,y_s]=direction*rho*[cos(theta),sin(theta)]`，ADJUST 固定 XY。原 XY 参考管每分量 0.1 m、航向 π/2、机械臂每分量 0.1 rad，原端点、关节界、非完整、接缝与方向需独立验收；内部优化边界不等同物理合同。默认 22 个 rho 控制点，特定多目标流程在固定 profile QP infeasible 时可同 P 试 36；仍 infeasible 时有条件触发 theta–rho 联合恢复。恢复实现目前仅支持 ADJUST→MOVE→ADJUST，见 `signed_recovery.solve`；单目标的触发选项与多目标不完全相同，不保证任意 chart 成功。不得放宽管或碰撞掩盖失败。
- **P0**：从 H/R 独立 H-only 冷启动统一 A，五次 smoothstep 密度，再从 A/smooth 优化嵌套 C：`d_C=d_A exp(sum beta_k*64*tau^3*(1-tau)^3)`，局部支持端及一、二阶导数为零；`beta=0` 保留 A。时律需要 A 节点与 `density_x`、`active_intervals`、`density_beta` 完整重建，不能只用总时间或兼容节点字段。实际使用安全时间修正及独立密集事件检查；结果不是全局最优证明。
- **P1**：在完整 P0 时律上以相对密度 λ 作 SLSQP 时间、平顺和可选时间恢复；源码使用 λ `[0.25,4]`，候选筛选用 jerk 5 参考，恢复分支也在其内部使用采样 jerk 5 作为搜索 guard；它不是最终统一物理 jerk 硬合同。搜索停止或安全放大要如实记录，候选仍须数值/碰撞后验。研究中的预算平顺方案未成为默认。
- **实际合同** `V2_NINE_DOF_VA_ONLY`：每个九维分量 `|v|≤0.5`、`|a|≤1.0`，XY 使用米制，其余弧度制；不等价于平移合速度或轮速也 ≤0.5。jerk 仅软评价，分别报告各维单位；不把 m/s³ 与 rad/s³ 直接相加。
- 历史单段 P1 NPZ 字段参见 `g1_signed_output.py`：`t,u,q,velocity,acceleration,jerk,chart,direction,nodes,lambda_values`；CSV 为 `time_s` 和九维 `q_…` 等标准列，展示 CSV 是源行紧凑采样，非高密度验收替代。数据采用 world/car_link；其他 joint 初态及模型必须匹配。完整任务 NPZ 字段与单段不同，不可假设当前 CSV 绘图器或 Gazebo 协调器能读取。

## 研究演进与有用的失败

- 旧时间律对 θ 加速度窄峰曾漏检（约 2.923629 rad/s²）；事件节点与时间加密后重算，不能仅凭训练采样宣称合格。详见 `H_SIGNED_FIVE_GOAL_FINISH_20261115/`。
- 原 G0 固定 H 第 4/5 段 244.795885/258.930877 s；H-only 独立 G0 平顺候选 102.664488/46.638851 s，θ jerk 峰 20.039927/6.697200 rad/s³。局部细化能显著改善第 4 段但不能直接推广所有几何；独立研究还显示 jerk 与时间权衡。见 `G0_INDEPENDENT_SEG4_SEG5_20261116/`、`G0_SEG4_CHART3_OPT_20261116/`、`G0_SEG5_LOCAL_KNOT_TEST_20261116/`。
- 五段统一 A/B 后发现非嵌套 smoothstep 增点改变曲线，第三段 fast 狭窄 jerk 峰主要由时间密度二阶导数项放大；C 嵌套对比同预算 A-continue，在部分段有效、另一些段恶化，所有所述 SLSQP 到迭代上限。见 `G0_NESTED_DENSITY_PROTOTYPE_20261116/`，不要重述成全段均改善或求得最优。
- 新 P 原终点曾产生 Link6 0.045344 m/0.137178 rad 偏差；收紧规划容差和实际 FK 后验，新 P 的实验 0.000816 m/0.001020 rad。固定旧 P 的 22 ρ 不可行而 36 可行；见 `V2_PRP_NEW_P_FEASIBILITY_20261118/`。
- 多目标第 2 段同一真实 P，固定 profile 的 22/36 最小参考管 0.136235/0.123390 m，均超过 0.1 m；联合 theta–rho 后精确闭合得到 0.024082 m 和单段 16.558872 s、97938 状态离散碰撞 PASS。其适用结构有限，见 `V2_PRP_SEG2_SIGNED_RECOVERY_20261119/`。
- P1 第五段预算平顺研究 37.852363 s、θ 峰 5.293387 rad/s³；原推荐 38.925123 s、4.971606 rad/s³，前者快但 θ 峰变差，且新状态碰撞未查，不能直接替换正式推荐。见 `G1_SEG5_BUDGETED_SMOOTH_20261117/`。

## 代表性成果与数据定位

单目标新 P→36 ρ Signed R 的研究候选 34.165601 s，Link6 FK 0.000816 m/0.001020 rad，104263 状态离散碰撞 PASS；36 当时为显式固定 P 的试验，不能冒充当时 YAML 默认端到端成功。第 2 段恢复单目标正式输出在 `v2/outputs/PRP-SEG2-SCP-20261119/result/PRP/P1/recommended/`。单目标全部 30 项与源轨迹、时长、状态看本地 `v2/outputs/plots/plot_index.csv`，不要把 30 项视为独立统计重复。

存档完整多目标索引有：两目标 40.253856 s、旧前向五目标 225.725087 s、当前有符号五目标 193.672418 s。索引中实际是两目标，不能凭要求补造三目标。新任务 `PRP-MULTI-20261119-020714-832492` 五段新 P，P1 分别 25.215802、16.741002、38.018116、55.463931、53.233567 s，五处各驻留 1 s；353744 行。五段 FK、九维数值和 737028 个机器人状态离散碰撞通过，拼接新增 12 状态通过；接缝 q 最大 4.45e-16、v/a 0。最大九维分量 v/a 0.499003/0.839921；θ jerk 峰 6.722748 rad/s³，5 为软参考。第 3 段首次规划失败后同轮重试并用同轮前两段检查点，非旧轨迹替代。规划/验收墙钟约 881.090 s。参考任务不等于实际执行；未重新对整体做连续碰撞认证。

输出目录 74.809 MiB，正式退出码 1，`OUTPUT_BUDGET_EXCEEDED`，因此数值/离散碰撞 PASS 与整体容量目标 FAIL 必须并列。高密度 `task_trajectory.npz` 不被现有单段 CSV 绘图/回放直接支持；不为格式方便丢弃其唯一数值事实。存档图 30 条单目标、3 条完整多目标中 2 条 CSV 多目标；总 32 条、64 PNG，最新整体 NPZ 未绘制。图像及 `plot_index.csv` 在本地 `v2/outputs/plots/`，不随 Git 同步。

## 2026-11-22 新五目标实验现状

实际使用 Git `bc0f65399d1ee2f73faba615cdcbcc5e86edbd25` 与 `global.yaml`，生成 `v2/outputs/prp_yaml/V2-20261122-185203-MULTI-IKPRP-RRTstar/`。首目标六次新 P 各完成 IK、R（22 ρ）、P0、P1 与末端 FK 数值，但全部在碰撞首批 60 s 预算返回 UNKNOWN，分别检查 36486、35115、36746、36690、36741、36714 状态，未报告接触；第 2–5 目标未运行。正式状态 PARTIAL、退出 1，未生成 `task/trajectory.csv`、未绘图；目录实占 127.285 MiB，超 30 MiB。不得把 UNKNOWN 写成碰撞 PASS 或 FAIL。历史 193.672418 s 完整数值及离散碰撞基线继续有效；两次不构成完整任务时间对照。详见仓库外 `research_management/evidence/V2_PRP_FIVE_CSV_ENDTOEND_20261122/`。

当前 PRP YAML 输出命名 `V2-YYYYMMDD-HHMMSS-SINGLE|MULTI-IKPRP|PRP-规划器`；多目标组分 `segments/segment_XX/attempt_XX/result/`，完整时直接从本轮拼接数据生成 `task/trajectory.csv`。此次未到该步骤，完整新任务 CSV 与其显示/绘图仍未实测。用户已将 NPZ 列为废弃交换格式；本轮新 P 计算链已改为 NPY 临时包，历史 NPZ 只读，不从历史 NPZ 生成展示轨迹。扫描现有 CSV，正式推荐/fast 保留；本轮 12 条只是首段候选（碰撞未知或未进行），不误称完整任务。

## RViz / Gazebo 展示

`./v2/scripts/view_v2.sh` 无参数选择 RViz-only 或 Gazebo+RViz。RViz-only 发布参考到 `/v2/display_trajectory` 的 V2_G；Trajectory_TOTG 接 `/move_group/display_planned_path`，都是参考动画，不驱动机器人。Gazebo 模式支持标准单段及完整多目标 CSV；用户对旧五目标 CSV 已完成 225.725087 s / 5638 状态回放（`v2/outputs/manual-replay-uppnr9ck/state.csv`，退出码 0）。不读取整体 NPZ；11511 联合 launch 启一个由 `v2_g.rviz` 派生的 RViz、固定 `/octomap_point_cloud_centers`，独立 OctoMap 默认关闭，不改变 PlanningScene。底盘用 `/gazebo/set_model_state`，手臂用 `FollowJointTrajectory`，CSV 首态先对齐，仿真 `/clock` 驱动；RViz RobotModel 来源为 Gazebo 的 TF 与 `joint_states`，不再独立播放参考。手动 A/B/C 命令见运行手册。

存档 71.430798 s 单段实验底盘和六轴实际运动；2026-11-22 菜单选同一 CSV 又真实完成 71.430798 s、1793 条反馈记录，协调器退出码 0，记录在本机 `v2/outputs/manual-replay-6mzkg8mu/state.csv`；用户报告手动回放两个窗口运动一致，属于使用观察。状态记录对参考拟合 TF XY 约滞后 30 ms、拟合后 RMSE 1.031 mm，六关节拟合 0–2 ms；Gazebo link 采样无源时间戳，不能由此推出 GUI 渲染延迟或准确的因果方向。该混合位置/运动学回放不是底盘动力学闭环、实机验证或连续碰撞。无需为了当前日常展示强行重构 TF/PID。

## Isaac 与资源

本机 `Extensions/Yourextension/yourextension_python/` 有旧 Isaac UI/场景扩展与 USD；同级 `zzs_ws/src/isaac_moveit_cr5` 是独立 ROS 包。历史定向调查发现旧扩展具有场景装配、ROS1 UI 目标发布及手动底盘位姿记录，未发现完整 V2 九维轨迹接收器；USD Action Graph 未解析，不能断言没有其他能力，也不能写成已完成对接。Gazebo 的统一物理时间、九维状态与反馈分层可供设计借鉴，但 Isaac 实际接口须另查。资产不日常 Git/rsync，同机保存；修改单个资产再单独传输。

## 状态与后续优先级

|能力|状态与影响|
|---|---|
|单目标与五目标|已实现且有真实数值/离散碰撞存档；须逐任务验证场景、FK，不推广为全路径连续碰撞/真机。|
|R 恢复与 P0/P1|已实现，恢复限特定 chart；P1 jerk 与搜索边界仍有研究空间，原结果非全局最优。|
|五目标存储|历史完整任务 74.809 MiB 超 30 MiB；2026-11-22 首段六次部分结果 127.285 MiB，完整 CSV 体积未知。|
|Gazebo+RViz|单段 CSV 与旧完整五目标 CSV 已有实际回放证据；GUI 渲染延迟未测，整体 NPZ 不支持。|
|Isaac/真实机器人|旧 Isaac 基础待调查，V2 九维对接未验；实机闭环未开展。|
|跨机与历史资料|源码 Git；模型、场景、大量数据、本机证据和主报告另行保留，不能自动覆盖另一台。|

**后续唯一优先任务**：对 2026-11-22 首段保留候选研究固定场景、完整状态和中点覆盖的碰撞分批，使每次查询能够在原 60 s 时限内完成；明确覆盖与结果传递，不放宽碰撞、不重新抽 P。该方案尚未实施。若用户更改研究目标，服从新任务；论文结论仍需对照原论文和审稿意见、使用可复核数据，不能从展示与单次案例推断统计成功率。

## 2026-11-22：输出与回放更新（取代上文单段限定）

用户已对旧前向完整五目标 `V2-20261113-FIVE-GOAL-FINAL-RECOMMENDED/task/trajectory.csv` 实际回放，终端记录 225.725087 s、5638 状态及退出码 0；现场 `v2/outputs/manual-replay-uppnr9ck/state.csv` 有 5638 状态。该证据仅属于该 CSV 的 Gazebo 混合回放，不代表现有整体 NPZ 被支持或规划碰撞合同发生变化。

用户确定 NPZ 为废弃格式：展示扫描不读取历史 NPZ，不从 NPZ 转换 CSV；新运行的数值与碰撞链已使用数组包并实际完成第2段，完整任务及所有入口尚需实测。当前正式 YAML 新结果命名 `V2-YYYYMMDD-HHMMSS-SINGLE|MULTI-IKPRP|PRP-实际规划器`，P/R 续算为 PRP、新 Link6 IK 为 IKPRP；旧非 PRP 实验和历史路径保留。推荐/fast CSV 在 `result/PRP/P1/{recommended,fast}/trajectory.csv`，多目标按 `segments/segment_XX/attempt_XX/result/…`，整体为 `task/trajectory.csv`。整体 CSV 由本轮任务拼接数组直接写出，保留状态与驻留；不删除 CSV 以迎合 30 MiB，超额必须报告。2026-11-22 新五目标实际运行在首段碰撞查询时限失败，完整任务 CSV 和体积未得到；不能把旧结果冒充新代码成果。

两种展示模式复用同一 CSV 扫描与三类名称：单目标、多目标、多目标组分。历史父路径缺失、模型引用错误或没有 CSV 的实验不自动伪造可播放输入；碰撞失败轨迹在 Gazebo 前要求用户确认。最新实际扫描、用户日志与测试见仓库外 `research_management/evidence/V2_OUTPUT_SCAN_STANDARD_20261122/`。
