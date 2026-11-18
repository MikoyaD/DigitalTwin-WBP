# V2 G 轨迹实验手册（阶段 II-B）

适用范围：ROS Noetic、MoveIt `whole` 九维离线规划、静态仓储 OctoMap、V2 Geometry-First G 时间律。此流程只生成规划与参考轨迹，不向控制器发送运动命令；`PASS` 表示数值验收和当前 PlanningScene 的**离散**全机器人碰撞检查通过，不表示实机跟踪或连续碰撞保证。阶段 III 的 SOCP/SCP-QP 尚未实现。

## 1. 从干净终端运行

在 `/home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit` 执行：

```bash
./v2/scripts/run_experiment.sh --stage all --headless
```

脚本自身使用干净 Bash，依次加载 `/opt/ros/noetic/setup.bash`、`/home/robot/ws_moveit/devel/setup.bash`、当前工作空间 `devel/setup.bash`；检查 `ROS_DISTRO=noetic`，并将结果写入 `environment.json`。它使用配置中的专用 Master 地址 `http://127.0.0.1:11321`，缺失时启动本轮自己的 `roscore` 和无界面 `move_group`；不启动 RViz、gzclient、Gazebo 或旧 QP/采集节点。静态障碍由仓储 `.bt` 地图经 `octomap_server` 装入 PlanningScene，不依赖 Gazebo 传感器。若端口已被非本轮进程占用，脚本拒绝接管；在新的 V2 配置中改用空闲端口。

流程为 `preflight → planning → g → validate`。运行开始打印 `RUN_DIR=...`，结尾打印 `STATUS=PASS/PARTIAL/...`。正式可用结果必须以该运行的 `result_summary.md`、`validation_index.json` 和 `manifest.json` 一同判读。首次成功冷启动的实例：`v2/outputs/runs/run_20261022T092444Z_adb7f2f5/`。

先查看将执行的步骤而不启动 ROS：

```bash
./v2/scripts/run_experiment.sh --stage all --headless --dry-run
./v2/scripts/run_experiment.sh --help
```

`--dry-run` 自己创建标记为 `DRY_RUN` 的新目录，不产生规划或验收结果。

## 2. 分阶段运行与复用父路径

在同一专用 Master 保持运行期间可分步执行：

```bash
./v2/scripts/run_experiment.sh --stage preflight --headless
./v2/scripts/run_experiment.sh --stage planning --input v2/outputs/runs/<preflight_run_id>
./v2/scripts/run_experiment.sh --stage g --input v2/outputs/runs/<planning_run_id>
./v2/scripts/run_experiment.sh --stage validate --input v2/outputs/runs/<g_run_id>
```

将尖括号占位符替换成上一步打印的真实目录名；每一步创建新的 `run_id` 目录，`parent_run_id` 保留继承关系。`g` 读取冻结的 `planning_*/capture/g_input.json`，不会重新调用 RRTstar；同一规划 run 可以多次独立运行 `g`。`validate` 校验保存的轨迹、数组和哈希，然后用与父路径一致的真实场景检查碰撞。若场景或服务不可用，结果为 `UNKNOWN`/失败，不能当作 `PASS`。如要复用已有 ID，可用 `--run-id <existing_id>`；系统仍创建带父关系的新目录，不覆盖旧文件。改变配置或模型后不可静默续跑，需重新进行 preflight/planning。

阶段职责：

| 阶段 | 必备条件 | 主要产物 |
| --- | --- | --- |
| `preflight` | Noetic、专用 Master、`move_group`、TF、完整模型、非空 OctoMap | `preflight*/scene_probe.json`、`scene_snapshot.bin`、URDF/SRDF 快照 |
| `planning` | 与 preflight 相同的模型和场景 | `planning_*/request.json`、`capture/{capture_result.json,g_input.json,parent_path.json}`、`planning_index.json` |
| `g` | 冻结父路径、原模型/配置哈希 | `g_*/{result.json,trajectory.csv,phase_arrays.npz}`、`g_index.json`；此时碰撞状态仍为 `UNKNOWN` |
| `validate` | 规划时场景仍可访问且一致 | `validation_*.json`、`validation_index.json`、`validation.json` |

`all` 按上述顺序执行，必要条件失败立即停在出错阶段。所有阶段都会保存 `commands.log`、环境、日志、ROS 节点/服务清单、失败码、清单和结果摘要。清单含文件 SHA-256、生成时间、源配置及受保护文件哈希。不要用单独的 `g` 数值 `PASS` 代替最终 `validate` 结果。

## 3. 任务与场景如何核对

本轮可运行配置是 `v2/configs/experiment_ii_b.yaml`，默认规划器 `RRTstar`，组 `whole`，5 秒前端规划预算，任务为既有 `map/waypoints.json` 的 5→1、1→2。5 秒是**预算**；实际规划耗时在各 `capture_result.json` 的 `reported_planning_time_s` 与 `front_wall_s`。任务/规划器调整时，在 `v2/configs/` 新建带新名字的配置并传 `--config <path>`；不要改既有运行引用的配置，也不要改 V1 路点文件。`planning_request.py` 依此构造显式九维起点和目标区域，完整请求、响应及父路径哈希随运行保存。随机种子若未获 MoveIt 后端可控且可核验的设置，不得宣称固定。

检查障碍是否确实装入：打开 `preflight_recovered/scene_probe.json`（或 `preflight/scene_probe.json`），确认 `status=PASS`、`octomap_id=OcTree`、`octomap_data_bytes>0`、`scene_sha256`、`urdf_sha256`、`world_to_odom` 和 ACM 记录。对应 `scene_snapshot.bin` 保存了场景消息；后续 `validation_*.json` 必须具有相同 `scene_sha256`、`collision.status=PASS` 与 `checked_states>0`。这比仅检查地图文件存在更严格。

当前 G 版用五次端点静止时间律，并按配置中的 `research_limit` 验收；轮速/轮加速度、自旋 jerk 限值是研究设定，不是硬件能力证明。非完整约束、速度/加速度、接缝连续性与分组 jerk 见 `g_*/result.json` 和 `validation_*.json`。若输入出现底盘原地转动与机械臂同时变化，V2 显式采用 `SPIN_THEN_ARM` 分段，`decisions` 中有记录，最终状态必须重新碰撞验收。

## 4. 查看日志、结果与安全停止

`RUN_DIR` 中 `result_summary.md` 看总状态；`commands.log` 查阶段、命令、退出码；`launch.log` 查 Master/MoveIt/场景；`planning.log` 查规划请求；`optimization.log` 查 G 与碰撞验收。实时追踪：

```bash
tail -f v2/outputs/runs/<run_id>/launch.log
tail -f v2/outputs/runs/<run_id>/optimization.log
```

CSV 在 `v2/outputs/runs/<run_id>/g_<task_id>/trajectory.csv`；同目录 `result.json` 是时长、路长、jerk、后端耗时和策略决策，`validation_<task_id>.json` 是最后的独立验收。失败时保留整个 run 目录，不删除或覆盖。

结束专用 ROS 进程时，对**启动它的** run ID 执行：

```bash
./v2/scripts/stop_experiment.sh --run-id <owner_run_id>
```

脚本核对 PID、进程启动时间和 `V2_RUN_ID` 后仅向本轮进程组发 SIGINT，并生成 `stop_*.json`；不会清理用户既有 ROS/Gazebo。分阶段运行时 owner 通常是 `preflight` 的 run ID。`g`/`validate` 子 run 不拥有 Master。若仍未退出，按 `stop_*.json` 和系统进程核查，勿用广泛的 `pkill`。

## 5. 扩展与常见故障

故障现象、实际修复及证据见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。新增 SOCP/SCP-QP 时，复用冻结父路径、`run_id`/parent hash/场景快照、阶段日志和独立 `validate` 合同，增加独立阶段及输出目录；保持 G 基线和原始 V1 不变。不得把尚未实现的优化阶段写入本轮 `PASS`。

## 6. 播放已有 G 轨迹（不规划、不重新生成）

以下命令均从 `/home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit` 执行，使用已验收的 `run_20261022T092444Z_adb7f2f5`。打开三个终端；全部命令使用干净 Bash 和现有 `env_noetic.sh`，不触发交互式 `~/.bashrc` ROS 版本菜单。

终端 A：如果专用 ROS Master 和机器人模型尚未运行，仅做显示环境预检（不调用规划）：

```bash
./v2/scripts/run_experiment.sh --stage preflight --headless
```

记下打印的 `RUN_DIR` 中 `run_id`，结束时用它停止本轮节点。终端 B：打开从现有 MoveIt RViz 配置复制的 V2 专用配置；其中 `V2_G` 只订阅 `/v2/display_trajectory`，旧 TOTG 显示默认关闭：

```bash
bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; exec rviz -d v2/rviz/v2_g.rviz'
```

终端 C：任选一条**已保存**的九维 G 轨迹。`--repeat 0` 循环重发，按 Ctrl+C 停止；`--repeat 1` 只发一次，进程在原始轨迹时长结束后退出。

```bash
bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; exec python3 -B -m v2.adapters.display_g --task 5to1 --repeat 0'
bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; exec python3 -B -m v2.adapters.display_g --task 1to2 --repeat 0'
```

切换任务前先停止当前发布命令。发布器会等待 RViz 订阅；打印 `PUBLISHED ... connections=1` 才说明 ROS 话题实际发出，但仍应观察 RViz。`5to1` 为约 10.91 s 的联合平移；`1to2` 为约 49.89 s，阶段依次 `SPIN–ARM_ONLY–TRANSLATE–SPIN–ARM_ONLY`。其首段 0–11.53 s、末段约 45.10–48.24 s 是原地自旋，可在 RViz `V2_G` 轨迹滑块上复看。绿色 `V2_G` 整机模型应移动和转向；若静态交互模型遮挡，可在 Displays 中取消 `MotionPlanning` 与 `RobotModel`，保留 `V2_G`。`V2_G` 的 `State Display Time` 预设为 `1x`；如要更快/更慢，可在 RViz 属性中调整倍率。消息中的 `time_from_start` 始终使用原 CSV 实际时间，不受视觉倍率修改。

不接 Master 也能先核验消息构造：

```bash
bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; python3 -B -m v2.adapters.display_g --task 1to2 --dry-run'
```

该命令只输出 `MESSAGE_CONSTRUCTION_PASS; ROS_PUBLISH_NOT_ATTEMPTED`，不等于 RViz 可见。显示脚本既不发布 `/move_group/display_planned_path`，也不调用 `execute`、`cmd_vel` 或 Gazebo。结束终端 A 的节点时执行 `./v2/scripts/stop_experiment.sh --run-id <终端A的run_id>`；只会停止该 run 自己启动的进程。

## 7. 通用轨迹播放器（推荐用于后续 G / T / J 展示）

统一目录 `v2/trajectories/<唯一名称>/metadata.json` 只保存引用，不复制或替代 `outputs/runs` 证据。已注册 `warehouse_5to1` 与 `warehouse_1to2`。从 ROS 包根目录执行以下命令。

**启动环境和 RViz，各一次即可。** 如果已有 V2 专用 Master、正确的 RobotModel 正在运行，直接复用；否则执行现有预检入口（不会规划或生成 G），记下新 `RUN_DIR` 的 owner run ID：

```bash
./v2/scripts/run_experiment.sh --stage preflight --headless
```

在另一个终端打开 RViz：

```bash
bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; exec rviz -d v2/rviz/v2_g.rviz'
```

**启动一个常驻播放器**，无需重复打开 RViz：

```bash
./v2/scripts/view_trajectory.sh
```

在 `viewer>` 提示符中逐条输入：

```text
list
play warehouse_5to1
play warehouse_1to2
replay
play 1
quit
```

编号以最近一次 `list` 的字母序结果为准，新增注册后用 `list` 刷新。错误名称、损坏或丢失文件只报错，随后仍可继续输入。每次只发一条 `DisplayTrajectory`，同一 ROS 发布器保持运行；RViz `V2_G` 的 `Interrupt Display=true` 用新动画替换当前动画，`Loop Animation=false`，需要再看时输入 `replay`。时间轴直接来自 CSV，显示倍率预设 `1x`。**最终 GUI 动画检查为 `USER_VERIFICATION_REQUIRED`**，请自行观察底盘、六关节和首尾自旋。不要同时启动旧 `display_g.py --repeat 0`；发现同话题的其他发布器时新播放器会报 `COMPETING_DISPLAY_PUBLISHER`。

**注册已有 V2 G 结果**（下列两条已执行成功，无需重复；同名会拒绝覆盖）：

```bash
./v2/scripts/view_trajectory.sh register-v2 --name warehouse_5to1 --run-dir v2/outputs/runs/run_20261022T092444Z_adb7f2f5 --task 5to1
./v2/scripts/view_trajectory.sh register-v2 --name warehouse_1to2 --run-dir v2/outputs/runs/run_20261022T092444Z_adb7f2f5 --task 1to2
./v2/scripts/view_trajectory.sh list
```

注册记录包含相对引用、SHA-256、模型/坐标系/关节顺序、原 run ID、父路径 hash 和原验收文档引用。`register-v2` 当前针对已存在的同目录 G 结果布局；未来 T/J 可先使用下述通用 CSV 注册，不需新建消息转换器。

**注册独立 CSV**。要求单位为秒、米、弧度；字段为 `time_s,x,y,theta,joint1,...,joint6`，或现有 G 的 `time_s,q_world_car/x,q_world_car/y,q_world_car/theta,q_joint1,...,q_joint6`。按表头取字段，额外列允许存在；速度、加速度不必提供，也不会在缺失时编造导数。时间必须非负且严格递增，保留原 time_from_start，不重采样。必须用 `--frame` 明确坐标系，并用 `--state-from` 指定已有注册项作为完整非九维关节状态及 URDF/SRDF 模板；坐标系或模型不一致则拒绝。

下面以实际存在的 G CSV 演示通用注册，替换 `--csv` 路径即可注册自己的文件；此入口始终标记 `VISUALIZATION_ONLY`，不会继承模板的轨迹碰撞验收：

```bash
./v2/scripts/view_trajectory.sh register-csv --name preview_csv --csv v2/outputs/runs/run_20261022T092444Z_adb7f2f5/g_archived_5_to_1_joint_base/trajectory.csv --frame world --state-from warehouse_5to1 --algorithm G
```

`--algorithm` 是来源标签，可填 `G`、`T` 或 `J`，本身不是算法实现或安全证明。非九维关节取自已核对的完整 RobotState 模板，不任意填零。模板来源支持的是保存的 MoveIt 完整响应；暂不接受没有状态/模型来源的裸 CSV。

**移除注册而保留原始数据**：先在播放器退出或停止引用该名字，然后运行：

```bash
./v2/scripts/view_trajectory.sh unregister preview_csv
```

该命令只删除对应注册目录中的 `metadata.json` 和空目录；目录若含额外文件会拒绝。原始 CSV、run、模型、日志均保留。

路径引用相对于 `metadata.json` 所在目录；引用内容的哈希在加载时重新核对。缺文件、文件移动、哈希变化和符号链接均明确报错，不自动搜索替代数据。需要迁移时保持相对目录关系，或有意识地取消注册后重新注册。注册名称按大小写不敏感查重，拒绝静默覆盖。`RUNTIME_MODEL_MISSING/MISMATCH` 时检查 Master 地址以及 `robot_description`/`robot_description_semantic` 是否为同一模型；不要为了播放而替换现有机器人模型。`NO_SUBSCRIBER` 时检查 RViz `V2_G` 是否启用并订阅 `/v2/display_trajectory`。播放器从不启动 Gazebo，也不写实际 RobotState、TF 或控制话题。

结束时在播放器输入 `quit`，关闭本轮 RViz；若是自己由 preflight 启动的节点，再用 `./v2/scripts/stop_experiment.sh --run-id <owner_run_id>` 停止。已有用户 Master 不应停止。原 `display_g.py --task ...` 命令保持兼容，使用它前先退出交互播放器。

## 8. 单目标 YAML 配置与保存后诊断

复制 `v2/configs/single_goal_template.yaml` 为自己的新配置，不修改历史配置。`tasks` 中配置唯一 `id`、`start_q`、`goal_q`，顺序固定 `[x,y,theta,joint1,...,joint6]`，单位 m/m/rad；`goal_tolerance` 对应九维正容差。底盘转换为 world_car MultiDOF RobotState，臂转换为 joint1..6 RobotState。`group_name` 必须 whole。`planner_id`、`allowed_planning_time_s`（秒，0–120，不含0）、`velocity_scaling`/`acceleration_scaling`（0–1，不含0）直接进入 MoveIt 请求。保留旧路点编号配置的兼容性。

`backend_limits` 集中配置 arm_velocity/arm_acceleration（6维 rad/s、rad/s²）、xy_component_velocity/acceleration（2维 m/s、m/s²）、angular_velocity/acceleration（标量 rad/s、rad/s²）。每项含 value/unit/kind=research_limit/source；不得超过现有 URDF/YAML 经请求 scaling 得到的后端上限。`research_limits` 保留 XY 模长、切向加速度、轮速/轮加速度和 spin_jerk，单位见模板；这些是研究设定，不是硬件能力。默认值未提高。更小的分量上限可能比 XY 模长上限先限制时间律。

`static_octomap` 是相对于 ROS 包的现有 .bt 路径（也支持绝对路径），默认 map/warehouse_cropped.bt；它实际传入 V2 headless launch 的 static_octomap 参数。切换地图请先停止自己上次启动的专用节点，再对新配置重新 preflight，不混用旧父路径。

```bash
./v2/scripts/run_experiment.sh --stage all --headless --config v2/configs/single_goal_template.yaml
```

每个新 run 保存解析后的 `config_snapshot.json`、地图 hash、完整请求/响应、模型/场景、G 输出与实际 LimitSet。G 结果保存后，独立运行 diagnose_g，新增 `g_<task>/diagnostics.json` 与 diagnostics.md；JSON 包含每阶段时间/路长/自旋/关节净变化、各限值峰值与利用率、时间占比和单位时长候选时间。原 G 求解过程未保存精确缩放迭代，实际缩放因子和确切主导约束标 UNKNOWN；候选仅由保存的峰值及统一时间缩放关系复算，不能当作求解器日志。重新诊断可用 `python3 -B -m v2.scripts.diagnose_g <g目录>`，仅在该目录尚无诊断文件时执行；已存在则拒绝覆盖。

本轮实际新 run 为 `run_20261023T110410Z_dfce1162`，task_id 为 config_single_goal。模型与 RViz 启动后，可直接播放：

```bash
bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; python3 -B -m v2.adapters.display_g --run-dir v2/outputs/runs/run_20261023T110410Z_dfce1162 --task config_single_goal --repeat 1'
```

或用现有注册工具按任意 task ID 注册新 G，然后在常驻播放器中 list/play。历史 --task 5to1、1to2 不变。

末端位姿请求可用 task.pose_goal 字段：frame_id、link_name、position 三维、orientation_xyzw 单位四元数、position_tolerance_m 球半径、orientation_tolerance_rad。请求确实生成 MoveIt PositionConstraint 和 OrientationConstraint；**目前不支持其完整 G capture/验收链**，因为已有父路径合同要求明确九维目标，capture 会以 POSE_CAPTURE_NOT_SUPPORTED 停止，不把 start_q 伪装成末端目标的 IK 结果。本轮只验收九维目标，当前请使用 goal_q。

## 9. 用户单目标 0.5 配置与自动发现（当前推荐）

`v2/configs/v2_single_goal_05.yaml` 保存用户提供的旧单目标九维起终点，task ID 为 `single_goal_05`；目标 theta 原样为 **-3.1532 rad**。速度/加速度请求 scaling 均为1.0；六臂关节、底盘 XY 分量和角速度/角加速度的研究上限均为0.5（单位沿用模板）。这些值通过当前 URDF/YAML 与 LimitSet 静态检查，等于对应 YAML 的上限，不证明硬件能力。XY模长/车轮/自旋jerk仍沿用模板 research_limits。goal_tolerance也沿用模板，**尚未证明与旧实验相同**。

起终点只能在新配置副本中修改 `tasks[0].start_q/goal_q`，顺序 `[x,y,theta,joint1,...,joint6]`；不修改旧实验或已被run引用的配置。标准命令：

```bash
./v2/scripts/run_experiment.sh --stage all --headless --config v2/configs/v2_single_goal_05.yaml
```

本轮标准端口11321已有非本轮Master，入口安全拒绝接管。为保留原配置，另存只更换master_uri为11322的 `v2_single_goal_05_isolated.yaml`；若需要使用该空闲端口，运行：

```bash
V2_MASTER_URI=http://127.0.0.1:11322 ./v2/scripts/run_experiment.sh --stage all --headless --config v2/configs/v2_single_goal_05_isolated.yaml
```

本轮隔离联调场景预检通过，5秒RRTstar规划返回TIMED_OUT(-6)，故没有G、路长、参考时长或最终碰撞PASS。失败保存于 `run_20261023T142746Z_d6fad99c`，不会被自动播放器列为可播放。未放宽预算/限值/场景。此结果不能说明任务永远不可行，也不能称为已复现旧论文实验。

**播放器现在默认自动发现，无需逐条注册实验结果。** 同一ROS模型/RViz启动一次后：

```bash
./v2/scripts/view_trajectory.sh
```

终端输入：

```text
list
refresh
play 1
play run_20261022T092444Z_adb7f2f5/archived_5_to_1_joint_base/G
replay
quit
```

名称为 `run_id/task_id/G`，不会因重复task名称选错run。list/refresh都会重新扫描 `v2/outputs/runs/` 的直接run子目录，通过manifest、validation_index和g_index的已知父链寻找数据（最多32层，不做任意目录递归）。每项检查CSV/NPZ/result、完整起始RobotState、模型/场景快照和验收哈希一致性；只自动展示完整的数值与离散碰撞PASS。列表显示保存时间、task、G变体、run、参考时长及验证状态；坏目录显示INVALID且不阻止其它轨迹使用。播放时再次核对，不复制任何实验大文件。刷新在新run保存且验收完成后即可发现，无需退出播放器。

历史手动注册G与同一CSV的自动发现项去重，旧注册名称仍可作为别名播放。手动 `register-csv` 主要用于缺少可追溯初始状态的外部CSV或特殊来源，始终保持VISUALIZATION_ONLY，不升级为碰撞PASS；独立预览与正式run即使引用同一CSV也保留其状态差别。当前自动发现支持旧G及新G1的fast/recommended，具体只读预览入口与状态含义见第16节。自动发现项以实际输出目录标识，不以预览行为改变正式验收记录。

如使用11322显示环境，启动RViz和播放器时都须指定同一地址：

```bash
V2_MASTER_URI=http://127.0.0.1:11322 bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; exec rviz -d v2/rviz/v2_g.rviz'
V2_MASTER_URI=http://127.0.0.1:11322 ./v2/scripts/view_trajectory.sh
```

本轮未再打开GUI；真实ROS消息自动发现/发布/重播已验证，最终动画为USER_VERIFICATION_REQUIRED。旧第7节的register-v2命令继续兼容，但已有有效实验run现在无需执行它。

## 10. 工作空间与规划前端点检查

当前仓储任务示例为 `v2/configs/v2_workspace_single_goal.yaml`。复制成新配置再修改工作空间，不改已存run引用的配置：

```yaml
workspace:
  frame_id: world
  min_corner: [-20.0, -20.0, -2.0]
  max_corner: [20.0, 20.0, 5.0]
default_workspace_bounds: 40.0
```

每个min必须小于对应max，必须为有限数，当前只接受world坐标系。workspace是真实MotionPlanRequest.workspace_parameters，并非机器人硬限位。PLANAR底盘实际使用XY范围；z字段保留在请求中，不会成为末端/机械臂的额外避障或位置约束。±20 m是本轮研究范围，包含用户端点及本轮实际规划参考，不是硬件行程或全仓库覆盖保证。

`default_workspace_bounds` 是未提供workspace时的立方体**全边长**。MoveIt的FixWorkspaceBounds会除以2：默认10意味着[-5,5]，40意味着[-20,20]。启动脚本实际传入launch arg，同时设置 `/move_group/default_workspace_bounds` 与**适配器实际读取的** `/move_group/planning_pipelines/ompl/default_workspace_bounds`。显式请求优先；不能靠只设置根参数而忽略管线命名空间。旧YAML不含workspace时保留10的回退行为。

本轮任务保持起点 `[0.4315,-7.7584,1.7959,0.0002,-0.0001,0,0,-0.0001,0]`、终点 `[-3.0931,6.6455,3.1291,-0.7799,0.323,-2.494,2.1701,0.78,0]`，whole/RRTstar/5秒/两种scaling=1与现有0.5限值、容差、地图不变。使用已确认空闲的独立端口：

```bash
V2_MASTER_URI=http://127.0.0.1:11324 ./v2/scripts/run_experiment.sh --stage all --headless --config v2/configs/v2_workspace_single_goal.yaml
```

新请求在规划前独立保存 `planning_<task>/capture/endpoint_checks.json`：逐维值、workspace与模型范围、具体超界维度，以及真实PlanningScene两端完整RobotState的有效性/接触信息。theta按周期变量解释，原始航向不改。INVALID_BOUNDS、START_IN_COLLISION、GOAL_IN_COLLISION或UNKNOWN会阻止这一显式工作空间任务继续规划；状态服务不能可靠连接时绝不伪报PASS。未碰撞的端点不代表整条路径安全，最终validate仍必需。

同目录的 `request_wire.bin` 是实际GetMotionPlan请求的ROS序列化内容；`effective_ompl_bounds.json` 来自独立C++探针：使用运行时RobotModel、规划管线参数和OMPL库建立上下文，直接读取状态空间变量边界，**不调用solve**。探针检查显式XY边界确实进入上下文，结果嵌入capture_result。它与规划服务内正在solve的上下文是不同实例；结合适配器规则和实际服务请求核验传递，不把JSON记录本身当作生效证据。`workspace_probe.log` 保存探针日志。

本环境已编译 `v2/outputs/workspace_tools/workspace_probe`，不需每次重编。若移动代码副本后工具不存在，按现有依赖编译（不安装新软件）：

```bash
mkdir -p v2/outputs/workspace_tools
bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; g++ -std=c++14 v2/adapters/workspace_probe.cpp -o v2/outputs/workspace_tools/workspace_probe $(pkg-config --cflags --libs moveit_planners_ompl moveit_ros_planning)'
```

本轮run `run_20261023T165050Z_548ef707` 全链PASS；新G也已被现有自动发现列表接受。实际探针显示 `whole_JointModel/JointModel`，不能因旧OMPL YAML写有ReedsShepp就声称加载的是OMPL ReedsSheppStateSpace；本轮没有更换空间/算法。请求端点和真实返回的端点应按原有goal_tolerance区分，不能声称目标区域规划精确命中指定数值。

端口存在历史Master时不接管、不全局清理。退出时只用 `./v2/scripts/stop_experiment.sh --run-id <本轮owner_run_id>` 停止该run的自有进程。其它端口及未知归属节点保持不动。

## 11. 新 P/H/G 结构入口（2026-10-25更新，北京时间）

用户已确认V1/V2均使用 **post-TOTG 九维父路径**，当前不切换规划管线。P、H成功并不等于G成功；G必须通过数值及真实场景离散碰撞验收。2026-10-25已用固定P/H成功生成下述新格式G，并通过独立validate和实际ROS显示消息发布。上文历史 `run_experiment.sh` 与旧 `outputs/runs` 入口继续保留。

所有新入口都从 ROS 包根目录直接执行，不需手动 `source`/`export`，固定使用 V2 专用本机端口 `11411`；环境进程状态与日志在 `/tmp/cr5_v2_runtime_<uid>/`，不存入规划结果。若端口属于未知 Master，脚本拒绝接管。只停止本工具拥有的进程：

```bash
./v2/scripts/start_v2.sh --config v2/configs/v2_workspace_single_goal.yaml
./v2/scripts/status_v2.sh
./v2/scripts/stop_v2.sh
```

规划接口：

```bash
./v2/scripts/plan_v2.sh --stage planning --config v2/configs/v2_workspace_single_goal.yaml --task single_goal_05
./v2/scripts/plan_v2.sh --stage geometry --input v2/outputs/V2-<北京时间>-single_goal_05-RRTstar
./v2/scripts/plan_v2.sh --stage g --input v2/outputs/V2-<北京时间>-single_goal_05-RRTstar
./v2/scripts/plan_v2.sh --stage validate --input v2/outputs/V2-<北京时间>-single_goal_05-RRTstar
./v2/scripts/plan_v2.sh --stage all --config v2/configs/v2_workspace_single_goal.yaml --task single_goal_05
```

`--input latest` 只解析**具备所需前置成功文件**的最近新格式目录；启动后先核对打印的 `LATEST_RESOLVED`，尤其是多任务实验。`geometry` 读取已保存 P 并生成 `geometry.json`；`g` 只读取同目录冻结 H，不重规划、不重新选择几何。`t`/`j` 明确返回 `NOT_IMPLEMENTED`。同一 H 的不同 G 参数实验使用 `--variant NAME`，既有成功目录不可覆盖。当前候选几何的选择过程仍使用旧 G 限值估时，这属于旧 H 决策规则，**不是独立 SOCP 时间优化**。

成功新目录仅含 `parent_path.csv/json`、`geometry.json` 与 G 的 `trajectory.csv/result.json/diagnostics.json/report.md`；某阶段失败时只留该阶段 `error.log`，前序成功文件不删除。G先在临时目录完整写出四个文件，再提交成功目录；NumPy数组与标量在JSON输出边界无截断转换，序列化或诊断失败不会留下半个成功目录。新播放器自动跳过失败G并兼容旧结果。

本轮真实新路径 `v2/outputs/V2-20261024-231827-single_goal_05-RRTstar/` 的 P/H 成功，G 在 `phase_000` 的 `s=0.601318359375` 出现 `<octomap>` 与 `Link6` 接触，已按合同拒绝 G PASS；详情见其 `G/error.log`。不可用放宽场景或限值消除该失败。


### 已验证的新格式轨迹：固定P/H重试G

`v2/outputs/V2-20261025-003321-single_goal_05-RRTstar/G/`：7阶段、70.215789442 s、XY路长15.328711231 m；G及独立validate均通过25,708个状态的全机器人离散碰撞检查。读取冻结P/H，不调用RRTstar或几何拟合。已成功的G再次执行`g`会拒绝覆盖，直接验证或播放即可：

```bash
./v2/scripts/plan_v2.sh --stage validate --input v2/outputs/V2-20261025-003321-single_goal_05-RRTstar
./v2/scripts/view_v2.sh
```

播放器中输入：

```text
list
play V2-20261025-003321-single_goal_05-RRTstar/G
replay
quit
```

已有RViz时使用 `./v2/scripts/view_v2.sh --no-rviz`。本轮实际验证了上述新轨迹在同一会话中的play/replay发布、九维位置和统一时间轴；**GUI动画仍由用户目视确认**。实际消息只发布 `/v2/display_trajectory`，无机器人执行。ROS Duration有纳秒量化，本轮CSV与消息最大时间差小于1.1 ns。

若检查大量状态曾触发60秒预算，不应把UNKNOWN当碰撞或PASS。当前验收器使用同一持久TCPROS连接逐个查询，无自动跳过或重试；查询数、60秒预算、5 mm/0.01 rad分辨率及半间距复核均未改变。

若H报 `GEOMETRY_CANDIDATES_EXHAUSTED/HEADING_REFINEMENT_BUDGET`，保留父路径与error.log，不手动提高预算或修改目标。已知 `V2-20261025-003230-single_goal_05-RRTstar` 在末端微小XY移动伴大角度/关节变化时触发此问题；需单独审定阶段边界和航向网格设计，不属于JSON保存故障。

## 12. 统一九维连续 H/G 快速原型（contract_version=2）

新入口只读取既有 post-TOTG 父路径，不重新调用 RRTstar；独立新目录保留同一 P，旧版 `geometry.json` 与 G 不变。从 ROS 包根目录运行：

```bash
./v2/scripts/plan_unified_v2.sh --stage h --input v2/outputs/V2-20261025-224610-single_goal_05-RRTstar
./v2/scripts/plan_unified_v2.sh --stage g --input v2/outputs/V2-20261026-214202-single_goal_05_UHG-RRTstar
./v2/scripts/plan_unified_v2.sh --stage validate --input v2/outputs/V2-20261026-214202-single_goal_05_UHG-RRTstar
```

`h` 返回新目录名；后两条命令中的目录应换成它实际打印的目录。也可用 `--stage all --input <已有P目录>` 连续运行 H/G/独立验收。如果兼容仓储场景已由其它终端在本机 `11411` 端口运行，且不希望脚本接管启动，可在 `g`、`validate` 或 `all` 命令尾部加 `--existing-scene`：入口只读查询 Master、模型、TF、场景和状态有效性，逐项对照父路径保存的身份；不匹配即拒绝。不要把这个选项用于未知或不兼容场景。G 失败只保留 `G/error.log`，不会生成综合 PASS。

冻结合同 `geometry.json` 的 `contract_version=2` 包含一个正式 MotionBlock 和三个内部 chart；chart 变化不重新设置整机 rest-to-rest。已验证的固定 P 新结果：`v2/outputs/V2-20261026-214202-single_goal_05_UHG-RRTstar/G/`。新结果的 `result.json` 与 `report.md` 分别记录数值、场景碰撞和计算耗时。严格区间认证尚未做，状态为 `NUMERICAL_CHECKED`。旧合同版本 1 仍由原入口读取和执行；不要使用旧 `plan_v2.sh --stage g` 读取版本 2 几何。

绘图不改变 CSV：

```bash
python3 -B -m v2.scripts.plot_trajectory v2/outputs/V2-20261026-214202-single_goal_05_UHG-RRTstar/G/trajectory.csv --output v2/outputs/plots_unified_fast
```

RViz 与常驻播放器沿用原入口，刷新后直接发现新结果，无需注册：

```bash
./v2/scripts/view_v2.sh
```

在播放器输入 `refresh`、`play V2-20261026-214202-single_goal_05_UHG-RRTstar/G`、`replay`、`quit`。已打开 RViz 时用 `--no-rviz`；已有播放器运行时不要另启竞争发布器。只发布 `/v2/display_trajectory`，不执行机器人。当前环境完成了 7380 点九维 JointTrajectory/MultiDOF 消息构造；RViz 动画仍需用户目视检查。

## 13. 独立九维 v/a 研究合同与配对实验

新文件 `v2/configs/v2_workspace_single_goal_9d_va_only.yaml` 不改变旧配置或默认行为。`constraint_profile.velocity` 和 `acceleration` 依次对应 `[x,y,theta,joint1,...,joint6]`；默认速度全为 0.5（前两项 m/s，其余 rad/s），加速度全为 1.0（前两项 m/s²，其余 rad/s²）。这是经用户批准的**后端研究限值**，其中加速度 1.0 高于现有 MoveIt YAML 的 0.5；不是实机硬件能力。前端仍使用现有 MoveIt 配置和请求 scaling。

手动修改时先复制新配置，再按上述顺序编辑九个正有限数。`constraint_profile.enabled` 中 `xy_speed`、`xy_acceleration`、`tangential_acceleration`、`wheel_velocity`、`wheel_acceleration`、`spin_jerk` 的 `false` 表示不参与时间缩放和通过判定；其诊断峰值仍可见。要重新启用某项，把该项改为 `true`，对应值来自配置中保留的 `research_limits`。不要通过修改旧 `v2_workspace_single_goal.yaml` 来改变历史默认值。非完整、前进、九维端点/位置及真实碰撞检查不受这些开关控制。

在包根目录打印实际后端 LimitSet（不会启动 ROS）：

```bash
python3 -B -m v2.scripts.print_limits --config v2/configs/v2_workspace_single_goal_9d_va_only.yaml
python3 -B -m v2.scripts.print_limits --config v2/configs/v2_workspace_single_goal.yaml
```

新 profile 的启用状态、九维数值及有效限值身份保存在结果 `limits.metadata.constraint_profile`。第一条命令中注意 `note` 标明研究限值与 MoveIt/硬件的区别；第二条使用旧完整默认合同。新规划和配对处理示例：

```bash
./v2/scripts/plan_v2.sh --stage planning --config v2/configs/v2_workspace_single_goal_9d_va_only.yaml --task single_goal_05
# 把下面的 P 目录替换为上一步实际打印的 P_PASS 目录
./v2/scripts/plan_v2.sh --stage geometry --input v2/outputs/<P目录>
./v2/scripts/plan_v2.sh --stage g --input v2/outputs/<P目录>
./v2/scripts/plan_v2.sh --stage validate --input v2/outputs/<P目录>
./v2/scripts/plan_unified_v2.sh --stage all --input v2/outputs/<P目录>
```

Unified 入口复制并使用同一个 P，不重新规划；它打印单独的 Unified 输出目录。示例中的 `<P目录>` 是需替换的路径占位符，不是实际目录名。旧全限值实验继续使用原配置 `v2/configs/v2_workspace_single_goal.yaml`；两个配置不互相覆盖。生成结果后可按路径分别运行：

```bash
python3 -B -m v2.scripts.paired_9d_metrics --input v2/outputs/<结果目录>
```

该命令独占写入 `G/paired_metrics_10ms.json`；结果已存在则拒绝覆盖。九维 jerk 分通道报告，按最多 0.01 s 的共同时间采样并分别处理阶段/chart 接缝左右极限。其余禁用项的峰值是诊断，不是额外的通过条件。真实 PlanningScene PASS 与独立 validate 才能支持完整结果；没有现场景时不能将数值 PASS 冒称碰撞 PASS。

## 14. 冻结 H 的通用 G1：一次运行时间与平滑优化

以下命令从已有的 v4 冻结 H 和同一 H 的已验收 G0 出发，不重新规划，也不更改 H。以 P02 为例，在包根目录执行；`--out` 必须是尚不存在的新目录：

```bash
OPENBLAS_NUM_THREADS=1 python3 -B -m v2.scripts.g1_general \
  --geometry v2/outputs/V2-20261030-021501-single_goal_05_TM_RHO_FEEDBACK-RRTstar \
  --g0 v2/outputs/V2-20261030-021525-single_goal_05_TM_PIECEWISE-RRTstar/G \
  --out v2/outputs/MY_NEW_G1_RESULT
```

一次运行自动依次执行有限次时间优化、平滑优化，以及满足 jerk 参考值时最多一次时间回收；无需手动交替运行两种优化。默认尝试复用兼容的本机 V2 ROS 环境；若未启动则启动专用环境，完成后只停止本次启动的环境。指定 `--scene-snapshot <文件>` 可选择已有场景快照；实际场景或模型不一致时碰撞状态不是 PASS。新目录已有内容时拒绝覆盖。

`fast/` 是本次搜索最快的候选，`recommended/` 是时间和平滑兼顾候选；各含轨迹、九维统计、独立复核及 `collision.json`。`summary.json` 记录 G0 路径、阶段与求解器状态；G0 原目录保持可回退。`high_jerk.json` 按通道给出 |jerk|≥4 的累计秒数、占比和峰簇数。碰撞未通过或未知时不得将数值候选视为完整成功。`--numeric-only` 不启动 ROS，但结果明确等待碰撞验证，不能声称完整 PASS。

该入口接收**已完成**的 H 和 G0，并不隐式调用前端或 H；上游须使用相应的既有 v4 H 与分段 G 入口。G1 只支持与输入 G0 相同的冻结几何及 `V2_NINE_DOF_VA_ONLY` 合同。速度/加速度上限为研究合同，不是硬件能力证明。`jerk=5` 是逐通道研究参考，不是机器人硬限制；求解器达到迭代上限不等于轨迹失败或自动通过，须看实际验收结果。旧 P01 的 48.8060108877 s 和 59.630008575 s 成果仍位于原目录，不会被本入口自动纳入新一轮的候选池；比较全历史最快结果时应一并查看。

## 15. 固定父路径一次生成 H→G0→G1

在包根目录执行，输出目录必须不存在；以下使用已有 P02，不重新运行 RRTstar：

```bash
./v2/scripts/run_phg1.sh \
  --input v2/outputs/V2-20261028-224453-single_goal_05-RRTstar \
  --out v2/outputs/MY_NEW_PHG1_RESULT
```

脚本自动加载 Noetic、由指定 P 新建 H 和 G0，再将这次 H/G0 传给 G1；H 和 G0 的实际目录写入输出目录 `result.json`。最终轨迹位于 `g1/recommended/trajectory.csv`，最终碰撞状态在同目录 `collision.json`。仅推荐候选进行真实场景碰撞验收；`g1/fast/` 是数值候选，不应标成碰撞 PASS。G0 独立保留，先做数值验收而不查询碰撞，故单独使用 G0 前仍需完成其碰撞验收。失败时查看 `result.json`、`error.log` 及对应阶段目录，不覆盖或替换已生成阶段。

`generation_wall_s` 包含 H、G0、G1 生成及独立数值复核；`acceptance_wall_s` 另包含环境准备与最终碰撞检查，不包含最后的进程停止开销。这两项是计算墙钟时间，不是轨迹参考执行时间。可用 `--scene-snapshot` 指定期望场景；只停止本次工具所启动的 ROS 进程。

## 16. 研究轨迹只读预览（无需正式验收或注册）

从包根目录执行 `./v2/scripts/view_v2.sh`，在已有 RViz 时改用 `./v2/scripts/view_v2.sh --no-rviz`。播放器只读递归扫描 `v2/outputs/V2-*` 内可关联原始机器人状态的轨迹，同时支持旧目录和统一目录；`list`、`refresh`、`play 编号或名称`、`replay`、`quit` 保持不变。也可直接输入：

```text
play-file /home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit/v2/outputs/V2-20261104-PHG1-P02-ENDTOEND/g1/recommended/trajectory.csv
```

`play-file` 要求绝对路径，但不要求注册、正式验收 PASS 或特定输出目录名；CSV 须有严格递增的 `time_s` 和完整九维位置，且其关联的保存结果须能提供匹配 URDF/SRDF、坐标系与完整初始 RobotState。单独复制到任意位置、丢失关联元数据的 CSV 会报 `MISSING_PARENT_PATH_AND_INITIAL_ROBOT_STATE`，不会猜测未给出的轮关节等状态。原有外部 CSV 注册功能仍可用于显式指定状态模板，但研究预览不再依赖注册。

列表按证据显示 `VERIFIED`、`COLLISION_PENDING`、`NUMERICAL_FAIL`、`UNVALIDATED` 等状态；播放统一提示 `VISUALIZATION_ONLY`。例如 `V2-20261104-194613-single_goal_05_TM_PIECEWISE-RRTstar/G` 为 `COLLISION_PENDING`，而 `V2-20261104-PHG1-P02-ENDTOEND/g1/recommended` 为 `VERIFIED`。预览不改动源 CSV/JSON、不运行规划，也不升级原验收状态；只发布 `/v2/display_trajectory`，不执行机器人。若已有 RViz 订阅名但播放器提示 `NO_SUBSCRIBER`，先检查 RViz 的 `V2_G` 显示是否真正连上话题；消息构造和 ROS 测试订阅成功不能替代 GUI 目视确认。

## 17. 在轨迹旁生成原始曲线图

从 ROS 包根目录运行，无需 ROS。无 CSV 参数时列出 `v2/outputs/` 下具有完整 q/v/a/jerk 表头的轨迹；日期来自 `V2-YYYYMMDD-...` 目录，无法解析时列表会标记 `[mtime]`。在 `plot>` 中输入 `list`、`refresh`、`plot 1`、`plot 1 3`、`plot all` 或 `quit`：

```bash
python3 -B v2/scripts/plot_trajectory.py
python3 -B v2/scripts/plot_trajectory.py --since 2026-10-30
python3 -B v2/scripts/plot_trajectory.py v2/outputs/V2-20261104-PHG1-P02-ENDTOEND/g1/recommended/trajectory.csv
```

每条 CSV 的图保存在自身同目录的 `plots/trajectory_6panel.png` 和 `plots/jerk.png`；已装 Inkscape 时另导出同名 EMF。直接指定多个 CSV 可一次依次绘图，`--since` 只影响自动扫描。曲线直接使用 CSV 原值，不求导、过滤或平滑。再次绘图只覆盖该 CSV 对应的生成图，不清理其他历史图片或实验文件。

## 18. 新规划到 PHG1 的可选单入口

在包根目录，使用新 P 与九维研究限值默认配置：

```bash
./v2/scripts/plan_v2.sh --until g1
```

`--until p`、`--until h`、`--until g0` 依次在新 P、TM 几何 H、数值 G0 后停止；`--until g1` 继续生成与验收推荐 G1。也可追加 `--config <YAML> --task <任务ID>`；H/G0/G1 要求 YAML 是 `V2_NINE_DOF_VA_ONLY`、九维 0.5/1.0 且六项附加研究限值禁用。完整 `--until g1` 在一个新目录下保存 `P/`、`H/`、`G0/G/`、`g1/{fast,recommended}/` 和 `result.json`；内部索引使用相对路径。保留历史 P 续算的 `run_phg1.sh --input <P目录> --out <新目录>`，新目录下复制所需 P 文件至 `P/`，并生成 `H/`、`G0/`、`g1/`，原 P 不移动。`--until p/h/g0` 与旧 `--stage` 入口保持原用法。G0 碰撞仍待验，只有 G1 最终独立数值和离散碰撞都成功才报告 PASS。规划前需符合已有运行环境及场景配置，默认场景快照须与实际场景一致。

`--stage all` 保留旧版 Legacy P→H→G→validate，不包含新 G1；两套入口不可混淆。该入口本次使用模拟阶段测试，不代表重新运行了 RRTstar 或真实端到端验收。

### 末端位姿目标与最多三次规划

复制 `v2/configs/joint_ik_pose_example.yaml`，在任务中填写 `start_q` 和 `pose_goal`：`frame_id: world`、`link_name: Link6`、`position: [x,y,z]`，姿态使用 `orientation_xyzw` 四元数或 `rpy`（rad）之一。运行：

```bash
./v2/scripts/plan_v2.sh --until g1 --config v2/configs/joint_ik_pose_example.yaml --task joint_ik_pose
```

每个任务最多三次；通过 FK、研究范围和 PlanningScene 的九维 IK 解存于 `IK/`，后续重规划复用该解。P/H/G0/G1 或最终碰撞失败时从新的 P 重新规划；只有末端误差、独立数值和碰撞均通过才发布正式轨迹。`summary.json` 和各失败阶段的 `failure.json` 给出次数、阶段和原因。位置与姿态容差分别为 0.01 m 和 0.01 rad，需核对是否满足实际任务。直接使用九维 `goal_q` 时不运行 IK。该功能生成参考轨迹，不执行机器人。

### 五目标预求 IK 与五段参考轨迹

`v2/configs/multi_5goal.yaml` 按顺序保存五个 world 系 Link6 目标、九维起点和 1 秒驻留。运行：

```bash
./v2/scripts/plan_v2_multi.sh --config v2/configs/multi_5goal.yaml
```

入口首先独立求解并验收五个 IK；任一个失败就停止。全部通过后顺序规划全部五个目标，每段最多三次，每次失败从新 P 重试；下一段使用前一段推荐轨迹实际终点作为规划起点。成功的各段及拼接展示文件分别位于新任务目录 `segment_01/` 至 `segment_05/` 和 `task/trajectory.csv`，五个预求结果位于 `IK/`。`result.json` 记录状态与衔接误差；任务 CSV 不替代各段密集轨迹的数值或碰撞证据。每个目标停车 1 秒，包括最终目标；不会控制机器人。要复用已成功的两段和五个 IK，请附加 `--resume v2/outputs/V2-20261112-161412-MULTI-IK-PHG1-RRTstar`；入口会核对配置和场景并重新检查五个目标。

第三段可在单独续跑时明确指定最多 10 次（其他段仍为 3 次）：

```bash
./v2/scripts/plan_v2_multi.sh --resume v2/outputs/V2-20261112-163638-MULTI-IK-PHG1-RRTstar --third-attempts 10
```

衔接位置按 XY 1e-4 m、周期 yaw/关节 1e-4 rad 核验，停车起止速度和加速度仍需满足原门槛；接缝实际差值写入结果。每次驻留的结束点保留上一段位置，下一段完整首行不改值，并在展示时间轴留 2 ns 防止 ROS 时间戳重复。该极短的展示衔接间隙未独立证明连续动力学或碰撞安全，不能将拼接 CSV 当作连续执行安全证明；各段原密集轨迹的验收仍单独保存。
