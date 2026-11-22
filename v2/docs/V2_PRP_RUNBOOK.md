# V2-PRP 手动规划、显示与绘图手册

本文基于实验室电脑 2026-11-19 的实际源码和已存运行证据。**本轮仅检查源码与文档，以下启动、规划、RViz 和 Gazebo 命令均未在本轮执行；标注为“依据源码，待实机验证”。** 规划结果是离线参考轨迹，不等于机器人控制或实际跟踪。已有五目标数值及离散碰撞结果为 193.672418 s，但容量超 30 MiB，正式入口退出码为 1。

## 先选你要做的事

| 目的 | 入口/方式 | ROS Master | MoveIt/仓储场景 | RViz | Gazebo | 当前状态 |
|---|---|---|---|---|---|---|
| 已有 P→R、已有 R→P0、已有 R/P0→P1 数值 | 见 §2：`plan_v2.sh` 阶段入口和 `g1_general --numeric-only` | 不需要连接 | 不需要；仍读取本地模型与限制 | 不需要 | 不需要 | 代码路径支持；本轮未验证完全断网执行。碰撞待验 |
| 新单目标/五目标正式规划 | §3：`plan_v2.sh --config ...` | 需要 | 需要，与 YAML 地图匹配 | 不需要 | 不需要 | 新 P 五目标已有数值/离散碰撞存档 PASS；容量例外 |
| 已存单段 R/P→P0/P1 正式验收 | §3 的 YAML 或 §4 的直接 CLI | R/P0 数值可无；P1 完整验收需要 | 最终碰撞需要 | 不需要 | 不需要 | 已有相关单段存档；不自动证明新候选通过 |
| 手动 MoveIt 交互规划/观察 | §5：附加 RViz MotionPlanning | 需要 | 需要 | 需要 | 不需要 | MoveIt GUI 入口；手动计划不是自动 V2-PRP 全链 |
| 已存单段 CSV 预览 | §5：`view_trajectory.sh interactive` | 需要 | 需要匹配模型和初始状态；预览不执行碰撞 | 需要 | 不需要 | 只发布参考显示消息；读取 t/q，不保留 v/a 到消息 |
| 已存整体五目标 NPZ 预览 | 无正式直接入口 | — | — | — | — | **未支持**：不是 CSV，`play-file` 不能直接读取 |
| 已有标准 CSV 画图 | §7：`plot_trajectory.py` | 不需要 | 不需要 | 不需要 | 不需要 | q/v/a 六面板、jerk 图；XY 另用文中小命令 |
| Gazebo 中观察旧模型/MoveIt | §6：旧 launch | 需要 | 两者可同时有，但 Gazebo world 与仓储 PlanningScene 需另核对 | 可选/演示 launch 含 RViz | 需要 | 旧环境可启动代码；本轮未实机验证 V2 同步回放 |
| 用 V2 轨迹真正驱动 Gazebo 整机 | 无已验正式适配 | — | — | — | — | **未支持/未验证**，不把旧 V1 模型位姿桥接当作已完成 |

> 下文命令示例在用户主动执行时会创建文件或启动节点。本轮未执行；源代码确认不等于你的当前机器场景、依赖或 GUI 已通过验证。

## 1. 通用终端准备与安全边界

```bash
cd /home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit
source /opt/ros/noetic/setup.bash
# catkin 工作区已编译时；如还有 MoveIt overlay，按本机环境先 source 它
source /home/robot/digitaltwin/zzs_ws/devel/setup.bash --extend
export ROS_MASTER_URI=http://127.0.0.1:11411
export V2_MASTER_URI="$ROS_MASTER_URI"
export ROS_HOME=/tmp/prp_newp_ros_home
```

模型、MoveIt 插件、Python 依赖、地图文件必须预先存在。纯数值路径使用包内 URDF/SRDF/limits 及 Python 的 NumPy/SciPy/CVXPY 等依赖；`source` 是加载依赖，不等于启动 ROS。`global.yaml` 的 PRP 路径按包与 YAML 位置定位，支持 `runtime.overlay_setup`；旧直接 CLI、旧 viewer 和 runtime 包装器仍经 `v2/scripts/env_noetic.sh` 的旧固定 overlay（`/home/robot/ws_moveit/devel/setup.bash` 等），不存在时会报错，不能说已跨机通用。

所有命令均以**包根目录**为相对路径基准。除明确说明，使用独立新输出目录，切勿覆盖旧结果。不要随意停止外部 Master、MoveIt、RViz 或 Gazebo。

## 2. A：只做离线数值，不启动 ROS/MoveIt/Gazebo

### 2.1 保存 P→Signed R（可选再到 P0）

**依据源码，待实机断网验证。** 只要求 Python/模型文件存在；`--until r` 在构造/数值验收后返回，`--until g0` 在 P0 数值结果后返回，均不执行 PlanningScene 碰撞。已保存 P 必须含 `parent_path.json` 及兼容模型/约束。

```bash
# 接 §1；以下输出目录必须不存在
P=/绝对路径/parent_path.json
OUT="$PWD/v2/outputs/manual-offline-p-$(date +%Y%m%d-%H%M%S)"
./v2/scripts/plan_v2.sh --from-p "$P" --reconstruction signed --until r --out "$OUT"
# 要从该次生成的 R 继续数值 P0，见下一节；也可首次将 --until r 改为 --until g0。
```

期待 `SIGNED_R_NUMERICAL_PASS` 及 `$OUT/R/geometry.json`、`candidate.npz`、`verification.json`。直接 CLI 默认 22 控制点，不会自动启动多目标中的 22→36 或 θ–ρ 恢复；不可据此判定全局无解。

### 2.2 保存 R→P0，以及不经 ROS 的 P1 数值

```bash
# 接 §1；R 须包含 geometry.json、parent_path.json 及其实际引用的 candidate.npz
R=/绝对路径/已保存R目录
G0_OUT="$PWD/v2/outputs/manual-offline-r-$(date +%Y%m%d-%H%M%S)"
./v2/scripts/plan_v2.sh --from-r "$R" --until g0 --out "$G0_OUT"

# P0 完成后：独立调用现有正式 P1 求解器的 numeric-only 模式
P1_OUT="$PWD/v2/outputs/manual-offline-p1-$(date +%Y%m%d-%H%M%S)"
/usr/bin/python3 -B -m v2.scripts.g1_general \
  --geometry "$R" --g0 "$G0_OUT/P0/G" --out "$P1_OUT" \
  --numeric-only --allow-deferred-g0-collision
```

P0 的 `$G0_OUT/result.json` 应为 `P0_NUMERICAL_PASS_COLLISION_PENDING`，P1 的 `$P1_OUT/summary.json` 应为 `NUMERIC_ONLY_COLLISION_PENDING`，候选在 `fast/`、`recommended/`。G0 的 geometry 副本、输入 R、P0 时间律和模型必须匹配。**这不是完整 P1/碰撞 PASS**；最后检查要在匹配的真实 PlanningScene 上进行，参见 §3/§4。不要把离线命令误写成硬件运行，也无需启动或停止 ROS 进程。

## 3. B：无图形 MoveIt + 正式 global.yaml 规划

### 3.1 复用现有服务，或启动 V2 自有环境

接 §1，先查看：

```bash
rosnode list
rosservice list | grep -E '^/(get_planning_scene|check_state_validity|plan_kinematic_path|compute_ik|compute_fk)$'
./v2/scripts/status_v2.sh
```

- 若外部 Master/MoveIt 已经运行且为预期机器人和仓储场景，不启动另一套；`status_v2.sh` 对外部 master 可能显示 `INCOMPATIBLE_UNOWNED_MASTER`，不意味着应接管或杀掉它。服务名字存在也不证明地图正确。
- 若确认 11411 无 Master，且 `status_v2.sh` 为 `STOPPED`，可启动现有 V2 自有环境：

```bash
# 依据源码，待本机验证；使用旧环境包装器的固定 overlay
./v2/scripts/start_v2.sh --config v2/configs/v2_workspace_single_goal_9d_va_only.yaml
./v2/scripts/status_v2.sh
```

`start_v2.sh` 启动并记录自有 `roscore -p 11411` 和 `v2/launch/v2_headless_test.launch`，后者启动 MoveIt、机器人描述/TF、OctoMap，不启动 RViz/Gazebo，且 `allow_trajectory_execution=false`。`runtime_v2.py` 会探测场景，必要时用 `scene_loader.py` 将订阅到的 OctoMap 加入 PlanningScene；不可将此注入用于未获授权的外部场景。`READY` 仍应配合规划入口的现场模型、地图核验。
- 新 P 的 YAML 正式入口若 Master 不存在，使用 11411 可尝试调用同一 V2 启动逻辑；若已有 Master 可达但 MoveIt 不完整，不会自动新开第二套来修复。外部现场必须通过 `V2_PRP_SCENE_SNAPSHOT` 或 YAML `scene.planning_scene: snapshot` / `scene.snapshot` 显式提供**与当前现场匹配**的场景快照（自有 READY 场景使用自己的 map 检查）。

```bash
# 对外部现场：请由环境维护者确认快照来自当前场景，不要盲用历史场景
export V2_PRP_SCENE_SNAPSHOT=/绝对路径/匹配当前场景的scene_snapshot.bin
```

地图在启动前选择；运行中的 Master 不做动态换图。`global.yaml` 中 `scene.octomap`、实际加载地图及碰撞场景必须一致。

### 3.2 配置与运行

```bash
cp -n v2/configs/global.yaml v2/configs/local_prp.yaml
# 在编辑器中按下表修改 local_prp.yaml，然后：
./v2/scripts/plan_v2.sh --config v2/configs/local_prp.yaml
```

| 需求 | 必需 YAML 设置 |
|---|---|
| 新单目标 | `task.mode: single`；`input.stage: new`、`input.path: null`；`task.source: configured_goal`，填写九维 `start_q`、`goal`（world/Link6 位置与 quaternion_xyzw）；`planner.id: RRTstar`、`group: whole`、有效 `allowed_time_s`。 |
| 新五目标 | `task.mode: multi`；new/g1、`source: configured_goal`；填写原九维起点和恰好五个有序 `task.targets`，`dwell_s: 1.0`。从头运行时不要配置 `reuse_segments`。 |
| 保存 P | single；`input.stage: P`，`input.path` 指向 P 目录或 `parent_path.json`；`source: saved_parent`，`start_q: null`、`goal: null`、`planner.allowed_time_s: null`。 |
| 保存 R | 如保存 P，但 `input.stage: R`、路径指含 `geometry.json`、`candidate.npz`、`parent_path.json` 的目录。 |

`input.until: g1` 走最终碰撞；`g0` 是 P0 数值与碰撞待验。当前算法组合 `signed_v1/a_c_v1/formal_v1`，九维限值 profile 不要自行放宽。多目标尝试 22→36 和特定三 chart θ–ρ 恢复；单目标 YAML 默认不自动启用恢复。多目标每段当前默认最多 6 个候选，**通过 `plan_v2.sh` 导出的 `V2_PRP_MAX_ATTEMPTS` 不会透传**；跨进程续跑也不自动累计旧预算。

YAML 的相对输入/输出路径相对配置文件所在目录；`package://cr5_car_moveit/...` 基于包路径。入口打印 `PRP_CONFIG` 和 `PRP_GLOBAL_RESULT` 或 `PRP_MULTI_RESULT`。常见结果：

- 单段：YAML `output.root` 下 `PRP-*/effective_config.json`、`result/P`、`result/R`、`result/PRP/P0/G`、`result/PRP/P1/recommended/trajectory.npz`、`result/PRP/collision.json`、`storage.json`；紧凑 CSV 可能因容量省略。
- 五目标：`PRP-MULTI-*/effective_config.yaml`、`result.json`、成功时的 `task_trajectory.npz` 与各段必要参数。务必同时检查 `status`、每段 FK/碰撞、`storage.status` 和 shell `$?`；`SIGNED_FIVE_GOAL_TASK_PASS` 可能伴随 `OUTPUT_BUDGET_EXCEEDED` 和退出码 1。

### 3.3 收尾

规划命令结束不等于其自有后台 ROS 环境自动关闭。**仅当确认由本次本人通过 V2 `start_v2.sh` 或自动启动创建、且不再被他人使用时**，运行 `./v2/scripts/stop_v2.sh`；它按记录检查所有权并尝试停止自己启动的 launch 和 roscore。不要用它处理外部 Master 的占用或盲目 `kill`。外部环境由其所有者管理。

## 4. 保存 P/R 的直接 CLI

这是原有显式续算入口，输出目录必须尚不存在。**依据源码，待实机验证。** `--until r/g0` 的数值模式见 §2；`--until g1` 进行真实最终碰撞，需要正确 ROS/MoveIt/PlanningScene：

```bash
# 接 §1 和 §3.1；scene 为显式确认的同一现场快照
P=/绝对路径/parent_path.json
R=/绝对路径/已保存R目录
SCENE=/绝对路径/匹配当前场景的scene_snapshot.bin
./v2/scripts/plan_v2.sh --from-p "$P" --reconstruction signed --until g1 \
  --scene-snapshot "$SCENE" --out "$PWD/v2/outputs/manual-from-p-$(date +%Y%m%d-%H%M%S)"
./v2/scripts/plan_v2.sh --from-r "$R" --until g1 \
  --scene-snapshot "$SCENE" --out "$PWD/v2/outputs/manual-from-r-$(date +%Y%m%d-%H%M%S)"
```

该旧包装器的固定 overlay 限制见 §1，且不自动打开 Signed R 22→36/联合恢复；当前没有证据说完整任务 NPZ 可作为它们的输入。规划输出不会自动发布 RViz。

## 5. C：MoveIt/RViz 观察与单段预览

### 5.1 已有 MoveIt/场景上附加 RViz

**依据源码，待 GUI 实机验证。** 先完成 §1 和 §3.1 的 Master/MoveIt/仓储场景检查，再开独立图形终端：

```bash
# 新终端重新执行 §1 的 source 与 ROS_MASTER_URI 设置
roslaunch cr5_car_moveit moveit_rviz.launch \
  rviz_config:="$(rospack find cr5_car_moveit)/v2/rviz/v2_g.rviz"
```

这只附加 RViz，不再启动 MoveIt。配置固定帧 `world`，`V2_G` 轨迹 topic 为 `/v2/display_trajectory`。要手动使用 MoveIt MotionPlanning 面板，需启用该显示并选 `whole` 组、设置起点与目标；面板的 MoveIt Plan 不是自动调用本项目 P→R→P0→P1，且 headless V2 禁用执行。仓储地图在 PlanningScene 上真实存在后才可观察相关障碍。

若使用 V2 **自有** runtime，也可用 `./v2/scripts/view_v2.sh` 一起打开 RViz 和交互播放器；已有 RViz 用 `--no-rviz`。该包装器调用 `runtime_v2.start`，会拒绝未受其管理的外部 Master；外部 Master 请使用上面的独立 RViz 和下列 `view_trajectory.sh`，不应让它接管。`view_v2.sh` 退出时只尝试关闭它自己启动的 RViz 子进程，不自动停止 V2 ROS runtime。

### 5.2 播放有完整关联信息的单段 CSV

一次 §3 单目标规划结束后，若紧凑 CSV 未被容量机制省略，先在规划结果终端取得它的绝对路径，再用于下面的 `play-file`（不要把 R/P0 的 CSV 误作推荐 P1）：

```bash
RUN=/绝对路径/本次单段PRP运行目录
CSV="$RUN/result/PRP/P1/recommended/trajectory.csv"
test -f "$CSV" && printf '%s\n' "$CSV"
```

若文件不存在，请先看 `storage.json` 的 `csv_omitted`、P1 数值与碰撞状态；不存在并不自动意味着数值规划失败。

另开终端接 §1（需正确的本机旧 overlay）并运行：

```bash
V2_MASTER_URI=http://127.0.0.1:11411 ./v2/scripts/view_trajectory.sh interactive
```

在 `viewer>` 中输入：

```text
list
play-file /绝对路径/已保存单段/trajectory.csv
replay
quit
```

`play-file` 需要 CSV 的邻近结果、geometry/parent 中的真实初始 RobotState、模型文件和坐标系，及严格递增且 ROS 纳秒量化后不重复的时间；文件复制到 `/tmp` 而丢失这些关联信息会报错。播放器构建 JointTrajectory + MultiDOF 并发布到 `/v2/display_trajectory`；此预览读 **t/q**，并不把 CSV 中 v/a 作为播放命令。RViz 中预览不代表执行、碰撞认证或控制跟踪。

若已有单段 YAML 结果因容量省略 CSV，先使用 §7.1 的导出命令供绘图；该导出**位于结果目录外**，普通 `play-file` 无法保证找到邻近元数据。可先准备与该轨迹模型/完整初始状态匹配的注册模板，再使用 `register-csv` 注册预览；不能把随意选用的模板当作真实初始状态。已留在原标准单段目录的 CSV 则优先直接 `play-file`。本轮五目标整体 NPZ 没有现成的正式一键转 CSV/播放入口。

退出 `viewer>` 输入 `quit`；自行启动的 `roslaunch moveit_rviz.launch` 在其终端 Ctrl-C，只关闭该终端的 RViz。ROS runtime 的关闭按 §3.3。

## 6. D：Gazebo + MoveIt 可观察环境（旧 V1，独立使用）

**依据 launch 源码，待实机验证。** 不要在已有 11411 V2 MoveIt 旁再启动重复节点；推荐先获环境所有者同意，在**确认空闲**的独立 11511 上运行。Gazebo world 和 MoveIt PlanningScene/OctoMap 是不同对象，必须分别核对，不能因为看到模型就断言两者相同。

终端 A：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11511
roscore -p 11511
```

终端 B，仅 Gazebo 模型（默认 empty.world；如需地图需显式 `world_name:=...`）：

```bash
source /opt/ros/noetic/setup.bash
source /home/robot/digitaltwin/zzs_ws/devel/setup.bash --extend
export ROS_MASTER_URI=http://127.0.0.1:11511
roslaunch cr5_car_moveit gazebo.launch
```

该 launch 启动 Gazebo、模型、机械臂控制器和 robot_state_publisher，**不包含 MoveIt 或 RViz**。结束时只在自己终端 Ctrl-C。

如果要用旧演示环境将 Gazebo、MoveIt 和 RViz 一起启动：先结束自己刚启动的 Gazebo，再在终端 B 使用以下**替代**命令（不要同时启动上面两个）：

```bash
roslaunch cr5_car_moveit demo_gazebo.launch enable_old_collector:=false
```

该 launch 默认 world 是 `map/1goal.world`，并启动旧 `ideal_trajectory_bridge.py` 和旧 MoveIt/RViz；默认并不能证明加载了 V2 的 `warehouse_cropped.bt` PlanningScene。该桥在收到旧 DisplayTrajectory 和 action 后 50 Hz 设置 Gazebo 模型位姿，机械臂是另一控制器；这不是已经验证的 V2 整机同步时间轨迹控制。不要点击 Execute 来假定 V2 轨迹已接入。

仅显示 V2 单段 CSV 时，可在同一 Master 上附加 §5 的 RViz/播放器，但需先确认完整模型和场景、合适显示配置；此时图形终端使用 `ROS_MASTER_URI=http://127.0.0.1:11511`，播放器命令改为 `V2_MASTER_URI=http://127.0.0.1:11511 ./v2/scripts/view_trajectory.sh interactive`，不要照搬 §5 的 11411。它只是 RViz 预览，不驱动 Gazebo。若不需要 V1 MoveIt Execute，可避免发布到其 `/move_group/display_planned_path`；V2 viewer 发布 `/v2/display_trajectory`。**本项目尚无已验的 V2 NPZ→Gazebo 整机同步回放/控制入口。** 结束时在本人终端分别 Ctrl-C 关闭 roslaunch、roscore；不要影响别人的 Master。

## 7. E：已有 CSV 绘图与底盘 XY 路径

### 7.1 先获得可读 CSV

单段成功运行若 `result/PRP/P1/recommended/trajectory.csv` 存在，直接使用。若被容量机制省略，可用**单段** YAML 结果运行目录导出到目录外（依据源码，待实机验证）：

```bash
RUN=/绝对路径/单段PRP运行目录
CSV=/tmp/prp_single_display.csv
./v2/scripts/plan_v2.sh --export-csv "$RUN" --csv-out "$CSV"
```

需要相应 R、P0 时间律、P1 λ 和密集 P1 NPZ；`CSV` 不可已存在且不可放进运行目录。它选择原始行供展示，不能取代密集轨迹碰撞证据。该命令不接受五目标 `task_trajectory.npz`。

### 7.2 六面板与 jerk

`plot_trajectory.py` 读取 `time_s` 及 `q_`、`v_`、`a_`、`jerk_` × 9 个标准通道（`world_car/x`、`world_car/y`、`world_car/theta`、`joint1`…`joint6`），要求时间严格递增；额外列允许。V1 CSV 如果列名或缺少 v/a/jerk 不满足这些要求，不能直接用该脚本，也不应把数值差分或补零冒充原测值。

```bash
# 指定完整 CSV 路径，可不在 v2/outputs；--output 是新图根目录
CSV=/绝对路径/符合标准字段的trajectory.csv
python3 -B v2/scripts/plot_trajectory.py "$CSV" --output /tmp/prp_plots
```

结果位置由终端 `CSV -> 目录` 精确打印：`trajectory_6panel.png`（q/v/a 对时间）、`jerk.png`、`source.txt`，若本机 Inkscape 可用还输出 EMF。默认不指定 `--output` 时写原 CSV 同级 `plots/`，可能覆盖同名图；为保护历史结果优先指定新输出目录。无位置参数时仅扫描 `v2/outputs` 内名为 `trajectory.csv` 的文件并进入交互模式，可输入 `list`、`plot 1`、`plot all`、`quit`。

### 7.3 XY 底盘路径图

现有 `plot_trajectory.py` **不输出 XY 平面路径图**。对具备标准 `q_world_car/x`、`q_world_car/y` 列的现有 CSV，以下小命令可单独画原行连线（不更改源 CSV；依据源码与 Python API 编写，本轮未执行）：

```bash
CSV=/绝对路径/符合标准字段的trajectory.csv
XY_PNG=/tmp/prp_xy_path.png
/usr/bin/python3 - "$CSV" "$XY_PNG" <<'PY'
import csv, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
with open(sys.argv[1], newline='', encoding='utf-8-sig') as f:
    rows = list(csv.DictReader(f))
x = [float(r['q_world_car/x']) for r in rows]
y = [float(r['q_world_car/y']) for r in rows]
if len(x) < 2:
    raise ValueError('CSV needs at least two rows')
fig, ax = plt.subplots()
ax.plot(x, y); ax.scatter([x[0], x[-1]], [y[0], y[-1]], marker='o')
ax.set(xlabel='world x (m)', ylabel='world y (m)', title='Base XY reference')
ax.axis('equal'); ax.grid(True); fig.savefig(sys.argv[2], dpi=200, bbox_inches='tight')
print(sys.argv[2])
PY
```

`/tmp/prp_xy_path.png` 如已存在会被覆盖，请改为自己的新路径。展示 CSV 的抽样连线不是连续碰撞认证。五目标整体 NPZ 当前不能直接输入六面板脚本或 `play-file`；**最小后续改进**是增加只读整体 NPZ→标准紧凑 CSV 的适配，保留全局时间、五目标事件、方向和必要极值，再用现有绘图/显示器，并明确预览插值不继承原碰撞验收。本轮不实现。

## 8. 实际接口缺口与 Git 检查

- **真实接口缺陷**：`plan_v2.sh` 的清洁环境不透传 `V2_PRP_MAX_ATTEMPTS`；多目标检查点跨进程也不会自动累计重试次数。
- 单目标默认不带多目标 Signed R 恢复；部分相邻 MOVE 换向和多 chart 恢复尚不支持。
- 已存 P/R 直接 CLI、viewer/runtime 旧包装器仍有旧工作区绝对路径；优先 global YAML，迁机前核对 overlay。
- 整体 NPZ 没有现成标准 CSV 导出/单次完整轨迹预览；现有单段 CSV 外置导出后需要解决模型与初态关联。
- Gazebo 真实同步回放未完成；RViz 是预览，不是动力学或硬件执行。30 MiB 目标另行处理。

```bash
cd /home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit
git status --short
git branch --show-current
git remote -v
git log -1 --oneline
git diff --stat
```

当前工作区含未提交修改与未跟踪代码；同步前先审阅、处理两机各自改动，再按用户授权使用 Git。不要以 reset/pull 覆盖未知修改。

## 9. 两台电脑的 Git 日常同步

以下命令作用于 `cr5_car_moveit` 仓库，不会同步 ROS、MoveIt 安装或实验数据。执行 `pull`、切换版本前先看状态；有未提交修改时先自行提交到合适分支或安全保存在另一个位置，不要用 `reset --hard`、`clean` 或强制 push 覆盖。命令使用现有系统 GitHub SSH 认证，不需要在文档中写 token 或私钥。

### 查看版本与本地修改

```bash
cd /home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit
git status --short --branch
git branch -vv
git remote -v
git log -1 --oneline
git tag --points-at HEAD
git diff --stat
```

### 修改后提交并推送

先核对状态和差异，只暂存明确需要的源码/配置/文档，不用 `git add .`。将下例路径换成自己实际修改的文件：

```bash
git status --short
git diff -- v2/scripts/plan_v2.sh
git add -- v2/scripts/plan_v2.sh
git diff --cached --stat
git diff --cached --check
git commit -m "Describe the change"
git push origin master
```

如果当前分支不是 `master`，将最后一条的 `master` 改为实际分支；有远端非快进冲突时先 fetch 并阅读差异，不能强推覆盖。

### 另一台电脑首次获取与之后更新

先克隆到独立目录，不覆盖该机已有 catkin 包、独有修改、地图或模型：

```bash
mkdir -p "$HOME/prp-sync-review"
git clone git@github.com:MikoyaD/DigitalTwin-WBP.git "$HOME/prp-sync-review/cr5_car_moveit"
cd "$HOME/prp-sync-review/cr5_car_moveit"
git status --short --branch
git log -1 --oneline
git tag --list 'v2-prp-*'
# 在把源码用于该机 catkin 工作区前，先与已有包人工比较并保存独有变更
```

在另一台已经正确接入仓库、且工作区干净的电脑上，日后可：

```bash
git status --short --branch
git fetch origin --tags
git log --oneline --left-right HEAD...origin/master
git pull --ff-only origin master
```

如两台都有未提交改动，先各自检查并提交或妥善保存，再比较共同祖先及远程修改；先解决冲突，不能直接覆盖。`--ff-only` 若拒绝，先处理分支历史，而不是改用强制操作。

### 查看和临时切换历史 tag

```bash
git fetch origin --tags
git tag --list 'v2-prp-*'
git show --no-patch --oneline v2-prp-five-goal-20261119
git status --short --branch
git switch --detach v2-prp-five-goal-20261119   # 仅在工作区可安全切换时
# 返回原分支（示例为 master）：
git switch master
```

分离 HEAD 适合查看历史；如需从历史版本继续开发，先建自己的分支。持续同步 V2 源码、必要文本配置、研究说明和开发指令；必要场景快照（如确在使用的 `default_scene_snapshot.bin`）先确认两端位置和用途，再明确选择跟踪或逐文件传输，保留实验室已验收配置，不盲目覆盖。未来修改的 Isaac 扩展 Python 位于包外时也须单独列入同步清单。

两机已有的 `robot_description/meshes/`、`robot_description/usd/`、`robot_description/env/`、大型 USD/mesh/贴图/环境资源既不参与日常 Git，也不参与日常 rsync，不删除或覆盖；确需修改时单独指定文件。无需 Git LFS 或对静态资产做 SHA-256 检测。`v2/outputs/`、完整 NPZ/CSV、历史 evidence、构建产物和系统 ROS 依赖不做日常源码同步。用户说明实验室电脑未使用本地 Codex Agent：无需迁移会话或 `~/.codex/`，个人电脑的 Codex 历史独立保留。

下一轮先最小修正 `env_noetic.sh` 的双机兼容，保留实验室现场场景和配置，不凭推测恢复 `ros_probe.py`、`scene_loader.py` 历史版本；两端 SSH 建立后再比较实际所需源码，按 `SYNC_FILES.md` 轻量同步。库内 `v2/docs/V2_PRP_RUNBOOK.md` 是此手册的副本；外部主报告 `/home/robot/digitaltwin/项目报告.md` 保持原位，不由该仓库自动同步。


### Gazebo + RViz 联合回放：尚未实现

2026-11-19 独立 11511 Master 的 Gazebo 预检发现六轴 PID 缺省报错与 CSV 初始关节位置不一致；影响尚待验证，联合回放协调器、实际状态镜像及完整实验未完成。现有 `./v2/scripts/view_v2.sh` / `play-file` 仍只作参考轨迹 RViz 预览，不能控制 Gazebo。不要把 Gazebo 和 RViz 分别显示参考动画视为实际状态同步。详情：`/home/robot/digitaltwin/research_management/evidence/V2_PRP_GAZEBO_REPLAY_20261119/REPORT.md`。


### CSV 的 Gazebo 联合回放：实验原型，尚未验收

2026-11-20 在隔离 `11511` 环境执行了一条单段 CSV。`--gazebo-csv` 原型可驱动 Gazebo，但 TF/反馈时间配对超过 40 ms 的样本存在，RViz 实际显示、暂停同步及场景 origin 尚未验收；请勿用于机器人或视为正式功能。现有 `./v2/scripts/view_v2.sh` 参考预览用法不变。实验命令、依赖和证据详见 `/home/robot/digitaltwin/research_management/evidence/V2_GAZEBO_CSV_REPLAY_20261120/REPORT.md`。


### Gazebo + RViz：启动参数修复，仍属实验模式（2026-11-20）

`gazebo_replay.launch` 已为 world、CSV 首态及关节设置默认值，并自动启动仓储 OctoMap 加载和 RViz；需在独立 `11511` Master 上启动。实际测试中 Gazebo、控制器、场景服务、地图话题及 RViz 节点可用，但 MoveIt 启动记录缺少 `~moveit_controller_manager` 的 FATAL 和地图注入期间的 TF 变换错误，完整回放同步仍未通过验收。切勿将此实验入口用于真实机器人。

终端 A/B/C 完整可复制的启动、回放、停止命令及注意事项见 `/home/robot/digitaltwin/research_management/evidence/V2_GAZEBO_STARTUP_20261120/REPORT.md`。B 使用 `roslaunch cr5_car_moveit gazebo_replay.launch`；C 必须用 `view_v2.sh --gazebo-csv ... --replay-output ... --no-rviz` 复用 B 中的 RViz。默认参考轨迹预览入口不变。


### 单窗口 Gazebo 展示（2026-11-20，视觉待验）

联合 launch 默认创建唯一新 RViz；CSV 入口默认复用，不再需要 `--no-rviz`，如确需另开使用 `--replay-open-rviz`。联合配置默认 RobotModel 与 `/visual_hologram_map` PCD 开、OctoMap 关但可勾选；PlanningScene 地图由等待 TF 的加载器自动注入。实测节点、订阅和 PlanningScene 可用，但 RViz 实际画面及 OctoMap 开关尚待用户确认，历史时序偏差也未解决。A/B/C 完整命令及风险见 `/home/robot/digitaltwin/research_management/evidence/V2_GAZEBO_SINGLE_RVIZ_20261120/REPORT.md`；仅用于独立仿真。


### V2 原布局派生的 Gazebo 单窗口（2026-11-20）

联合 launch 从 `v2_g.rviz` 派生实际状态配置，默认显示 Gazebo RobotModel 和固定 PCD，OctoMap 默认关闭但可勾选，参考动画关闭。CSV 入口默认复用窗口；`--replay-open-rviz` 才另开。环境就绪不等于 CSV 已开始，终端 C 才进行对位和实际回放。独立 Master 的三终端命令、实际节点结果和未解决的 RViz 模型报错及同步限制见 `/home/robot/digitaltwin/research_management/evidence/V2_GAZEBO_V2_LAYOUT_20261120/REPORT.md`。视觉观察由用户完成。


### 联合地图显示更正（2026-11-20）

取代上一节“默认固定 PCD”的说明：联合 RViz 默认使用原 V2 的 `V2 warehouse occupied centers`（`/octomap_point_cloud_centers`），继承原 `v2_g.rviz` 的颜色及点大小；独立 `Fixed warehouse OctoMap` 仍默认隐藏且不影响 PlanningScene。显示数据来自原 `warehouse_cropped.bt` 的 octomap_server，不再默认发布彩色 `warehouse.pcd`。重启联合 launch 生效，三终端命令不变；颜色和地面效果由用户本人确认。时序分析与限制见 `/home/robot/digitaltwin/research_management/evidence/V2_GAZEBO_MAP_DELAY_20261120/REPORT.md`。


### Gazebo 联合展示与 V2 参考预览（2026-11-20）

普通 `./v2/scripts/view_v2.sh` 使用原 RViz-only 预览：V2_G 订阅 `/v2/display_trajectory`，Trajectory_TOTG 订阅 `/move_group/display_planned_path`，可用于参考轨迹显示；不驱动 Gazebo。`play-file` 按上文要求读取带初始状态等关联资料的标准单段 CSV，不能直接读取整体 NPZ。

联合展示使用独立 11511 Master 与 Gazebo：终端 A 运行 `roscore -p 11511`；终端 B 运行 `roslaunch cr5_car_moveit gazebo_replay.launch`（两个终端均先 source Noetic 与实际工作区 setup，导出 `ROS_MASTER_URI=http://127.0.0.1:11511`、`ROS_HOME=/tmp/v2_csv_replay_ros`）；终端 C 在包根导出 `V2_MASTER_URI=$ROS_MASTER_URI` 后运行：

```bash
OUT="v2/outputs/manual-replay-$(date +%Y%m%d-%H%M%S)"
./v2/scripts/view_v2.sh --gazebo-csv v2/outputs/V2-20261114-SIGNED-P2-DISPLAY/G/trajectory.csv --replay-output "$OUT"
```

A/B/C 的完整环境加载与停止命令见本手册前文；B 仅准备环境，C 才会对位并驱动仿真。只有 B 默认打开 RViz，C 默认复用；必要时可显式 `--replay-open-rviz`。输入可换成同格式单段 CSV；不支持 NPZ，未验收完整多目标 Gazebo 播放。联合显示使用固定 `warehouse_cropped.bt` 的 `/octomap_point_cloud_centers` 和实际 RobotModel；独立 OctoMap 默认隐藏，可在 Displays 勾选而不改变 PlanningScene。联合模式不加载 V2_G、Trajectory_TOTG 或 MotionPlanning 参考动画。结束先 Ctrl-C 自己的 C、B，再仅停止自己启动的 A，不关闭 11411。

用户报告手动回放中 Gazebo 与 RViz 运动一致，这是使用观察；已有数据中 TF 相对参考拟合约 30 ms，时戳与仿真/GUI 真值关系未确定，不作同步精度或动力学闭环声明。源码同步不包括 `v2/outputs`、模型 mesh/USD 或大型场景；另一台机器需已有 ROS Noetic、Gazebo、MoveIt、catkin overlay、仓储 world/机器人模型和本地 CSV，地图 `map/warehouse_cropped.bt` 随仓库提供。详见 `SYNC_FILES.md`。

### 统一交互入口（2026-11-22，取代上一节中“无参数直接进入 RViz-only”）

```bash
cd /home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit
./v2/scripts/view_v2.sh
```

输入 `1` 进入原 11411 RViz 参考轨迹交互，沿用 `list`、`refresh`、`play 编号`、`play-file 绝对路径`、`replay`、`quit`；已有含参数的 RViz-only 用法不变。V2_G 订阅 `/v2/display_trajectory`，Trajectory_TOTG 订阅 `/move_group/display_planned_path`；二者是参考，不表示仿真执行。

输入 `2` 进入 11511 Gazebo+RViz：如果端口空闲，入口启动本次 ROS Master 和联合 launch，一个实际机器人 RViz；如果端口已有 Master，只有必要 Gazebo、MoveIt、控制器状态、固定地图、TF 及 RViz 接口均就绪时复用，否则报错而不接管。环境启动不等于运动，输入 `play 编号` 才对齐并执行对应 CSV，`replay` 再播、`list`/`refresh` 重新列出、`quit` 返回模式选择。退出本次菜单只停止它自己启动的 11511 进程；预先存在的 11511 和 11411 进程不会被主动停止。源目录按现有 V2 注册表及 outputs 扫描，列出单段/模块和多目标子段，明确子段不是完整任务；输出状态需查看所属报告。首次扫描只快速检查 CSV 首尾和元数据，实际播放前仍进行完整 CSV 校验。仅支持原协调器能够读取的标准九维有时间 CSV；整体 NPZ 不支持。轨迹目录不随 Git 克隆，需在本机保留；选择显示不等于轨迹安全或实际执行通过。

**高级三终端方式保留**，在 Ubuntu 图形环境中每个终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/robot/digitaltwin/zzs_ws/devel/setup.bash --extend
export ROS_MASTER_URI=http://127.0.0.1:11511 ROS_HOME=/tmp/v2_csv_replay_ros
```

终端 A：`roscore -p 11511`。终端 B：`roslaunch cr5_car_moveit gazebo_replay.launch`，只准备环境、加载固定地图并开一个 RViz，不播放。终端 C：

```bash
cd /home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit
export V2_MASTER_URI=http://127.0.0.1:11511
OUT="v2/outputs/manual-replay-$(date +%Y%m%d-%H%M%S)"
./v2/scripts/view_v2.sh --gazebo-csv v2/outputs/V2-20261114-SIGNED-P2-DISPLAY/G/trajectory.csv --replay-output "$OUT"
```

C 才会开始对位和运动，默认不另开 RViz；如确需第二窗口可显式 `--replay-open-rviz`。Displays 中 `Gazebo actual state` 与原 V2 固定占据中心点云默认显示，独立 `Fixed warehouse OctoMap` 可勾选、默认关闭；关闭显示不删除场景。依次 Ctrl-C 本人终端 C、B、A，不停止外部 11411 或别人的窗口。运行记录为 `state.csv`；异常及模拟时钟/TF/控制器就绪情况看终端和日志。手动视觉观察由用户进行；已有 30 ms TF 对参考拟合不等于 GUI 延迟或底盘动力学闭环。

### 2026-11-22：CSV 输出与轨迹列表约定

本节取代早期“联合模式仅支持单目标 CSV”的表述：标准多目标任务级 CSV 也可选择。用户终端记录了 `V2-20261113-FIVE-GOAL-FINAL-RECOMMENDED/task/trajectory.csv` 完整回放 225.725087 s、5638 状态、协调器退出码 0；`v2/outputs/manual-replay-uppnr9ck/state.csv` 现场有 5638 行反馈。该用户记录不改变原规划的验证状态，也不证明任意多目标轨迹或物理闭环。

`./v2/scripts/view_v2.sh` 的两个模式共用扫描器，显示 `实验目录/相对模块路径 | 单目标/多目标/多目标组分 | 状态`；`play 编号` 时输出绝对路径，Gazebo 对明确碰撞失败的条目先确认。可扫描历史 `V2-*`、`PRP-*` 与 `prp_yaml*` 中的有效 `trajectory.csv`，包含推荐、fast 与模块轨迹；源轨迹已验证的展示抽样不等于该 CSV 已经碰撞认证。存在模型、起点或关联元数据缺失时会列出 INVALID，不自动代换模型。`list`、`refresh`、`play`、`replay`、`quit` 与高级直调方式不变。

新 PRP 正式 YAML 运行目录采用 `V2-YYYYMMDD-HHMMSS-SINGLE|MULTI-IKPRP|PRP-规划器`。新 Link6 联合 IK 的任务是 IKPRP；从保存 P/R 续算不执行新 IK，名称为 PRP，规划器取保存父路径。新多目标路径包括 `segments/segment_01/attempt_01/result/{P,R,PRP/P0,PRP/P1}` 和 `task/trajectory.csv`；失败重试递增 attempt，具体状态以 result.json 为准。单目标推荐与快速 CSV 在 `result/PRP/P1/{recommended,fast}/trajectory.csv`，快速候选并不自动获得推荐候选的碰撞验收。旧非 PRP 算法及历史目录不重命名。

任务级 CSV 由本轮运行的拼接数组直接写出，使用标准 time/q/v/a/jerk 九维列和 segment/chart/direction；包含实际运动与驻留行，不从历史 NPZ 转换。最终 CSV 不再因 30 MiB 上限被删除；如超限，storage 报告真实失败。NPZ 已被用户列为废弃格式，扫描和展示不把它作为输入；现有求解和碰撞链内部仍依赖其数组和解析参数，本轮并未移除这些内部依赖或历史文件。新输出模块已用已有 CSV 数值做写入与两种读取器检查，尚未在本次运行一次真实五目标规划验证完整新目录。
