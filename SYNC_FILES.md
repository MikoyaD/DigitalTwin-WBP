# V2-PRP 源码同步边界

本仓库以 `cr5_car_moveit/` 为 catkin 包根目录。同步 `v2` 的正式源码、配置、launch、校验代码和简要文档，以及本包的 `package.xml`、`CMakeLists.txt`、`config/`、`launch/`、`src/crop_floor.cpp` 和 `map/warehouse*.bt`、`map/waypoints.json`。并非本包所有旧实验脚本和数据都随仓库分发。

## 本机自行保留

- 不同步 `v2/outputs/`、轨迹 CSV/NPZ、日志、历史图表及工作区 build/devel。
- 本包 `.pcd`、`map/1goal.world`、旧 `script/`、旧 `data/` 及 USD/mesh 不跟踪；如果使用相关旧功能，需保留各机现有文件。
- 同级 `../robot_description/` 是独立 ROS 包；本包 URDF 引用其中 `meshes/cr5/`、`meshes/cr5_car/` 等资源，必须由每台机器已有部署提供。`../aws_robomaker_world/`、`../isaac_moveit_cr5/` 仅在对应旧仿真功能中另行提供，不在本仓库内；不宣称这些后端已验证。
- 必要依赖包括 Ubuntu/ROS Noetic、MoveIt/OMPL、octomap、相应 Python SciPy/NumPy/CVXPY 等；具体环境和本机 catkin overlay 需自行配置。本仓库不打包依赖或系统级 MoveIt 补丁。

## 路径及首次运行

`v2/configs/global.yaml` 的 `package://cr5_car_moveit/` 按当前包定位，普通相对路径相对 YAML 所在目录。地图仓库包含当前小型 `.bt`；ROS Master、可选 overlay、任务和输出按机器配置；不要提交凭据。PRP YAML 入口使用所在 catkin 工作区的 `devel/setup.bash`。旧非 YAML 入口的 `v2/scripts/env_noetic.sh` 仍有历史机器路径，非本轮承诺的跨机入口。

新 P 请求含工作空间时 `v2/adapters/capture_plan.py` 需要本机生成的 `v2/outputs/workspace_tools/workspace_probe`；它是构建产物，不随仓库提交。ROS/MoveIt 环境和 pkg-config 依赖就绪后，可在包根执行：

```bash
mkdir -p v2/outputs/workspace_tools
V2_PRP_CONFIG_MODE=1 bash --noprofile --norc -c 'source v2/scripts/env_noetic.sh; g++ -std=c++14 v2/adapters/workspace_probe.cpp -o v2/outputs/workspace_tools/workspace_probe $(pkg-config --cflags --libs moveit_planners_ompl moveit_ros_planning)'
```

该命令沿用现有构建方式，本轮未在第二台电脑实测。将不同机器的工作区各自构建后核对机器人模型和 PlanningScene，再运行 YAML；2026-11-19 在实验室电脑上已取得新 P 五目标数值及仓储离散碰撞基线；这不代表第二台电脑场景、ROS 或仿真已验。五目标结果超 30 MiB，当前 YAML 输出上限也不涵盖旧直调入口。详细限制参见 `CHANGELOG.md` 和 `v2/docs/V2_PRP_RUNBOOK.md`。


## 文档

本仓库跟踪 `CHANGELOG.md` 和 `v2/docs/V2_PRP_RUNBOOK.md`；仓库外的正式 `/home/robot/digitaltwin/项目报告.md` 与研究 evidence 仍由原位置维护，不通过本仓库同步。运行手册是发布时复制的快照；后续修改需同步更新本机和仓库副本。

## 两机增量同步约定（用户补充，2026-11-19）

- **不参与日常 Git 或 rsync 的静态资产：**两台机器已经各自具备 `robot_description/meshes/`、`robot_description/usd/`、`robot_description/env/` 以及大型 USD、mesh、贴图和环境模型。保留各机现有资源，不删除、不覆盖；今后确需变更时只单独指定受影响文件。无需 Git LFS，也不安排针对这些资产的 SHA-256 一致性检测。
- **持续同步范围：**实际使用的 V2 Python/C++ 源码、必要 YAML/launch/URDF/SRDF 文本配置、研究说明和项目开发指令；未来实际修改的 Isaac Sim 扩展 Python 源码也应纳入明确的同步清单。`Extensions/Yourextension/` 位于本仓库之外，不会因为推送本仓库自动同步；届时按实际路径和授权，单独跟踪或明确同步需要的 Python 文件，不连带 USD、贴图等资源。
- **必要运行输入单独确认：**`default_scene_snapshot.bin` 等确实影响规划的文件应按实际使用情况纳入必要清单，即使其为二进制也不能误判为历史实验数据；先核实两端文件位置、所属场景、当前引用及 Git 跟踪策略，再决定 Git 或明确指定文件传输。实验室已通过 PHG1 验收的场景快照和运行配置须保留，未经确认不覆盖。本次在本机 `zzs_ws/src/` 的文件名检索中未发现 `default_scene_snapshot.bin`，不能推断另一台不存在，也不能把历史运行快照自动当作默认快照。
- 历史轨迹、实验输出、构建产物和日志不随源码日常同步。`项目报告.md`、`AGENTS.md` 与研究说明可能位于本仓库之外；按实际需要明确选择小型文档同步方式，不整体同步 `research_management/evidence/`，不移动或覆盖正式主报告。
- **不迁移 Agent 状态：**用户说明实验室电脑此前仅使用网页端 AI，没有安装或运行本地 Codex Agent；不需要处理该机 Codex 会话或同步 `~/.codex/`，也不以配置 Codex 为 Git/SSH 前提。个人电脑的 Codex 历史独立保留，不删除。

### 下一轮工作顺序

1. 首先针对 `v2/scripts/env_noetic.sh` 做最小双机兼容修改：在不改变原有调用合同的前提下，核查工作区与 MoveIt overlay 的实际位置，提出可配置/包相对方式；保护已通过 PHG1 验收的实验室场景和配置。修改后只做直接相关的环境加载和入口检查，不运行完整 PHG1。
2. 不凭推测恢复 `ros_probe.py`、`scene_loader.py` 的历史版本；如实际报错，再根据当前源码、两端差异与错误证据决定是否修改。
3. 两端 SSH 连接建立后，在**不覆盖**对方的前提下比较所需源码与配置的实际差异；再按这份清单进行轻量 Git 同步。比较前保存两边未提交的独有修改；不使用强制推送、`reset --hard` 或直接覆盖。
4. 不引入 Git LFS、复杂分支审批流程或大量自动检查。本次仅调整方案，未修改 `env_noetic.sh`，未连接另一台机器、同步文件或执行 PHG1。
