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

该命令沿用现有构建方式，本轮未在第二台电脑实测。将不同机器的工作区各自构建后核对机器人模型和 PlanningScene，再运行 YAML；新 P→最终碰撞全链路尚待 ROS 实际验收。当前 YAML 输出上限不涵盖旧直调入口。
