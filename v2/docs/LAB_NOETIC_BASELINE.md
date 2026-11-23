# 实验室 Noetic / Isaac 基线（2026-11-24）

该分支保存实验室经使用的 V2 源码、系统 Noetic 工作区环境适配，以及独立 Isaac 物理驱动实验脚本。`isaac_physical_drive_probe.py` 为研究实验，不是已验收的 P1 物理控制器。已有 Isaac 九维运动学回放和自动发现功能沿用父提交。

此仓库不是包含大型资源的自包含安装包。实验室运行仍依赖：Ubuntu 20.04、ROS Noetic/MoveIt、`/home/robot/digitaltwin/zzs_ws/devel/setup.bash`，工作区内 `robot_description` 和 `aws_robomaker_small_warehouse_world`，项目配置 `v2/configs/default_scene_snapshot.bin`，以及本机输出和 `workspace_probe`。Isaac 4.2 回放还依赖 `v2/assets/isaac_import_20261124/warehouse_v3.usd`、其机器人及仓储网格引用；物理短测当前使用额外颜色覆盖层 `warehouse_color_overlay.usda`。这些本机快照、USD、mesh 和实验输出未包含在 Git 提交内，须另行保留及按原引用路径提供。

本机 `v2/outputs` 是指向旧工程备份输出的符号链接；现有已验收 P1 位于 `/home/robot/digitaltwin/v2_single_retry_20261124_014331/outputs/`，不随 Git 同步。请勿以没有这些外部依赖的干净克隆冒充完整运行环境。可选自定义发现根通过 `V2_DISCOVERY_ROOTS` 指定。

先 source `/opt/ros/noetic/setup.bash`、工作区 devel setup，再从正式包运行 `./v2/scripts/view_v2.sh`。本机物理实验的条件、结果与限制见 `/home/robot/digitaltwin/research_management/evidence/ISAAC_PHYSICAL_DRIVE_20261124_01/REPORT.md`；此证据不随 Git 同步。未重新运行规划或完整回放。
