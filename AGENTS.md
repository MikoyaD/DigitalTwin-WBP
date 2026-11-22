# V2-PRP 接手约定

如果工作区存在 `/home/robot/digitaltwin/AGENTS.md` 和唯一正式 `项目报告.md`，先阅读其规则与最新维护日志；仅克隆本仓库时，从 `v2/docs/AI_HANDOVER.md`、`v2/docs/V2_RELEASES.md`、`v2/docs/V2_PRP_RUNBOOK.md` 开始，再读任务对应的源码与原始报告。仓库外主报告和 evidence 不随 Git 自动同步。

保护已有源码修改与原始实验轨迹；明确区分源码、历史数据、用户观察、推断和本轮复现。修改规划、执行仿真及 Git 操作遵守当前用户授权。九维统一规划不能表述为先规划底盘、再独立规划机械臂；离散碰撞不等于连续碰撞；Gazebo 模型位姿回放不等于差速动力学闭环或实机。

RViz、Gazebo、Isaac Sim 等 GUI 的视觉、画面和人工交互验收由用户本人完成；AI 负责源码、命令、ROS 状态、数值及日志，不以截图或窗口视觉检查作为前置条件，也不绕过 GUI 授权。不修改、不自动同步大型 mesh/USD/场景资产或历史输出；两机同步边界见 `SYNC_FILES.md`。
