# V2-PRP 版本记录

## v2-prp-five-goal-20261119 — 2026-11-19

五目标研究基线，链路为联合九维 IK → RRTstar P → Signed R → A→C P0 → formal P1 → 末端 FK 及完整机器人离散碰撞。单目标、五目标以及保存 P/R 输入由 `v2/scripts/plan_v2.sh` 和 `v2/configs/global.yaml` 调用；具体命令、场景及回放边界见 [运行手册](v2/docs/V2_PRP_RUNBOOK.md)。

- Signed R 以显式车体航向与有符号纵向进度表达前进/倒车。五目标路径固定 profile 的 22 控制点 ρ-QP 不可行时可尝试 36；二者均精确不可行时，可选择已集成的有界 θ–ρ 联合恢复和真实积分闭合。联合恢复目前针对 ADJUST→MOVE→ADJUST；不能推广为任意 chart 全部可行。单目标 YAML 与直接 `--from-p` 默认不自动启用多目标回退。
- 2026-11-19 的五目标新 P 基线总参考时间 **193.672418 s**（含五次各 1 s 驻留）。五段最终 Link6 FK 和数值检查通过；五段同场景完整机器人离散碰撞共 **737,028 状态**通过，拼接新增过渡另查 **12 状态**通过。这是离线参考轨迹和离散检查，不是连续碰撞认证、Gazebo 控制跟踪或实机执行。
- 采用 `V2_NINE_DOF_VA_ONLY`：九维各分量速度限值 0.5、加速度限值 1.0（按 m、rad 对应单位）；底盘额外合速度/轮速研究限制当前未启用。jerk 5 是软参考，不是硬约束。
- 已知限制：该五目标完整无损高密度结果目录 **74.809 MiB**，未满足 30 MiB 目标，正式命令因此退出 1；完整五目标 NPZ 暂无正式一键 CSV/预览适配。Gazebo/Isaac 同步整机执行未完成。部分直调入口存在旧 overlay 绝对路径；`V2_PRP_MAX_ATTEMPTS` 未被 shell 清洁环境透传，跨进程重试不自动累计。相邻 MOVE 直接换向及一般多 chart 恢复仍有限制。
- 实验文件未纳入 Git。本机证据索引（非仓库内文件）：`/home/robot/digitaltwin/research_management/evidence/V2_PRP_FIVE_FRESH_20261119/`、`/home/robot/digitaltwin/research_management/evidence/V2_PRP_BASELINE_REVIEW_20261119/`；高密度结果位于 `/home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit/v2/outputs/prp_yaml/PRP-MULTI-20261119-020714-832492/`。根目录 `项目报告.md` 是本机独立维护主报告，不包含在此仓库。
