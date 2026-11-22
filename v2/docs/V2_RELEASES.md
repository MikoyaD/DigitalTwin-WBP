# V2-PRP 版本与证据

## 命名

Git 提交、分支、标签标识源码快照；`v2-prp-*` 是项目阶段标签，不是每次实验目录的版本号。V1 是历史 ROS/MoveIt 与 Gazebo 工具；V2 是九维规划研究代码。V2-PRP 是目前 P（Parent Path）→ R（Reconstruction，历史 H）→ P0（历史 G0）→ P1（历史 G1）的正式链路；旧 PHG1 是历史命名，不能因此改变旧实验的算法版本。

## 已核实的历史

|标识|含义与证据|
|---|---|
|`756fabe00a084b9c7eb2ec31cd36b96189a6adf8`|本机保留的初始 V2-PRP 源码基线 commit；不在当前 master 的祖先链，不能假装是当前可从远程克隆取得的祖先。|
|`6217086393862e148984f37b913115367598a9d6`|当前 master 历史根：V2-PRP 规划源码与配置基线。|
|`9e10a4bef580b8658b00d474979960e57f1589cc`|五目标研究基线；注解标签 `v2-prp-five-goal-20261119` 指向此提交。五段数值与离散碰撞通过，容量目标未通过，非全条件 PASS。|
|`4b4f49b2ee2902f3e97aa7c0bb220d6f045275c8`|Gazebo 单段 CSV 与实际状态 RViz 联合展示整合。|
|`v2-prp-gazebo-display-20261122`|交互展示及交接文档阶段；以 `git show` 为该标签的实际源码基准。|

本仓库远端 `origin=git@github.com:MikoyaD/DigitalTwin-WBP.git`，使用 `master`。阶段参考：五目标目录 `v2/outputs/prp_yaml/PRP-MULTI-20261119-020714-832492/` 及仓库外 `research_management/evidence/V2_PRP_FIVE_FRESH_20261119/`；数值/离散碰撞不等同连续碰撞或真机结果。Gazebo CSV 及地图时序见 `V2_GAZEBO_CSV_REPLAY_20261120/`、`V2_GAZEBO_MAP_DELAY_20261120/`。这些输出与证据不随本仓库 clone。

推荐使用本分支最新提交，执行 `git log --oneline --decorate -5` 和 `git status` 确认现场状态。只读历史可 `git show v2-prp-five-goal-20261119:v2/configs/global.yaml`；需要在历史版本做独立工作时先保存现有修改，再 `git switch --detach v2-prp-five-goal-20261119`，完成后 `git switch master`。不要移动标签或覆盖未提交修改。

## 2026-11-22：CSV 输出与历史扫描后续变更

在 `v2-prp-gazebo-display-20261122` 之后，PRP 正式 YAML 新实验统一时间、任务、实际链路与规划器命名；扫描兼容旧 `V2-*`、`PRP-*` 及 `prp_yaml*` CSV，按三类显示与碰撞状态排序。新多目标将拼接数组写入 `task/trajectory.csv`，CSV 不因容量预算被删除。用户将 NPZ 标为废弃输入，但当前规划内核仍使用已有 NPZ 中间数据；不能将其写成已彻底去除。用户终端有旧五目标 CSV 225.725087 s 回放完成证据。本次更改的实际提交以 `git log` 为准，未移动五目标与 Gazebo 既有标签；未重新运行新五目标完整链路。

## 2026-11-22：真实新五目标 CSV 试运行

运行源码基线 `bc0f65399d1ee2f73faba615cdcbcc5e86edbd25`；这是源码提交，不是任务输出名称。真实目录 `V2-20261122-185203-MULTI-IKPRP-RRTstar` 的第 1 目标六条新 P 经 R/P0/P1 数值与末端 FK，但碰撞第一批均达到 60 s 时限并返回 UNKNOWN；其余目标、任务 CSV、绘图未产生。正式 PARTIAL / 退出 1，目录 127.285 MiB 超预算。此前 193.672418 s 的 2026-11-19 数值和离散碰撞完整结果仍为历史基线。证据位于仓库外 `research_management/evidence/V2_PRP_FIVE_CSV_ENDTOEND_20261122/`。

## 2026-11-22 固定候选碰撞与存储迁移（部分完成）

固定 attempt_03 实际 103632 状态离散碰撞 PASS；R 几何 4463 状态仅作非等价研究对照。正式分批与 UNKNOWN 处理改进，50 Hz 目标的 P1/整体 CSV 源行选择接入。零新 NPZ、P0 存储和历史高密 CSV 处理尚未完成；已有 NPZ 保留。证据 `research_management/evidence/V2_PRP_STORAGE_COLLISION_OPT_20261122/REPORT.md`。

## 2026-11-22 检查点与 P0 存储（部分完成）

原 attempt_03 独立碰撞 PASS 已可由多目标检查点读取器消费，目标2规划前的准备测试通过。历史 R04 P0 CSV 由 101743 降至 3864 行，原验收与 NPZ 保留。正式 P0 CSV 输出接入紧凑选样；零 NPZ 数据链、事件直接传递和正式链实跑仍未完成，不能称为续跑就绪。

## 2026-11-22 零 NPZ 数据接口及续跑（部分完成）

新 P→R→P0→P1→原密度碰撞实际第2段 PASS，新任务未生成 NPZ；旧 NPZ 只读。第3段六次预算未通过，第4/5段未执行，完整拼接与临时数据清理未实测。证据 `research_management/evidence/V2_PRP_ZERO_NPZ_FIVE_RESUME_20261122/REPORT.md`。

## 2026-11-22：R 预检与五目标续跑

源码以实际本次 Git 提交为准，历史五目标标签不移动。前两段采用已验检查点，第3–5段实际新规划，完整任务 150.166589 s，数值、目标 FK 与 591637 次分段离散查询 PASS；紧凑任务 CSV 12951 行。目录 34627584 B 超预算，退出码 1。R 几何预检前置并复用相同几何/场景结果；新任务未生成 NPZ，新组分密集临时数据已清理。删除 14 个历史 NPZ（包含仅存旧完整任务密集 NPZ），保留其研究报告；其密集整体数据不再可直接恢复。详细证据 `research_management/evidence/V2_PRP_FIVE_RECOVERY_STORAGE_20261122/REPORT.md`。
