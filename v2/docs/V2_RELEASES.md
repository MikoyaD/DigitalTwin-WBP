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
