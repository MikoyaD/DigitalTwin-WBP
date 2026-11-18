# V2 II-B 故障与修复记录

所有故障运行均保留在 `v2/outputs/runs/`，先看其 `result_summary.md`、`commands.log`、对应阶段日志和 `manifest.json`；`STATUS=PARTIAL` 不表示完整 G 验收成功。下面是本轮真实遇到的故障，不是理论排查清单。

| 现象与证据运行 | 根因/诊断 | 已实施处理及验证 |
| --- | --- | --- |
| `run_20261022T090027Z_7350c7d5`：`NETWORK_PERMISSION_DENIED` | 沙箱不能访问本机 ROS XMLRPC 端口 | 在获授权的普通宿主环境重跑；`run_20261022T092444Z_adb7f2f5` 冷启动 PASS。不要把权限错误误判成 MoveIt 缺失。 |
| `run_20261022T090650Z_966217e0`：专用 Master 已启动但 owner 校验失败 | `/proc` 环境的 `V2_RUN_ID` 判定与启动方式不一致 | 修正 V2 的进程标识/校验；仅用 `stop_experiment.sh` 停止自己进程，记录 `stop_5e93cb6e.json`；回归 `test_stop_unowned_has_no_kill`。 |
| `run_20261022T091003Z_2c13855e`：初次场景 `STATIC_OBSTACLE_MISSING` | 无界面 MoveIt 启动后仓储 `.bt` 尚未进入 PlanningScene | `scene_loader.py` 从仓储地图向本轮场景注入 OctoMap，随后探针查到 4,802,615 字节非空障碍；冷启动实例场景哈希 `ad46f519…`。不清空/放宽障碍来求 PASS。 |
| 同一运行：`world`/`odom` 框架检查失败 | MoveIt 返回的 PlanningScene 世界框架为 `world`，输入底盘使用 `odom` | 探针要求可核验 `world→odom` 变换、保存场景框架和快照；最终两条轨迹以完整 RobotState 检查。原运行进程由 `stop_f22e7e66.json` 记录停止。 |
| `run_20261022T091216Z_5df24d06`：第二父路径 `AMBIGUOUS_STATIONARY_MODE` | 真实 MoveIt 路径有 XY 静止而 theta 与关节同时变化的区段 | V2 明确记录 `SPIN_THEN_ARM` 顺序候选；保留端点和臂路点、全身状态重新碰撞验收；`run_20261022T091622Z_b9c07814` 两条最终 PASS，`test_ambiguous` 与全套回归通过。此策略并不宣称保留原始同步动作时间。 |
| `run_20261022T091809Z_3b6e8cf5`：子 run 的 `all` 在 planning 找不到 preflight | `parent_scene()` 只遍历父链，未检查本次 run 刚生成的 preflight | 修正 ancestry 包含当前 run；新增 `test_all_stage_uses_own_preflight_before_parent`；`run_20261022T091943Z_63f6eef7` 与冷启动实例全阶段 PASS。 |

日常诊断：

1. `ROS_DISTRO_NOT_NOETIC`：必须用 `run_experiment.sh`；它在干净环境按规定顺序 source 三个 setup。不要依赖有 ROS 版本选择菜单的交互式 `~/.bashrc`。
2. `UNOWNED_MASTER`：11321 端口由其他进程占用时，脚本不会接管。使用新的 V2 配置和空闲 `master_uri`，重新执行 `preflight`，不要终止别人的节点。
3. `STATIC_OBSTACLE_MISSING` / `UNKNOWN`：核查 `scene_probe.json` 的 OctoMap 字节、`scene_snapshot.bin`、TF、PlanningScene 服务和场景哈希。单独 `g` 的 collision `UNKNOWN` 是正常中间状态，需执行 `validate`。
4. `PARENT_SCENE_MISMATCH` / `MODEL_OR_SOURCE_CHANGED` / `CONFIG_CHANGED`：不要把不同场景、模型或配置的父路径拼在同一个 trial；创建新的运行链。
5. `G_NUMERICAL_FAILURE` 或碰撞 FAIL：保留父路径与失败结果，查 `g_*/result.json`、`validation_*.json` 的失败码和位置；不可削减障碍、调宽限值或覆盖 CSV 来制造成功。

短联调的两个成功任务只验证离线规划参考轨迹及离散样本。正式统计需另外定义重复试次、失败计数和随机种子控制；当前没有执行控制、真机或 Gazebo 动力学跟踪数据。
