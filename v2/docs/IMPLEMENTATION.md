# Geometry-First V2：阶段I数值核心

2026-10-22（北京时间）。仅实现数据合同、解析数学基础、静态参数读取和独立验收。没有几何拟合优化器、SOCP/SCP、ROS节点或launch。原版文件及旧研究报告只读。

## 目录与运行

ROS工作区为 `/home/robot/digitaltwin/zzs_ws`；ROS包为 `src/cr5_car_moveit`。本目录是独立Python包 `v2`，没有修改catkin配置。直接在ROS包目录运行，不需要source ROS：

```bash
cd /home/robot/digitaltwin/zzs_ws/src/cr5_car_moveit
PYTHONDONTWRITEBYTECODE=1 python3 -B -m v2.tests.run_checks
```

该入口运行unittest、AST语法检查、6个生产模块导入检查、11个旧输入文件的SHA-256保护检查；每次用新的UUID文件保存结果，不覆盖前次输出。不生成pyc。现有环境Python 3.8.10、NumPy 1.17.4、SciPy 1.3.3、PyYAML 5.3.1；采用标准库unittest，不新增依赖。

| 文件 | 内容 |
|---|---|
| core/contracts.py | 六个指定合同及StateSchema、PathJet、Quantity、PhaseType、Status；错误带code；严格SI与九维顺序 |
| core/math_tools.py | 五次B样条基与0–4阶导、全身参数、XY多项式极值检查、航向0–3阶导、链式导数、四阶段数学工具、差速轮运动学、时间缩放 |
| core/limits.py | 只读解析URDF/YAML/SRDF，形成带来源和hash的LimitSet；显式research参数与请求scaling |
| core/interfaces.py | GeometryBuilder、TimeParameterizer、JerkRefiner、InputAdapter的Protocol，无算法实现 |
| validation/numerical.py | 限位、限速、限加速度、非完整残差、rest边界、分段接缝、实际时间积分一致性、分组jerk |
| validation/collision.py | 请求/结果合同与UnknownCollisionChecker，无PlanningScene实现 |
| tests/support.py | 显式合成解析轨迹/限值夹具，不是机器人实验数据 |
| tests/test_math.py | 26项数学和退化测试 |
| tests/test_contracts.py | 14项合同、单位、元数据、模型参数测试 |
| tests/test_validation.py | 22项独立验收与反例测试 |
| tests/run_checks.py | 测试及不覆盖证据留存入口 |
| docs/source_protection.json | 11个关键旧文件原始hash，包括原设计报告/主报告/旧QP/配置 |
| docs/static_limitset_research_example.json | 从实际模型生成的独立参数快照，研究限值逐项标记，非运行时或硬件参数证明 |
| docs/test_run_*.json | 每次完整测试记录、源代码hash、版本和保护检查结果 |

## 合同和调用约定

列顺序固定为 `(world_car/x, world_car/y, world_car/theta, joint1,...,joint6)`，位置单位 `(m,m,rad,...,rad)`，其他导数使用对应SI单位，时间单位必须为s。未知字段不能填0；无时间的PathRecord明确用None。不同时间网格的底盘/臂流由 `merge_aligned_streams` 拒绝，不自动插值或按索引拼接。关节顺序变化须由后续适配器显式重排。

`TrajectoryRecord`保存一个阶段的完整九维q/v/a/j和全局实际时间；多阶段使用有序记录集合。每个阶段保留接缝两侧的导数，`validate_junctions`检查后，后续导出器才能消除重复接缝时间。本轮不实现最终ROS导出。首尾任务姿态允许mod 2π比较，阶段接缝不允许隐藏2π分支跳变。

`PhasePath.evaluate(s)`提供PathJet(q,qs,qss,qsss)，检查TRANSLATE正向切向、SPIN固定XY/臂、ARM_ONLY固定底盘、HOLD全状态不动。它不负责构造/优化几何。测试验证了HOLD→SPIN→TRANSLATE→ARM_ONLY保持九维状态及速度、加速度连续。

典型数学流程（仅API说明，未实现优化链）：

```python
s, sd, sdd, sddd = quintic_time_jet(local_times, duration)
jet = phase.evaluate(s)
velocity, acceleration, jerk = chain_rule(jet, sd, sdd, sddd)
result = validate(record, limits, requested_start, requested_goal)
```

`validate`默认要求rest-to-rest。单独测试非零边界的限值时可显式传 `require_rest=False`；后续默认流程不能因此绕过任务边界。位置/导数/单位错误在构造时抛ContractError；验收发现的违约保存在ValidationResult。`numerical_status=PASS`仅表示检查覆盖的采样参考通过；碰撞状态始终UNKNOWN，故本阶段整体状态不会PASS。

## 数学一致性和设计细化

1. 几何degree=5且内节点单重，提供0–4阶解析导数。航向实现设计G3的C/D、C'/D−CD'/D²及三阶公式；独立抛物线解析解、一般四阶XY多项式和圆弧测试交叉验证。没有把有限差分用作生产导数。
2. T1链式导数直接实现；独立使用q(s)=s³、s(t)=t²得到q(t)=t⁶验证v/a/j。时间缩放保持时间原点，v/a/j分别除以γ、γ²、γ³。
3. 五次时间多项式数学内容不变，但采用对称求值 `F(u)=1−F(1−u)` 及因式形式 `F'=30u²(1−u)²`、`F''=60u(1−u)(1−2u)`，防止端点附近抵消产生负速度。零端速度/加速度不等于零端jerk。
4. 对任意输入（包括有侧滑的错误轨迹），车体系前向速度导数为 `a_forward=cosθ ax+sinθ ay+ω e_lat`。不能在独立验收器中预先假设e_lat=0省略最后一项。满足非完整约束时退化为原设计模型。
5. XY正则性对每个样条片段的ρ²驻点及端点求最小值；病态根返回UNKNOWN。它是双精度数值审计，不是区间算术形式证明。`heading_jet`检查采样顺序、零切向和过大相邻转角；稀疏数据无法证明未漏掉完整旋转，阶段II必须构造自适应全路径角度分支表，不能拿单次局部unwrap替代。
6. 当前返回q到三阶路径导数，足够设计中SOCP的q_s/q_ss和SCP-J2的q_sss；原始八维样条到四阶可用于航向三阶。未提供航向四阶，也未实现需要它的直接解析jerk Jacobian。
7. 原设计对碰撞和连续约束的门槛未降低。这里仅采样数值验证；真实时间累计梯形积分检查有离散误差，必须结合加密收敛，不构成采样间的连续约束证明。对所有阶段包括端点计算jerk，不裁剪、不平滑。

## 限值来源和未知事项

读取config/gazebo_combined_robot.urdf、joint_limits.yaml、combined_robot.srdf。轮半径0.06m，插件轮距0.3935m；轮joint几何轮距0.395m，差异已记录。两轮parent均car_link、x=0且y对称；SRDF的planar world_car也连接car_link，因此该模型中的XY参考与轮轴中点投影一致。运行时world/odom TF、插件控制符号及实际反馈均未验证。

六臂位置[-3.14,3.14]rad，速度取URDF和YAML较小值再乘显式请求比例；加速度来自YAML乘比例。底盘逐轴v/a与研究设定的XY范数上限分别保存、分别检查，不混同。研究示例为XY速率0.5m/s、XY/纵向加速度0.5m/s²、两轮角速度0.5/0.06rad/s、轮角加速度0.5/0.06rad/s²、自旋jerk2rad/s³，全部标记research_limit，不能写成厂家能力。

轮速度必需；轮加速度可缺省但检查明确UNKNOWN；SPIN缺自旋jerk参数时不能通过相应验收。该读取器不把YAML的jerk声明当作旧TOTG已限jerk，也不把插件wheelAcceleration=0或torque当作真实轮限值。现阶段没有XY工作空间位置盒，报告其未知，不捏造边界。

真实ROS接口仍需：同一trial的完整request/response、adapter阶段、实际planner、request scaling、原始消息/hash、RobotModel有效限值、world/odom TF、完整PlanningScene含ACM和附着物/冻结关节、seed可控性、全部失败试次及单调墙钟计时。原始joint与multiDOF若时刻不同，须明确重采样及误差规则；本轮只拒绝，不猜测。

## 实际测试与开发失败记录

最终 `test_run_ad1ca01478f3.json`：**62项通过，0失败、0错误、0跳过**；15个Python文件语法通过，6模块导入通过；11个旧输入hash未变，无rospy/moveit_commander/roslaunch导入。运行时0.162秒仅为合成单元测试耗时，不是优化计算耗时。

开发期首次命令在包目录下使用了重复包前缀，导致两个测试文件未创建；该次只发现24项数学测试，23通过、1失败。失败为错误要求四阶导中心差分误差继续按h²收敛。实测误差5.52e−9与1.23e−8已处于浮点量级；三阶导是分段二次多项式，其中心差分在片段内理论精确。修正为前三阶检查收敛、四阶检查绝对误差<1e−7，并保留s⁵四阶导独立解析测试（2e−12容差）。未修改被测数学公式来迎合该测试。

随后完整56项通过（test_run_17f591863038.json）；新增端点稳定性与跨四阶段接缝后61项通过（test_run_dbccf805fa00.json）；补齐缺失schema拒绝测试后最终62项通过。全部历史完整结果保留，无未解决关键数学失败。没有机器人/场景性能实验。

## 阶段II边界

阶段I数值门槛通过，可在另行授权后开始阶段II编码。建议新增 `geometry/spline_builder.py`、`geometry/phase_segmenter.py`、`geometry/heading_lift.py`、`timing/feasible_profile.py`、`adapters/moveit_input.py`、`adapters/planning_scene_checker.py`、`tests/test_geometry.py`、`tests/test_scene_contract.py`。前四者分别负责真正的几何拟合、退化分段、全路径航向分支、低复杂度全身定时；后两者需要真实MoveIt场景/消息接口。此处仅列文件，未创建、未实现。

数值模块测试不需要ROS；真实碰撞与同条件输入联调需要单独批准启动MoveIt/场景服务。没有真实轮硬件上限不阻塞明确研究限值的仿真参考开发，但阻塞硬件合规结论。阶段II尚未启动，正式实验、SOCP/SCP及论文均不在本轮范围。
