# 遗留问题与后续工作（2026-11-25）

现象、原因、证据、影响与建议均限定于原记录。优先级是研究安排建议；本轮未实施专项修复。

## MoveIt 与 Isaac 场景配准

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：近单位 XY/yaw，留出 RMS 0.036593 m；高度未独立确认。
- 已确认与未确认：几何表达不同是事实，剩余坐标误差的具体来源未定。
- 已尝试与影响：已做非对称货架点到面配对，单位阵留出 RMS 比拟合略优；该结果影响障碍间隙和正式安全判断。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_GEOMETRIC_ALIGNMENT_20261124_025616/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：核对共享地标与世界变换，再在典型障碍附近验证。

## OctoMap 与障碍物对应

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：23 个可视网格有碰撞；OctoMap 为 0.05 m 体素。
- 已确认与未确认：网格和占据表达不同；缺少逐障碍完全对应。
- 已尝试与影响：已构建 23 个静态碰撞网格并用真实货架正例验证；无法从少数正例推断 OctoMap 全部占据体对应。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_ISAAC_WAREHOUSE_PHYSICAL_SCENE_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：逐对象空间占据与安全间隙对照。

## 地面和根节点高度

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：起点地面 0.034224–0.034362 m，当前平面 z=0.034224。
- 已确认与未确认：平面为实验近似，全程高度未确认。
- 已尝试与影响：已在起点做地面射线，轮/支撑保持稳定；多点地形与规划高度未测，可能影响贴地及障碍净空。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_EXECUTION_CAPABILITY_20261124_030928/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：多点地面射线与机器人轮底位置比较。

## Link6 FK 与真实坐标

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：四姿态 FK 最大位置误差 4.58e-7 m。
- 已确认与未确认：已验证局部运动学，世界高度依赖临时基准。
- 已尝试与影响：已比较零位、起终点及非对称四姿态，局部误差小；目标事件世界高度偏置会影响绝对任务达成评价。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_ALIGNMENT_KINEMATICS_20261124_024913/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：检查多位置统一末端标架和世界高度。

## URDF 与 PhysX 碰撞几何

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：轮为圆柱代理，支撑低摩擦，仓储为视觉静态 mesh。
- 已确认与未确认：碰撞形状和摩擦模型不是标定的硬件。
- 已尝试与影响：已验证圆柱轮和真实货架接触，避免误把 mesh 视作已标定硬件；形状差异限制安全外推。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_ISAAC_WAREHOUSE_PHYSICAL_SCENE_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：逐连杆尺寸、姿态及实际碰撞范围对照。

## dummy_link 与允许碰撞

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：75 对已映射，12 对含未导入刚体 dummy_link。
- 已确认与未确认：已知 USD 缺该刚体；其实际语义与潜在几何覆盖待复核。
- 已尝试与影响：已依据 SRDF 映射存在刚体的允许对、测试非允许 Link2/car_link 正例；dummy_link 对仍需语义确认。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_ISAAC_WAREHOUSE_PHYSICAL_SCENE_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：核查 URDF/SRDF 和 USD 固定合并、正反例。

## 初始化触地

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：world.reset 初始时 car_link 于 t=0 触地，轨迹期间未记录。
- 已确认与未确认：发生于初态设置之前；是否影响其它起点未确认。
- 已尝试与影响：已将 t=0 事件与轨迹期分开，运行期未见该事件；尚未修改 reset 顺序，影响起始冲量解释。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_ISAAC_WAREHOUSE_PHYSICAL_SCENE_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：记录 reset 前后位置和冲量，评估初始化顺序。

## Nested C 候选质量劣化

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：30 样本中 1 项后处理目标劣于 A，正式规则仍选 C。
- 已确认与未确认：现行代码无质量回退；不是所有样本均劣化。
- 已尝试与影响：已做 30 对 A/C 比较并保留劣化样本，未修改选择规则；不能声称优化目标每次改善。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_PRP_PARAMETERIZATION_ABLATION_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：配对质量回退实验，不改本版算法。

## P0 求解器迭代上限

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：30 组 A fast、A smooth、C 均达迭代上限。
- 已确认与未确认：终止不等于不可行；预算是否限制收益未证实。
- 已尝试与影响：已对最终候选做独立数值与离散碰撞验收；所有求解器达到上限并不意味着收敛。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_PRP_PARAMETERIZATION_ABLATION_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：小规模固定几何迭代预算对照。

## 原始候选 v/a 超限与缩放

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：C 30/30 原始候选至少一项超限，缩放后通过数值验收。
- 已确认与未确认：后处理安全缩放是正式实现；对收益的影响需量化。
- 已尝试与影响：已在不放宽限制下安全缩放并验收；缩放使优化器原始目标与实际执行收益不同。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_PRP_PARAMETERIZATION_ABLATION_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：记录缩放前后目标和峰值。

## Signed R 曲率低速奇异

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：局部曲率峰受低参数速度数值影响。
- 已确认与未确认：峰值含数值尺度效应；真实几何尖峰尚未区分。
- 已尝试与影响：已记录逐点曲率与参数速度，未进行专项修复；异常峰不可直接作为物理曲率结论。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_SIGNED_R_CONTROL_AND_STORAGE_OPTIMIZATION_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：检查参数速度、几何曲率及重参数化一致性。

## 规划失败与有界恢复

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：R QP infeasible、MoveIt -2、组分碰撞失败均有记录。
- 已确认与未确认：36 控制点与有界恢复改善样本，但不保证所有问题可解。
- 已尝试与影响：已复用 36 控制点及 theta-rho 有界恢复，三次单目标获 P1；样本少且 MoveIt -2 可继续出现。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_SINGLE_FAILURE_AND_STORAGE_RESOLUTION_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：新增独立任务并逐阶段统计失败及重试。

## 关节位置与速度积分残差

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：240 Hz 全程最大约 0.07874 rad，PGS 12 秒改善。
- 已确认与未确认：引擎内部机制尚未证实；120 Hz并未消除全部残差。
- 已尝试与影响：已比较 TGS/PGS，PGS 在短时降低残差；全程残差仍在，影响速度积分解释。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_ARM_ACCELERATION_DIAGNOSIS_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：固定物理条件记录位置与速度，单独评估反馈语义。

## 速度加速度 jerk 峰

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：实际反馈局部超规划参考；高阶差分对步长敏感。
- 已确认与未确认：仿真响应和数值求解共同影响，具体贡献未完全分离。
- 已尝试与影响：已比较 120/240/480 Hz 和离线降采样；实际加速度与 jerk 属反馈派生量，不能用滤波掩盖原始值。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_ARM_ACCELERATION_DIAGNOSIS_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：在同轨迹同条件报告原始与固定尺度估计。

## 原始任务目标达成误差

- 状态：未解决/覆盖不足；优先级：中。
- 实际现象：已保存目标时刻 Link6 对参考的跟踪误差。
- 已确认与未确认：轨迹跟踪误差不等于相对原任务目标的误差。
- 已尝试与影响：已计算各目标时刻 Link6 对参考轨迹误差；不能据此直接声称原目标位姿误差。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_PRP_EXISTING_TRAJECTORY_EVALUATION_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：统一 world/Link6 标架后计算事件目标误差。

## 仓储物理覆盖数量

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：新碰撞场景仅两条完整轨迹；用户已报告 GUI 全程正常。
- 已确认与未确认：实验覆盖有限，不推断所有任务安全。
- 已尝试与影响：已完整执行单目标及五目标各一条，用户观察正常；不足以证明其他轨迹与障碍不会接触。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_ISAAC_WAREHOUSE_PHYSICAL_SCENE_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：扩展独立轨迹并记录障碍/自碰撞。

## 材质与纹理

- 状态：未解决/覆盖不足；优先级：低。
- 实际现象：颜色覆盖可用；Gazebo 原始纹理未完整恢复。
- 已确认与未确认：货架/地面材质分组未完整建立。
- 已尝试与影响：已保留颜色覆盖层和局部 UV 调查；视觉风格未还原，影响展示而非已测数值。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_EXECUTION_CAPABILITY_20261124_030928/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：任选典型对象核对 UV、分组及纹理路径。

## 硬件物理参数

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：车轮、接触和驱动为仿真试验参数。
- 已确认与未确认：缺质量/摩擦/力矩等独立设备标定。
- 已尝试与影响：已有临时地面、轮代理和低摩擦支撑对照；参数不代表真实设备，不能支持硬件性能结论。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_PHYSICAL_DRIVE_20261124_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：获得硬件资料并进行基础辨识。

## 离散与连续碰撞

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：90/90 候选导出状态及中点通过离散检查。
- 已确认与未确认：采样之间仍有未覆盖空间。
- 已尝试与影响：已检查 90 项候选导出状态和线性中点；该证据无法覆盖中点之间的连续轨迹。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_ISAAC_WAREHOUSE_PHYSICAL_SCENE_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：按速度及净空选择连续碰撞或有界细化方法。

## 样本泛化

- 状态：未解决/覆盖不足；优先级：高。
- 实际现象：33 父路径和 30 组消融来自既有重复目标。
- 已确认与未确认：不是独立随机总体，不能推断总体成功率。
- 已尝试与影响：已保留失败记录、版本与重试分母；现有成功几何及重复目标不足以推断总体成功率。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/V2_SIGNED_R_CONTROL_AND_STORAGE_OPTIMIZATION_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：预注册独立目标/随机种子并记录全部失败。

## 统一全身动力学控制

- 状态：未解决/覆盖不足；优先级：低。
- 实际现象：当前同步差速反馈与臂关节位置驱动。
- 已确认与未确认：无全身耦合动力学或力矩优化；属于可选研究方向。
- 已尝试与影响：已实现同一仿真时间下底盘反馈与六关节驱动；尚无统一耦合动力学或力矩最优控制。 详见 [原始报告](/home/robot/digitaltwin/research_management/evidence/ISAAC_ARM_ACCELERATION_DIAGNOSIS_01/REPORT.md)；该问题限定相关执行性能或论文结论，不自动否定其余已验收结果。
- 后续思路与最小验证：视论文范围决定是否建立动力学对照。
