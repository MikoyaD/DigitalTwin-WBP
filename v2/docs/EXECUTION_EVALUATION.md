# 2026-11-25 正式版补充

物理执行默认 `warehouse_physics_self_overlay.usda`；历史简化场景通过 `--stage .../warehouse_color_overlay.usda` 选择。接触事件见运行手册；场景对齐与碰撞边界见遗留问题清单。原有评价字段和历史基线保持不变。

# V2 P1 物理执行评价

120 Hz、PGS；每步共用仿真时间插值九维参考。参考底盘速度前馈和车体系位置/航向反馈经 Isaac DifferentialController 形成轮速命令，Isaac ArticulationController 分别驱动轮速与六关节位置。除一次初始化外不直接重设状态；实际关节速度为 API 原值。参数及临时地面、轮碰撞代理、低摩擦支撑记录于每次 metadata。连续碰撞与完整仓储碰撞尚未验证；此前 240 Hz 的物理约束和关节反馈一致性失败结论不因 120 Hz 数据消失。实际限制偏离是仿真实验性能指标，不能代替真实设备安全验证。

令实际减参考误差为 $e_i=x_i-x_i^r$，航向差按圆周归一化。对 N 个等步长状态，$\operatorname{RMSE}(e)=\sqrt{N^{-1}\sum_{k=1}^{N}e_k^2}$；底盘 XY 误差是 $\sqrt{e_x^2+e_y^2}$，位置及航向给出 RMSE、绝对峰和终点值；六关节给出各自 RMSE、绝对峰和终点值。Link6 位置误差为同一 world 坐标系下的位置向量差范数，姿态误差为 $\lVert\operatorname{Log}((R^r)^T R)\rVert$，分别给出 RMSE、峰值和终点。

各维速度、加速度、jerk 分别报告参考与实际绝对峰；速度与加速度报告误差 RMSE，加速度与 jerk 还报告各自 RMS。实际 $a=\nabla_t v_{\rm API}$，$j=\nabla_t a$；参考的 $v,a,j$ 来自原 P1 对应列的时间插值。差分算法和物理单位见数据字典，未经滤波，不声称连续时间真实加速度。

对实际分量 $x(t)$ 和有效规划上限 $L$，定义超额 $d(t)=\max(|x(t)|-L,0)$；峰值倍率 $\max |x|/L$，最大超额 $\max d$，超限时间比例 $\int\mathbf1_{d>0}\,dt/T$，最长连续超限时长按逐步布尔区间累计，超限积分 $\int d\,dt$。采用轨迹所属任务的 effective_config 中的逐分量速度、加速度限制；时间积分使用相邻采样时间的左端点近似。参考规划限值不等于实机安全限制。

底盘横向速度 $v_\perp=-\sin\theta\,v_x+\cos\theta\,v_y$，报告绝对峰、RMS、$\int |v_\perp|dt$；两轮报告实际角速度减目标的 RMSE 与绝对峰。不把 $v_\perp$ 称为轮胎滑移率。结果分别存入 `metrics.json` 的 tracking、motion、constraint_deviation、nonholonomic；对应图表在 figures。平移和角度分维计算、保留各自单位。

## 多目标已记录事件

当任务结果提供 `motion_end_s`，取参考时间轴上最近的实际采样 $k=\arg\min_i|t_i-t_{\rm event}|$，记录该时刻 Link6 的轨迹参考位置和姿态误差以及采样时差；未记录事件不推测。该值属于跟踪误差，不等于末端对任务目标的达成误差。默认不绘图；需要时使用数据字典中的显式 `--figures`。

## 派生数据存储

默认保留原始每物理步状态及正式 metrics；derived.csv 按数据字典中的 `--save-derived` 生成，指标公式与数据密度不变。真实多目标样本离线抽样 60/30 Hz 时，位置 RMSE 接近，但加速度和 jerk 差分峰明显变化，因此降采样仅适合标注清楚的展示副本，不替代原始状态。
