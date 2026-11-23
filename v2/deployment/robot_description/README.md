# robot_description 外部包恢复资料

这里是本机相邻 ROS 包 `robot_description` 的 `package.xml` 和 `CMakeLists.txt` 原样文本备份，不是 catkin 工作区内另一个有效包；不要直接在 `cr5_car_moveit` 中构建它。当前 `config/gazebo_combined_robot.urdf` 才是 V2 联合 launch 所加载的机器人 URDF，网格通过 `package://robot_description/...` 解析，因此目标机器还需已有同名 catkin 包及其 mesh 资产。

若目标机已经有该包，不覆盖；若仅有资产且缺少包定义，先核对资产相对目录（含 `meshes/` 和供 CMake 安装规则使用的 `urdf/`），按需将这两个文件复制至工作区 `src/robot_description/`，再按运行手册构建工作区。本备份不含 mesh、USD、环境或旧 URDF；ROS 包发现及模型加载需在目标机验证。仓储 Gazebo world 另需 `aws_robomaker_small_warehouse_world` ROS 包。
