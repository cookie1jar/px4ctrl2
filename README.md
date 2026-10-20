# px4ctrl2

ROS 2 外部位置控制包。接收里程计和位置参考，生成目标姿态及归一化推力，经 MAVROS 发送给 PX4，由 PX4 完成底层控制。

支持已适配到机体的 VIO/LIO 定位、RViz 三维目标和外部 EGO-Planner 轨迹。

基于 ROS 1 px4ctrl 的提交 `7f3b0c8559d1d3569f92397ad35d38408bbd1ef5` 迁移，沿用 GPL-3.0。控制核心保留原版位置/速度反馈、小角度姿态求解、姿态参考对齐及延迟推力映射辨识。

## 文档

- [文件架构与模块说明](ARCHITECTURE.md)：完整目录、文件职责、数据流、状态机和接口边界。
- [仿真启动与起降检查](STARTUP.md)：PX4/MAVROS 手动启动及控制器检查流程。
- [更新记录](CHANGELOG.md)：近期功能与验证范围。

## C++ 源码组织

include/px4ctrl2 仅声明类型和接口；实现位于 src：controller.cpp 负责控制数学，freshness.cpp 负责数据新鲜度检查，px4ctrl_node.cpp 负责参数和接口注册，input_callbacks.cpp 负责输入检查，state_machine.cpp 负责起降状态机及控制循环，main.cpp 负责进程入口。完整文件说明见 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 功能

- 起飞、悬停、位置参考跟踪、降落。
- 非阻塞模式切换、解锁/上锁请求及实际反馈确认。
- 反馈新鲜度、源时间戳、坐标系、有限数值和四元数检查。
- 姿态/推力限幅，以及反馈失效时停止外部设定值。
- RViz 2D 目标点输入、平滑轨迹、到点悬停和目标取消。
- 三维交互目标：拖动 x/y/z 轴与航向环，右键 Fly to target 提交；Orbit 视角支持自由操控。
- 可选 EGO-Planner 参考接入、VIO/LIO 支持状态检查、规划取消和进度监督。
- 可配置三维目标边界，以及仿真时钟和现实时间的双重超时检查。
- 推力/电压 CSV 记录。

节点启动不会自动解锁。起飞需显式服务或消息请求。点目标功能不包含避障。

## 验证状态

已在外部 ROS 2 Humble / PX4 SITL / Gazebo / MAVROS 集成环境中使用 OpenVINS 和 FAST-LIO 机体定位运行起飞、悬停及目标导航，已有目标到达和返航记录。原生控制数学、数据新鲜度和 23 项目标节点回归通过。

本包不包含 OpenVINS、PX4、MAVROS、Gazebo、传感器桥或完整工程的一键启动脚本。独立安装后仍需外部系统提供定位及飞控通信。定位精度、长期漂移、完整异常恢复和真机飞行尚未完成独立验收；降落异常轮次需单独排查，不应归为完整起降成功。

## 依赖与构建

目标节点通过 `use_ego_planner:=true` 使用外部 EGO 轨迹，需额外安装 `quadrotor_msgs`。该模式转发经过检查的规划参考；规划器、雷达/深度建图和定位桥由外部集成工程提供，接口见下文。

使用 C++17、ament_cmake、Eigen、rclcpp、mavros_msgs、rosgraph_msgs、ROS 标准消息、rclpy、NumPy、visualization_msgs、interactive_markers 和 RViz。可选 EGO 模式另需 `quadrotor_msgs`。

将本仓库置于 ROS 工作空间的 `src/px4ctrl2`，例如 `~/px4ctrl2_ws/src/px4ctrl2`，确认 ROS 和 MAVROS 依赖可用后执行：

```bash
source /opt/ros/humble/setup.bash
# 源码安装 MAVROS 时，还需加载对应 MAVROS 工作空间的 install/setup.bash。
cd ~/px4ctrl2_ws
colcon build --packages-select px4ctrl2 --parallel-workers 1 \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DBUILD_TESTING=ON
source install/setup.bash
```

在现有 my_uav_vln 工程中使用时，将工作目录替换为该工程的 `ros2_ws`。普通安装模式下，修改脚本或配置后需重新构建安装，并重启节点。

## 控制器启动

外部 PX4 和 MAVROS 已运行、反馈有效时：

下面的默认、OpenVINS 和 FAST-LIO 启动方式选择一种；切换前停止旧控制器。

```bash
ros2 launch px4ctrl2 run_ctrl.launch.py
```

默认定位输入为 `/mavros/local_position/odom`。使用已适配到机体的 OpenVINS 定位：

```bash
ros2 launch px4ctrl2 run_ctrl.launch.py \
  odom_topic:=/openvins/body_odom
```

`/openvins/body_odom` 由外部适配器提供，本包不会生成它。控制器仍使用 `/mavros/imu/data`，并通过 MAVROS 模式和解锁服务控制 PX4。MAVROS 的 `setpoint_raw` 插件需设置 `thrust_scaling=1.0`，以实际读取结果确认。

FAST-LIO 的机体定位使用同一接口：

```bash
ros2 launch px4ctrl2 run_ctrl.launch.py \
  odom_topic:=/fastlio/body_odom
```

`/fastlio/body_odom` 也需要外部桥提供。原始 `/fastlio/odometry` 跟踪的是雷达 IMU，必须先处理安装外参和速度表达，再作为机体反馈。包内默认悬停推力是历史模型的起点；实际配置需匹配飞机质量和推力模型。完整工程当前使用单独生成的模型配置。

```bash
ros2 param set /mavros/setpoint_raw thrust_scaling 1.0
ros2 param get /mavros/setpoint_raw thrust_scaling
```

起降操作：

```bash
ros2 service call /px4ctrl2/takeoff std_srvs/srv/Trigger '{}'
ros2 service call /px4ctrl2/hover std_srvs/srv/Trigger '{}'
ros2 service call /px4ctrl2/land std_srvs/srv/Trigger '{}'
```

以上请求应按实际状态分别执行。服务接受不代表飞控操作已完成。降落要求 AUTO_HOVER；若有外部轨迹流，先停止轨迹源再请求悬停和降落。

## RViz 点目标

控制器使用 `/openvins/body_odom`、目标 frame_id 为 `vio_odom` 时，在加载工作空间环境的终端分别运行：

```bash
ros2 run px4ctrl2 rviz_goal_node.py
```

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share px4ctrl2)/config/rviz_goal.rviz"
```

起飞进入 AUTO_HOVER 后，使用 RViz 的 2D Goal Pose 点击并拖动设置目标位置和最终航向。节点保持当前高度，默认速度上限 0.4 m/s、加速度上限 0.3 m/s²，单次目标距机体不超过 5 米。

三维目标使用 Interact 拖动蓝色目标球的轴和航向环；右键选择 Fly to target 才提交，Reset to vehicle 重置草稿。也支持 `/goal_pose_3d`（PoseStamped，使用配置的世界坐标及完整 x/y/z）。默认视角为 Orbit，Move Camera 可旋转、平移和缩放。

目标流取消服务：

```bash
ros2 service call /rviz_goal/cancel std_srvs/srv/Trigger '{}'
```

手动控制时，应先取消目标流，再调用控制器 hover/land。完整工程的一键启动器已实现这一组合操作，但该启动器不在本仓库中。

### FAST-LIO 与 EGO 目标节点

外部系统已提供雷达建图、机体定位和匹配状态时，可运行：

```bash
ros2 run px4ctrl2 rviz_goal_node.py --ros-args \
  -p world_frame:=lio_odom \
  -p odom_topic:=/fastlio/body_odom \
  -p localization_source:=fastlio \
  -p use_ego_planner:=true \
  -r /ego/camera_pose:=/ego/sensor_pose
```

RViz 的 Fixed Frame 与定位显示话题也需设置为 `lio_odom` 和 `/fastlio/body_odom`；仓库中的 RViz 模板默认使用 OpenVINS 配置。

| 接口 | 方向与用途 |
|---|---|
| `/ego/camera_pose` | 输入，配对观测的位姿；雷达模式如上重映射 |
| `/ego/position_cmd` | 输入，`quadrotor_msgs/msg/PositionCommand`，只接受 READY、时间/坐标及限幅有效的新轨迹 |
| `/ego/planner_status` | 输入，`std_msgs/msg/String`；处理以 `REJECTED:` 开头的拒绝状态 |
| `/ego/goal` | 输出，`geometry_msgs/msg/PoseStamped` |
| `/ego/cancel` | 输出，`std_msgs/msg/Empty`；规划器及轨迹服务器需处理取消 |
| `/fastlio/status` | LIO 支持状态输入，JSON 至少包含 `ready` 和传感器时间 `stamp` |
| `/ov_msckf/points_msckf`、`/ov_msckf/points_slam` | VIO 模式下的特征支持输入，不用于 LIO 模式 |

这些是当前集成协议。原版 FAST-LIO/EGO 不一定发布状态或处理取消，需要集成方提供适配。集成 EGO 模式时，不要同时运行独立的 `position_command_bridge.py` 向同一个控制话题发送命令。

### 目标参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `world_frame` | `vio_odom` | 目标和定位共同的世界坐标 |
| `odom_topic` | `/openvins/body_odom` | 目标节点的机体反馈 |
| `localization_source` | `openvins` | 可选 `openvins`、`fastlio`，决定支持状态检查 |
| `use_ego_planner` | `false` | 使用外部 EGO，或生成平滑点目标参考 |
| `max_goal_distance` | `5.0` | 单次目标距机体的最大距离，米 |
| `enforce_goal_bounds` | `false` | 是否在提交前检查三维范围 |
| `goal_min_xyz`、`goal_max_xyz` | 宽范围默认值 | 启用边界后应按实际地图、顶棚及机体余量配置 |

启用边界后，越界新目标不会替换已经接受的任务。地图内的目标仍需通过规划和碰撞检查，不能只凭 RViz 的水平投影判断是否可达。

## 主要接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `odom_topic` 指定的话题 | `nav_msgs/msg/Odometry` | 定位反馈 |
| `/px4ctrl2/command` | `px4ctrl2/msg/PositionCommand` | 位置、速度、加速度和航向参考 |
| `/px4ctrl2/takeoff_land` | `px4ctrl2/msg/TakeoffLand` | 起飞/降落消息请求 |
| `/px4ctrl2/takeoff`、`hover`、`land` | `std_srvs/srv/Trigger` | 操作服务 |
| `/px4ctrl2/state` | `std_msgs/msg/String` | 当前状态，transient-local |
| `/px4ctrl2/debug` | `px4ctrl2/msg/ControllerDebug` | 控制输出和推力映射 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | PX4 姿态及推力设定值 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz 目标输入 |
| `/goal_pose_3d` | `geometry_msgs/msg/PoseStamped` | 使用完整三维位置的目标输入 |

位置参考与里程计必须处于相同世界坐标系，世界 z 向上，机体使用 FLU，不做隐式 TF 转换。`velocity_in_body=true` 时旋转机体线速度到世界坐标；速度已在世界系表达时使用 false。MAVROS 负责飞控边界 ENU/NED、FLU/FRD 转换。

默认使用单调时钟检查反馈和模式确认超时。`odom_reference_time:=true` 让运动参考、起降阶段时长和推力映射辨识使用里程计源时间；RViz 轨迹也按该时间推进。

控制器和目标节点均提供 `simulation_feedback`，默认 `false`。外部仿真启动器可将其设为 `true`：订阅 `/clock`，按仿真时间检查数据是否过期，并保留独立的现实时间上限；控制器的 `simulation_wall_timeout` 默认 2.0 秒。

使用该模式时，集成方需同步配置 MAVROS 核心和 time 插件的时钟，并让外部位姿及命令使用相同仿真时间基准。上述 MAVROS 设置由完整工程启动器执行，本包不会自动切换 MAVROS 参数。`run_ctrl.launch.py` 通过 `config:=你的配置.yaml` 加载仿真开关，运动参考另传 `odom_reference_time:=true`。

定位或 IMU 超时、时间回退、飞行中 frame 改变、飞控退出控制等可进入 ABORT。ABORT 不自动重抢 OFFBOARD，由 PX4 已配置的保护策略处理。

仿真目标节点由启动器传入 `enforce_goal_bounds`、`goal_min_xyz` 和 `goal_max_xyz`，取值与本轮 EGO 配置一致。RViz 目标标记显示允许的 Z 范围；越界目标在提交给规划器前被拒绝，不替换正在执行的目标。Z 是对应 `vio_odom`/`lio_odom` 的坐标值。

## 测试

```bash
source /opt/ros/humble/setup.bash
cd ~/px4ctrl2_ws
source install/setup.bash
colcon test --packages-select px4ctrl2 --event-handlers console_direct+
colcon test-result --test-result-base build/px4ctrl2
```

CTest 包含 `controller_test`、`freshness_test` 和 `rviz_goal_test`：覆盖控制数学、慢仿真及真实停更、VIO/LIO 目标处理、取消、三维边界和进度监督。前两项是原生数值检查，目标测试使用无 DDS 的测试驱动。

`test/mock_mavros_test.py` 是手动运行的协议测试，需可用的 ROS/DDS 通信环境，并应在隔离 ROS domain 中运行；它不模拟真实飞行动力学。

## 许可证

本项目沿用原控制器的 GPL-3.0 许可证，参见 [LICENSE](LICENSE)。
