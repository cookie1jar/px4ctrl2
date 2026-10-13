# px4ctrl2

ROS 2 外部位置控制包。接收里程计和位置参考，生成目标姿态及归一化推力，经 MAVROS 发送给 PX4，由 PX4 完成底层控制。

基于 ROS 1 px4ctrl 的提交 `7f3b0c8559d1d3569f92397ad35d38408bbd1ef5` 迁移，沿用 GPL-3.0。控制核心保留原版位置/速度反馈、小角度姿态求解、姿态参考对齐及延迟推力映射辨识。

## 文档

- [文件架构与模块说明](ARCHITECTURE.md)：完整目录、文件职责、数据流、状态机和接口边界。
- [仿真启动与起降检查](STARTUP.md)：PX4/MAVROS 手动启动及控制器检查流程。

## 功能

- 起飞、悬停、位置参考跟踪、降落。
- 非阻塞模式切换、解锁/上锁请求及实际反馈确认。
- 反馈新鲜度、源时间戳、坐标系、有限数值和四元数检查。
- 姿态/推力限幅，以及反馈失效时停止外部设定值。
- RViz 2D 目标点输入、平滑轨迹、到点悬停和目标取消。
- 可选 EGO-Planner 消息适配，以及推力/电压 CSV 记录。

节点启动不会自动解锁。起飞需显式服务或消息请求。点目标功能不包含避障。

## 验证状态

已在本项目 ROS 2 Humble / PX4 SITL / Gazebo / MAVROS / OpenVINS 集成环境中运行起飞、悬停和 RViz 点目标飞行，日志记录多次目标到达并返回 AUTO_HOVER。控制计算与目标处理回归测试通过。

本包不包含 OpenVINS、PX4、MAVROS、Gazebo、传感器桥或完整工程的一键启动脚本。独立安装后仍需外部系统提供定位及飞控通信。定位精度、长期漂移、完整异常恢复和真机飞行尚未完成独立验收；降落异常轮次需单独排查，不应归为完整起降成功。

## 依赖与构建

使用 C++17、ament_cmake、Eigen、rclcpp、mavros_msgs、ROS 标准消息、rclpy、NumPy、visualization_msgs 和 RViz。可选 EGO 适配脚本另需 `quadrotor_msgs`。

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

```bash
ros2 launch px4ctrl2 run_ctrl.launch.py
```

默认定位输入为 `/mavros/local_position/odom`。使用已适配到机体的 OpenVINS 定位：

```bash
ros2 launch px4ctrl2 run_ctrl.launch.py \
  odom_topic:=/openvins/body_odom
```

`/openvins/body_odom` 由外部适配器提供，本包不会生成它。控制器仍使用 `/mavros/imu/data`，并通过 MAVROS 模式和解锁服务控制 PX4。MAVROS 的 `setpoint_raw` 插件需设置 `thrust_scaling=1.0`，以实际读取结果确认。

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

控制器使用 `/openvins/body_odom`、目标 frame_id 为 `global` 时，在加载工作空间环境的终端分别运行：

```bash
ros2 run px4ctrl2 rviz_goal_node.py
```

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share px4ctrl2)/config/rviz_goal.rviz"
```

起飞进入 AUTO_HOVER 后，使用 RViz 的 2D Goal Pose 点击并拖动设置目标位置和最终航向。节点保持当前高度，默认速度上限 0.4 m/s、加速度上限 0.3 m/s²，单次目标距机体不超过 5 米。

目标流取消服务：

```bash
ros2 service call /rviz_goal/cancel std_srvs/srv/Trigger '{}'
```

手动控制时，应先取消目标流，再调用控制器 hover/land。完整工程的一键启动器已实现这一组合操作，但该启动器不在本仓库中。

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

位置参考与里程计必须处于相同世界坐标系，世界 z 向上，机体使用 FLU，不做隐式 TF 转换。`velocity_in_body=true` 时旋转机体线速度到世界坐标；速度已在世界系表达时使用 false。MAVROS 负责飞控边界 ENU/NED、FLU/FRD 转换。

反馈/操作超时使用单调时钟，消息时间戳使用 ROS 时钟。定位或 IMU 超时、时间回退、飞行中 frame 改变、飞控退出控制等可进入 ABORT。ABORT 不自动重抢 OFFBOARD，由 PX4 已配置的保护策略处理。

## 测试

```bash
source /opt/ros/humble/setup.bash
cd ~/px4ctrl2_ws
source install/setup.bash
colcon test --packages-select px4ctrl2 --event-handlers console_direct+
colcon test-result --test-result-base build/px4ctrl2
```

CTest 包含控制数学测试和 RViz 目标处理测试。`test/mock_mavros_test.py` 是手动运行的协议测试，需可用的 ROS/DDS 通信环境，并应在隔离 ROS domain 中运行；它不模拟真实飞行动力学。

## 许可证

本项目沿用原控制器的 GPL-3.0 许可证，参见 [LICENSE](LICENSE)。
