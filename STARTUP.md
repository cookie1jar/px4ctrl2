# px4ctrl2 仿真启动与起降验证（逐终端）

适用：本项目 ROS 2 Humble、Gazebo Harmonic、PX4 v1.16、源码编译的 MAVROS。以下使用完整集成工程的目录示例；请按实际安装位置调整。准备 **4 个终端**，按下面顺序操作。

当前状态：本包已在完整工程中运行 OpenVINS 定位反馈下的起飞、悬停和 RViz 点目标飞行。本文保留使用 MAVROS/PX4 定位的手动分终端流程，不等同于无 GPS 的完整工程一键流程。文件架构及外部组件边界见 [ARCHITECTURE.md](ARCHITECTURE.md)，独立包构建及 RViz 启动见 [README.md](README.md)。

本文依赖外部集成工程提供 PX4 模型、世界及 MAVROS，不是独立克隆本仓库后即可运行的整套仿真。包内 hover_percentage=0.76 是历史模型的起点；目前搭载 MID360 的完整工程使用独立生成的约 0.835 配置。实际数值需匹配模型或实机，不应按文中的历史常数强制设置。

## 0. 首次使用新配置：构建一次

在任意普通终端执行，完成后再开始以下四终端流程：

```bash
cd ~/my_ws/my_uav_vln/ros2_ws
source /opt/ros/humble/setup.bash
source ~/my_ws/my_uav_vln/external/mavros_ws/install/setup.bash
colcon build --packages-select px4ctrl2 --parallel-workers 1 \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
```

配置安装在 `~/my_ws/my_uav_vln/ros2_ws/install/px4ctrl2/share/px4ctrl2/config/ctrl.yaml`。改源目录配置后需要重新构建/安装，才能影响默认 launch；正在运行的节点也必须重启。

## 1. 终端 1：PX4 与 Gazebo

首次联调建议先结束此前已落地、未解锁的旧仿真和旧 MAVROS/ctrl2，避免多个控制节点或多个仿真世界同时运行。若飞机仍在空中，不要直接关闭进程，应先完成降落。

```bash
cd ~/my_ws/my_uav_vln/PX4-Autopilot
make px4_sitl gz_x500_depth_baylands
```

等待 Gazebo 出现飞机、PX4 出现 `pxh>`。Gazebo 必须处于播放状态。保持本终端运行。

本终端输入的是 **PX4 命令**，例如 `commander status`，不能输入 `ros2 ...`。

本流程按 **当前 Gazebo SITL 不依赖 QGC** 配置。在飞机位于地面且未解锁时，在本终端的 `pxh>` 输入：

```text
param set NAV_DLL_ACT 0
param show NAV_DLL_ACT
```

确认值为 0。它关闭地面站失联动作，并取消本地 PX4 因该动作配置而要求 GCS 连接的解锁条件；不强制解锁，也不关闭其他传感器/估计器健康检查。此设置仅用于这里的 SITL 联调，不作为实机配置建议。

QGC 可用于观察状态，但本流程不要求保持连接。若后续希望恢复原来的地面站失联动作，落地未解锁时执行 `param set NAV_DLL_ACT 2`，并连接 QGC。

## 2. 终端 2：MAVROS

```bash
source /opt/ros/humble/setup.bash
source ~/my_ws/my_uav_vln/external/mavros_ws/install/setup.bash

ros2 launch mavros px4.launch \
  fcu_url:='udp://:14540@127.0.0.1:14580'
```

保持本终端运行。不要另启第二个 MAVROS，不用此前不带配置的裸 mavros_node 启动命令。

**本机 MAVROS 插件仍需下一步运行时设置 thrust_scaling。只启动这个 launch 还不够。**

## 3. 终端 3：设置 MAVROS 参数并检查链路

```bash
source /opt/ros/humble/setup.bash
source ~/my_ws/my_uav_vln/external/mavros_ws/install/setup.bash
source ~/my_ws/my_uav_vln/ros2_ws/install/setup.bash

ros2 param set /mavros/setpoint_raw thrust_scaling 1.0
ros2 param get /mavros/setpoint_raw thrust_scaling
```

必须显示：

```text
Set parameter successful
Double value is: 1.0
```

若提示找不到节点/服务，说明 MAVROS 还未启动完成或命名空间不对，先处理，不继续起飞。每次重启终端 2 的 MAVROS 都要重新设置；ctrl2 的 hover_percentage 与 MAVROS 的 thrust_scaling 是两个不同参数，不能互相替代。

继续检查：

```bash
ros2 topic echo /mavros/state --once --qos-reliability best_effort
ros2 topic echo /mavros/extended_state --once --qos-reliability best_effort
ros2 topic echo /mavros/local_position/odom --once --qos-reliability best_effort
ros2 topic echo /mavros/imu/data --once --qos-reliability best_effort
```

起飞前应满足：

- state.connected=true、armed=false。
- extended_state.landed_state=1，确认在地面。
- odom 与 IMU 实际收到数据；IMU 含有效姿态。
- state.mode 不为 OFFBOARD；否则先在终端 1 的 `pxh>` 输入 `commander mode auto:loiter`。

命令若一直等待，没有消息，Ctrl-C 结束该条读取命令，检查终端 2，不继续起飞。

## 4. 终端 4：启动 ctrl2

```bash
source /opt/ros/humble/setup.bash
source ~/my_ws/my_uav_vln/external/mavros_ws/install/setup.bash
source ~/my_ws/my_uav_vln/ros2_ws/install/setup.bash

ros2 launch px4ctrl2 run_ctrl.launch.py
```

本终端显示控制器状态转移。启动时只应报告 ready，不会自行解锁。保持运行。

在 **终端 3** 核验本次实际加载的参数：

```bash
ros2 param get /px4ctrl2 hover_percentage
ros2 param get /px4ctrl2 max_thrust
ros2 param get /px4ctrl2 takeoff_height
ros2 param get /px4ctrl2 odom_topic
```

包内默认值分别为 **0.76、0.9、1.0、/mavros/local_position/odom**；使用外部模型配置时，以该配置为准。先核对本次加载值与所用模型一致，再请求起飞。

当前用 PX4 自身里程计进行控制；OpenVINS 可以旁路运行，但不作为本次起飞前置条件。

## 5. 终端 3：请求解锁并起飞

```bash
ros2 service call /px4ctrl2/takeoff std_srvs/srv/Trigger '{}'
```

这条服务执行整个 **Offboard → arm → 起飞 → 悬停** 流程，不是单独 arm。success=true 表示本节点接受请求，不保证实际起飞。

观察 **终端 4**，预期：

```text
MANUAL_CTRL / ABORT
  → PRESTREAM
  → WAIT_OFFBOARD
  → WAIT_ARM
  → AUTO_TAKEOFF
  → AUTO_HOVER
```

先有约 1.2 s 的设定值预发送，解锁确认后再有约 3 s 的电机启动曲线；此后按 0.3 m/s 上升到相对起飞位置约 1 m。飞机不能只是电机旋转，要在 Gazebo 中实际升高，并最终出现 AUTO_HOVER。

需要观察数据时，在 **终端 3** 执行：

```bash
ros2 topic echo /mavros/local_position/odom \
  --field pose.pose.position --qos-reliability best_effort
```

这是 MAVROS 的 ROS 世界坐标，z 向上。观察完用 Ctrl-C 结束此读取命令；不会停止终端 4 的控制器。不要与 PX4 `vehicle_local_position` 的 NED z 混淆，后者向上飞时 z 减小。

首次试验先只起飞悬停，不发布轨迹指令、不切换定位源。

## 6. 终端 3：请求降落

确认终端 4 已进入 AUTO_HOVER 后执行：

```bash
ros2 service call /px4ctrl2/land std_srvs/srv/Trigger '{}'
```

预期：AUTO_LAND → WAIT_DISARM → EXIT_OFFBOARD → MANUAL_CTRL。落地后还会等待本地/PX4 落地确认，不是接触地面立即上锁。

检查最终状态：

```bash
ros2 topic echo /mavros/state --once --qos-reliability best_effort
```

armed=false 才是已上锁。确认落地、上锁后再关闭各终端。

若后续已经在 CMD_CTRL，先停止轨迹发布，调用 `/px4ctrl2/hover` 回悬停，再调用 land；持续轨迹流会让控制器再次进入 CMD_CTRL。

## 7. 失败时如何处理

| 现象 | 含义 / 操作 |
|---|---|
| No connection to the ground control station | 终端 1 在地面未解锁时设置 NAV_DLL_ACT=0，或连接 QGC；按步骤 1 配置 |
| takeoff 返回 success=false | 请求前置条件未满足，看 response.message |
| WAIT_ARM → ABORT，MAV_RESULT=1 | PX4 拒绝解锁；检查终端 2 是否有 ignore_thrust，终端 3 读取 thrust_scaling |
| AUTO_TAKEOFF → ABORT，takeoff altitude timeout | 已解锁但未达到目标高度；保存 ctrl2 日志和 ULog，不算起飞成功 |
| stale/connection/IMU/odom 错误 | 检查 Gazebo 未暂停、MAVROS 数据持续推进 |
| PX4 exited OFFBOARD | 飞控退出外部控制，ctrl2 停止接管，不会自动抢回 |

ABORT 后 `/px4ctrl2/land` 不能用于重新接管降落，因为此时控制器已停止输出。如果飞机在空中，在 **终端 1 的 pxh>** 请求由 PX4 自身执行降落：

```text
commander mode auto:land
```

这依赖 PX4 自身状态估计有效，需观察真实下降、着陆和上锁，不保证命令一发就成功。不要在空中强制 disarm。

落地且未解锁后才重试；先在 **终端 1** 执行：

```text
commander status
commander mode auto:loiter
```

必要时重启终端 4 的 ctrl2，再在终端 3 请求 takeoff。不要每次一看到 ABORT 就连续发送起飞请求。

## 8. 首次起飞仍失败时，保存什么

提供：终端 4 完整状态转移/错误、终端 2 的 ERROR/WARN、终端 3 的两个参数值：

```bash
ros2 param get /mavros/setpoint_raw thrust_scaling
ros2 param get /px4ctrl2 hover_percentage
```

同时在 **终端 1 的 pxh>** 输入：

```text
commander status
listener vehicle_local_position 1
listener vehicle_attitude_setpoint 1
```

PX4 ULog 位于 `PX4-Autopilot/build/px4_sitl_default/rootfs/log/`。arm 成功、PX4 提示 Takeoff detected 或节点服务返回 true，都不能单独证明飞机已达到指定高度并悬停。

## 9. 外部 OpenVINS / FAST-LIO 定位

外部工程负责传感器、估计器和 IMU 到机体的适配。控制器可通过 odom_topic 接收
`/openvins/body_odom` 或 `/fastlio/body_odom`，目标节点需同步设置 world_frame、
odom_topic 和 localization_source，具体命令见 [README](README.md)。

本页的手动流程使用 PX4/MAVROS 定位，不等于外部无 GPS 定位已完成部署。
完整工程的一键启动器、场景、定位桥和 EGO 不包含在此仓库。仿真时间模式
还需由集成方统一 MAVROS、外部位姿和控制节点的时钟，不能只切换单个开关。

### 历史模型的悬停推力估算

以下为旧模型的计算记录，不用于当前 MID360 模型：估算假定四个电机转速相同、机体水平，使用总升力 `4 * motorConstant * omega^2 = mass * g`，得到 omega≈797.4，再按 `(omega-150)/(1000-150)` 得到约 0.762。实际分配、机体倾斜、电机动态和地面效应会改变结果；这只解释为什么原来 0.5 的配置可能起不来，不能替代飞行验收。在线辨识仅在悬停/命令状态启用，因此在第一次起飞前不会自动修正过低的推力初值。
