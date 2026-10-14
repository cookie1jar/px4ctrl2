// ROS node construction, parameter loading and interface wiring. GPL-3.0-only.
#include "px4ctrl2/px4ctrl2_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

namespace px4ctrl2 {

Node::Node() : rclcpp::Node("px4ctrl2") {
  load_parameters();
  setup_subscriptions();
  setup_publishers();
  setup_services();
  last_reference_tick_ = reference_time();
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
      std::bind(&Node::tick, this));
  RCLCPP_INFO(get_logger(),
              "px4ctrl2 ready. No automatic launch-time arming. MAVROS + valid external odom required.");
}

void Node::load_parameters() {
  odom_reference_time_ = declare_parameter("odom_reference_time", false);
  frequency_ = declare_parameter("control_frequency", 100.0);
  feedback_timeout_ = declare_parameter("feedback_timeout", 0.5);
  state_timeout_ = declare_parameter("state_timeout", 2.0);
  command_timeout_ = declare_parameter("command_timeout", 0.5);
  operation_timeout_ = declare_parameter("operation_timeout", 10.0);
  takeoff_timeout_ = declare_parameter("takeoff_timeout", 30.0);
  height_ = declare_parameter("takeoff_height", 1.0);
  speed_ = declare_parameter("takeoff_land_speed", 0.3);
  manual_velocity_ = declare_parameter("max_manual_velocity", 0.8);
  rc_reverse_[0] = declare_parameter("rc_reverse.roll", false);
  rc_reverse_[1] = declare_parameter("rc_reverse.pitch", false);
  rc_reverse_[2] = declare_parameter("rc_reverse.throttle", false);
  rc_reverse_[3] = declare_parameter("rc_reverse.yaw", false);
  no_rc_ = declare_parameter("no_rc", true);
  auto_arm_ = declare_parameter("enable_auto_arm", true);
  velocity_body_ = declare_parameter("velocity_in_body", false);
  controller_.hover = declare_parameter("hover_percentage", 0.5);
  controller_.max_thrust = declare_parameter("max_thrust", 0.8);
  controller_.max_angle = declare_parameter("max_angle_deg", 25.0) * std::acos(-1.0) / 180;
  const auto kp = declare_parameter("kp", std::vector<double> {1.5, 1.5, 1.5});
  const auto kv = declare_parameter("kv", std::vector<double> {1.5, 1.5, 1.5});
  if (kp.size() != 3 || kv.size() != 3) {
    throw std::invalid_argument("kp/kv require three values");
  }
  controller_.kp = Eigen::Vector3d(kp[0], kp[1], kp[2]);
  controller_.kv = Eigen::Vector3d(kv[0], kv[1], kv[2]);
  const std::array<double, 12> numbers{
      frequency_, feedback_timeout_, state_timeout_, command_timeout_,
      operation_timeout_, takeoff_timeout_, height_, speed_, manual_velocity_,
      controller_.hover, controller_.max_thrust, controller_.max_angle};
  for (double n : numbers) {
    if (!std::isfinite(n) || n <= 0) {
      throw std::invalid_argument("positive finite parameters required");
    }
  }
  if (frequency_ < 10 || frequency_ > 500 || controller_.hover >= controller_.max_thrust ||
      controller_.max_thrust > 1 || controller_.max_angle > 0.8 || !controller_.kp.allFinite() ||
      !controller_.kv.allFinite() || (controller_.kp.array() < 0).any() || (controller_.kv.array() < 0).any()) {
    throw std::invalid_argument("invalid controller bounds/gains");
  }
  controller_.reset();
}

void Node::setup_subscriptions() {
  using std::placeholders::_1;
  const auto sensor_qos = rclcpp::SensorDataQoS();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("odom_topic", std::string("/odom")), sensor_qos,
      std::bind(&Node::on_odometry, this, _1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/mavros/imu/data", sensor_qos, std::bind(&Node::on_imu, this, _1));
  state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", sensor_qos, std::bind(&Node::on_state, this, _1));
  extended_sub_ = create_subscription<mavros_msgs::msg::ExtendedState>(
      "/mavros/extended_state", sensor_qos, std::bind(&Node::on_extended_state, this, _1));
  if (!no_rc_) {
    rc_sub_ = create_subscription<mavros_msgs::msg::RCIn>(
      "/mavros/rc/in", sensor_qos, std::bind(&Node::on_rc, this, _1));
  }
  command_sub_ = create_subscription<msg::PositionCommand>(
      declare_parameter("command_topic", std::string("/px4ctrl2/command")), rclcpp::QoS(10),
      std::bind(&Node::on_command, this, _1));
  takeoff_sub_ = create_subscription<msg::TakeoffLand>(
      "~/takeoff_land", rclcpp::QoS(10), std::bind(&Node::on_takeoff_land, this, _1));
}

void Node::setup_publishers() {
  attitude_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", rclcpp::QoS(10));
  trigger_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/traj_start_trigger", 10);
  debug_pub_ = create_publisher<msg::ControllerDebug>("~/debug", 10);
  state_pub_ = create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(1).transient_local());
}

void Node::setup_services() {
  mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  takeoff_srv_ = create_operation_service("~/takeoff", &Node::takeoff);
  land_srv_ = create_operation_service("~/land", &Node::land);
  hover_srv_ = create_operation_service("~/hover", &Node::hover);
}

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr Node::create_operation_service(
    const std::string &service, Operation action) {
  return create_service<std_srvs::srv::Trigger>(
      service,
      [this, action](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const auto result = (this->*action)();
        response->success = result.first;
        response->message = result.second;
      });
}

}  // namespace px4ctrl2
