#pragma once
// ROS 2 migration of Fast-Drone px4ctrl. GPL-3.0-only.

#include "px4ctrl2/controller.hpp"
#include "px4ctrl2/msg/controller_debug.hpp"
#include "px4ctrl2/msg/position_command.hpp"
#include "px4ctrl2/msg/takeoff_land.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <utility>

namespace px4ctrl2 {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  enum class State {
    MANUAL, PRESTREAM, WAIT_MODE, WAIT_ARM, TAKEOFF, HOVER,
    COMMAND, LAND, WAIT_DISARM, EXIT_MODE, ABORT
  };
  using OperationResult = std::pair<bool, std::string>;
  using Operation = OperationResult (Node::*)();

  // Construction and ROS wiring (src/px4ctrl_node.cpp).
  void load_parameters();
  void setup_subscriptions();
  void setup_publishers();
  void setup_services();
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr create_operation_service(
      const std::string &service, Operation action);

  // Input validation (src/input_callbacks.cpp).
  void on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message);
  void on_state(mavros_msgs::msg::State::ConstSharedPtr message);
  void on_extended_state(mavros_msgs::msg::ExtendedState::ConstSharedPtr message);
  void on_rc(mavros_msgs::msg::RCIn::ConstSharedPtr message);
  void on_command(msg::PositionCommand::ConstSharedPtr message);
  void on_takeoff_land(msg::TakeoffLand::ConstSharedPtr message);

  // Flight state machine and control output (src/state_machine.cpp).
  double monotonic();
  double reference_time();
  static Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion &q);
  static const char *name(State state);
  bool feedback(double time) const;
  bool connected(double time) const;
  bool rc_valid(double time) const;
  bool command_valid(double time) const;
  bool grounded(double time) const;
  void transition(State state);
  void hold();
  void abort(const std::string &reason);
  void request_mode(const std::string &mode);
  void request_arm(bool value);
  OperationResult takeoff();
  OperationResult hover();
  OperationResult land();
  void publish(const Output &output);
  void tick();

  // Control state and latest feedback.
  State state_{State::MANUAL};
  LinearControl controller_;
  Feedback odom_;
  Desired command_, target_;
  Eigen::Quaterniond imu_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d imu_a_{Eigen::Vector3d::Zero()};
  mavros_msgs::msg::State fcu_;
  mavros_msgs::msg::ExtendedState extended_;
  std::array<double, 4> sticks_{};
  std::array<bool, 4> rc_reverse_{};
  bool rc_hover_{false}, rc_command_{false}, no_rc_, auto_arm_, velocity_body_;
  bool odom_reference_time_{false};
  bool have_odom_{false}, have_imu_{false}, have_state_{false};
  bool have_rc_{false}, have_extended_{false}, have_command_{false};
  bool takeoff_requested_{true};
  std::string odom_frame_;

  // Timestamps, asynchronous operations and state entry data.
  int64_t odom_stamp_{-1}, imu_stamp_{-1}, last_estimate_imu_stamp_{-1};
  bool trigger_sent_{false}, operation_done_{false}, operation_ok_{false};
  uint64_t generation_{0};
  double odom_at_{0}, imu_at_{0}, state_at_{0}, rc_at_{0};
  double extended_at_{0}, command_at_{0};
  double entered_{0}, started_z_{0}, land_still_at_{-1}, hover_at_{0};
  double entered_reference_{0}, last_reference_tick_{0};
  std::string previous_mode_{"POSCTL"}, abort_reason_;
  rclcpp::Clock steady_{RCL_STEADY_TIME};

  // Parameters loaded during construction.
  double frequency_, feedback_timeout_, state_timeout_, command_timeout_;
  double operation_timeout_, takeoff_timeout_, height_, speed_, manual_velocity_;

  // ROS entities.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<msg::PositionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<msg::TakeoffLand>::SharedPtr takeoff_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_pub_;
  rclcpp::Publisher<msg::ControllerDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_, land_srv_, hover_srv_;
};

}  // namespace px4ctrl2
