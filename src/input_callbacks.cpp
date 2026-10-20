// ROS input validation. GPL-3.0-only.
#include "px4ctrl2/px4ctrl2_node.hpp"

#include <algorithm>
#include <cmath>

namespace px4ctrl2 {

void Node::on_clock(rosgraph_msgs::msg::Clock::ConstSharedPtr m) {
  const int64_t stamp = rclcpp::Time(m->clock).nanoseconds();
  if (stamp < simulation_stamp_) {
    have_odom_ = have_imu_ = have_command_ = false;
    if (state_ != State::MANUAL && state_ != State::ABORT) abort("simulation clock moved backwards");
  }
  if (stamp != simulation_stamp_) simulation_at_ = monotonic();
  simulation_stamp_ = stamp;
}

void Node::on_odometry(nav_msgs::msg::Odometry::ConstSharedPtr m) {
  const int64_t stamp = rclcpp::Time(m->header.stamp).nanoseconds();
  if (stamp <= 0 || stamp == odom_stamp_) {
    return;
  }
  if (stamp < odom_stamp_) {
    have_odom_ = false;
    odom_stamp_ = stamp;
    if (state_ != State::MANUAL && state_ != State::ABORT) {
      abort("odometry time moved backwards");
    }
    return;
  }
  odom_stamp_ = stamp;
  Feedback f;
  f.p = {m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z};
  f.v = {m->twist.twist.linear.x, m->twist.twist.linear.y, m->twist.twist.linear.z};
  f.q = quaternion(m->pose.pose.orientation);
  if (!f.p.allFinite() || !f.v.allFinite() || !validQuaternion(f.q) || m->header.frame_id.empty()) {
    have_odom_ = false;
    return;
  }
  if (have_odom_ && m->header.frame_id != odom_frame_ && state_ != State::MANUAL && state_ != State::ABORT) {
    have_odom_ = false;
    abort("odometry frame changed during control");
    return;
  }
  f.q.normalize();
  if (velocity_body_) {
    f.v = f.q * f.v;
  }
  odom_ = f;
  odom_frame_ = m->header.frame_id;
  have_odom_ = true;
  odom_at_ = monotonic();
}

void Node::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr m) {
  const int64_t stamp = rclcpp::Time(m->header.stamp).nanoseconds();
  if (stamp <= 0 || stamp == imu_stamp_) {
    return;
  }
  if (stamp < imu_stamp_) {
    have_imu_ = false;
    imu_stamp_ = stamp;
    if (state_ != State::MANUAL && state_ != State::ABORT) {
      abort("IMU time moved backwards");
    }
    return;
  }
  imu_stamp_ = stamp;
  const auto q = quaternion(m->orientation);
  Eigen::Vector3d a(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
  if (m->orientation_covariance[0] < 0 || !validQuaternion(q) || !a.allFinite()) {
    have_imu_ = false;
    return;
  }
  imu_q_ = q.normalized();
  imu_a_ = a;
  have_imu_ = true;
  imu_at_ = monotonic();
  imu_simulation_stamp_ = simulation_stamp_;
}

void Node::on_state(mavros_msgs::msg::State::ConstSharedPtr m) {
  fcu_ = *m;
  have_state_ = true;
  state_at_ = monotonic();
}

void Node::on_extended_state(mavros_msgs::msg::ExtendedState::ConstSharedPtr m) {
  extended_ = *m;
  have_extended_ = true;
  extended_at_ = monotonic();
}

void Node::on_rc(mavros_msgs::msg::RCIn::ConstSharedPtr m) {
  if (m->channels.size() < 8) {
    have_rc_ = false;
    return;
  }
  for (size_t i = 0; i < 4; ++i) {
    double v = std::clamp((m->channels[i] - 1500.0) / 500.0, -1.0, 1.0);
    sticks_[i] = (std::abs(v) <= 0.25 ? 0 : std::copysign((std::abs(v) - 0.25) / 0.75,
                  v)) * (rc_reverse_[i] ? -1 : 1);
  }
  rc_hover_ = m->channels[4] > 1750;
  rc_command_ = m->channels[5] > 1750;
  have_rc_ = true;
  rc_at_ = monotonic();
}

void Node::on_command(msg::PositionCommand::ConstSharedPtr m) {
  if (state_ != State::HOVER && state_ != State::COMMAND) {
    have_command_ = false;
    return;
  }
  Desired d;
  d.p = {m->position.x, m->position.y, m->position.z};
  d.v = {m->velocity.x, m->velocity.y, m->velocity.z};
  d.a = {m->acceleration.x, m->acceleration.y, m->acceleration.z};
  d.yaw = m->yaw;
  if (m->header.frame_id != odom_frame_ || !d.p.allFinite() || !d.v.allFinite() || !d.a.allFinite() ||
      !std::isfinite(d.yaw)) {
    have_command_ = false;
    RCLCPP_WARN(get_logger(), "Rejected command: frame/values invalid");
    return;
  }
  command_ = d;
  have_command_ = true;
  command_at_ = monotonic();
  command_stamp_ = rclcpp::Time(m->header.stamp).nanoseconds();
}

void Node::on_takeoff_land(msg::TakeoffLand::ConstSharedPtr m) {
  std::pair<bool, std::string> r;
  if (m->takeoff_land_cmd == msg::TakeoffLand::TAKEOFF) {
    r = takeoff();
  } else if (m->takeoff_land_cmd == msg::TakeoffLand::LAND) {
    r = land();
  } else
    r = {false, "unknown takeoff_land command"};
  RCLCPP_INFO(get_logger(), "%s: %s", r.first ? "accepted" : "rejected", r.second.c_str());
}

}  // namespace px4ctrl2
