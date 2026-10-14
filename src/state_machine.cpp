// Flight state machine and MAVROS control operations. GPL-3.0-only.
#include "px4ctrl2/px4ctrl2_node.hpp"

#include <algorithm>
#include <cmath>

namespace px4ctrl2 {

double Node::monotonic() {
  return steady_.now().seconds();
}

double Node::reference_time() {
  return odom_reference_time_ ? std::max<int64_t>(odom_stamp_, 0) * 1e-9 : monotonic();
}

Eigen::Quaterniond Node::quaternion(const geometry_msgs::msg::Quaternion &q) {
  return {q.w, q.x, q.y, q.z};
}

const char *Node::name(State s) {
  switch (s) {
  case State::MANUAL:
    return "MANUAL_CTRL";
  case State::PRESTREAM:
    return "PRESTREAM";
  case State::WAIT_MODE:
    return "WAIT_OFFBOARD";
  case State::WAIT_ARM:
    return "WAIT_ARM";
  case State::TAKEOFF:
    return "AUTO_TAKEOFF";
  case State::HOVER:
    return "AUTO_HOVER";
  case State::COMMAND:
    return "CMD_CTRL";
  case State::LAND:
    return "AUTO_LAND";
  case State::WAIT_DISARM:
    return "WAIT_DISARM";
  case State::EXIT_MODE:
    return "EXIT_OFFBOARD";
  case State::ABORT:
    return "ABORT";
  }
  return "UNKNOWN";
}

void Node::transition(State s) {
  RCLCPP_INFO(get_logger(), "%s -> %s", name(state_), name(s));
  state_ = s;
  entered_ = monotonic();
  entered_reference_ = reference_time();
  operation_done_ = false;
  operation_ok_ = false;
}

bool Node::feedback(double t) const {
  return have_odom_ && have_imu_ && t - odom_at_ < feedback_timeout_ && t - imu_at_ < feedback_timeout_;
}

bool Node::connected(double t) const {
  return have_state_ && fcu_.connected && t - state_at_ < state_timeout_;
}

bool Node::rc_valid(double t) const {
  return no_rc_ || (have_rc_ && t - rc_at_ < feedback_timeout_ && rc_hover_);
}

bool Node::command_valid(double t) const {
  return have_command_ && t - command_at_ < command_timeout_;
}

bool Node::grounded(double t) const {
  return have_extended_ && t - extended_at_ < state_timeout_ &&
         extended_.landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
}

void Node::hold() {
  target_.p = odom_.p;
  target_.v.setZero();
  target_.a.setZero();
  target_.yaw = yaw(odom_.q);
  hover_at_ = monotonic();
  trigger_sent_ = false;
}

void Node::abort(const std::string &reason) {
  abort_reason_ = reason;
  ++generation_;
  mode_client_->prune_pending_requests();
  arm_client_->prune_pending_requests();
  transition(State::ABORT);
  RCLCPP_ERROR(get_logger(),
               "Control stopped: %s. PX4 retains failsafe authority; no automatic OFFBOARD retry.", reason.c_str());
}

void Node::request_mode(const std::string &mode) {
  const auto token = ++generation_;
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = mode;
  mode_client_->async_send_request(req, [this, token](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    if (token != generation_) {
      return;
    }
    operation_done_ = true;
    operation_ok_ = future.get()->mode_sent;
    if (!operation_ok_) {
      abort("mode request rejected");
    }
  });
}

void Node::request_arm(bool value) {
  const auto token = ++generation_;
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = value;
  arm_client_->async_send_request(req, [this, token](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    if (token != generation_) {
      return;
    }
    const auto result = future.get();
    operation_done_ = true;
    operation_ok_ = result->success;
    if (!operation_ok_) {
      abort("arm/disarm request rejected, MAV_RESULT=" + std::to_string(result->result));
    }
  });
}

std::pair<bool, std::string> Node::takeoff() {
  const double t = monotonic();
  if (state_ != State::MANUAL && state_ != State::ABORT) {
    return {false, "controller is busy"};
  }
  if (!connected(t) || !feedback(t)) {
    return {false, "fresh connected state, odom and IMU required"};
  }
  if (!grounded(t)) {
    return {false, "fresh PX4 on-ground confirmation required"};
  }
  if (!rc_valid(t)) {
    return {false, "RC hover switch and fresh RC required"};
  }
  if (!no_rc_ &&
      (!rc_command_ || std::any_of(sticks_.begin(), sticks_.end(),
                                  [](double v) { return std::abs(v) > 1e-5; }))) {
    return {false, "RC command switch and centered sticks required"};
  }
  if (command_valid(t)) {
    return {false, "stop trajectory commands before takeoff"};
  }
  if (odom_.v.norm() > 0.1) {
    return {false, "non-static odometry"};
  }
  if (!mode_client_->service_is_ready() || !arm_client_->service_is_ready()) {
    return {false, "MAVROS mode/arming services unavailable"};
  }
  if (fcu_.mode == "OFFBOARD") {
    return {false, "already OFFBOARD; release other control source first"};
  }
  if (!auto_arm_ && !fcu_.armed) {
    return {false, "auto arm disabled; arm externally first"};
  }
  takeoff_requested_ = true;
  ++generation_;
  controller_.reset();
  hold();
  started_z_ = odom_.p.z();
  previous_mode_ = fcu_.mode.empty() ? "POSCTL" : fcu_.mode;
  transition(State::PRESTREAM);
  return {true, "takeoff accepted; await state and actual flight feedback"};
}

std::pair<bool, std::string> Node::land() {
  if (state_ != State::HOVER) {
    return {false, "land requires AUTO_HOVER; call hover first and stop trajectory stream"};
  }
  hold();
  started_z_ = odom_.p.z();
  land_still_at_ = -1;
  transition(State::LAND);
  return {true, "landing accepted"};
}

std::pair<bool, std::string> Node::hover() {
  if (state_ == State::MANUAL || state_ == State::ABORT) {
    const double t = monotonic();
    if (!connected(t) || !feedback(t) || !rc_valid(t) || !fcu_.armed || grounded(t))
      return {false, "in-flight hover takeover requires armed vehicle, fresh feedback and RC authorization"};
    if (fcu_.mode == "OFFBOARD")
      return {false, "another OFFBOARD source must release control first"};
    if (!mode_client_->service_is_ready())
      return {false, "MAVROS mode service unavailable"};
    takeoff_requested_ = false;
    previous_mode_ = fcu_.mode;
    ++generation_;
    controller_.reset();
    hold();
    transition(State::PRESTREAM);
    return {true, "hover takeover accepted"};
  }
  if (state_ != State::COMMAND && state_ != State::HOVER) {
    return {false, "hover requires active hover/command control"};
  }
  hold();
  have_command_ = false;
  transition(State::HOVER);
  return {true, "hover accepted; stop trajectory stream before landing"};
}

void Node::publish(const Output &u) {
  mavros_msgs::msg::AttitudeTarget out;
  out.header.stamp = now();
  out.header.frame_id = "base_link";
  out.type_mask = out.IGNORE_ROLL_RATE | out.IGNORE_PITCH_RATE | out.IGNORE_YAW_RATE;
  out.orientation.w = u.q.w();
  out.orientation.x = u.q.x();
  out.orientation.y = u.q.y();
  out.orientation.z = u.q.z();
  out.thrust = static_cast<float>(u.thrust);
  attitude_pub_->publish(out);
  msg::ControllerDebug d;
  d.header = out.header;
  d.state = name(state_);
  d.desired_orientation = out.orientation;
  d.desired_acceleration.x = u.acceleration.x();
  d.desired_acceleration.y = u.acceleration.y();
  d.desired_acceleration.z = u.acceleration.z();
  d.thrust = u.thrust;
  d.thrust_to_acceleration = controller_.scale();
  debug_pub_->publish(d);
}

void Node::tick() {
  const double t = monotonic();
  const double reference = reference_time();
  const double dt = std::clamp(reference - last_reference_tick_, 0.0, 0.05);
  last_reference_tick_ = reference;
  std_msgs::msg::String state_msg;
  state_msg.data = name(state_);
  state_pub_->publish(state_msg);
  if (state_ == State::MANUAL || state_ == State::ABORT) {
    return;
  }
  if (!connected(t) || !feedback(t) || !rc_valid(t)) {
    RCLCPP_ERROR(get_logger(),
                 "Feedback failure: state_valid=%d connected=%d state_age=%.3fs (limit %.3fs); "
                 "odom_valid=%d odom_age=%.3fs; imu_valid=%d imu_age=%.3fs (limit %.3fs); "
                 "no_rc=%d rc_valid=%d rc_age=%.3fs",
                 have_state_, fcu_.connected, t - state_at_, state_timeout_,
                 have_odom_, t - odom_at_, have_imu_, t - imu_at_, feedback_timeout_,
                 no_rc_, rc_valid(t), t - rc_at_);
    abort("connection, odom, IMU or RC unavailable/stale");
    return;
  }
  const bool active = state_ == State::TAKEOFF || state_ == State::HOVER || state_ == State::COMMAND ||
                      state_ == State::LAND || state_ == State::WAIT_DISARM;
  if (active && (fcu_.mode != "OFFBOARD" || (!fcu_.armed && state_ != State::WAIT_DISARM))) {
    abort("PX4 exited OFFBOARD or disarmed");
    return;
  }
  if ((state_ == State::WAIT_MODE || state_ == State::WAIT_ARM || state_ == State::WAIT_DISARM ||
       state_ == State::EXIT_MODE) && t - entered_ > operation_timeout_) {
    abort("flight operation confirmation timeout");
    return;
  }
  if (state_ == State::PRESTREAM && reference - entered_reference_ >= 1.2) {
    transition(State::WAIT_MODE);
    request_mode("OFFBOARD");
  } else if (state_ == State::WAIT_MODE && operation_done_ && operation_ok_ && fcu_.mode == "OFFBOARD") {
    if (fcu_.armed) {
      transition(takeoff_requested_ ? State::TAKEOFF : State::HOVER);
    } else {
      transition(State::WAIT_ARM);
      request_arm(true);
    }
  } else if (state_ == State::WAIT_ARM && operation_done_ && operation_ok_ && fcu_.armed &&
             fcu_.mode == "OFFBOARD") {
    transition(State::TAKEOFF);
  } else if (state_ == State::TAKEOFF) {
    if (reference - entered_reference_ > takeoff_timeout_) {
      abort("takeoff altitude timeout");
      return;
    }
    const double elapsed = reference - entered_reference_;
    target_.p.z() = started_z_ + std::min(height_, speed_ * std::max(0.0, elapsed - 3.0));
    target_.v.z() = (elapsed > 3.0 && target_.p.z() < started_z_ + height_) ? speed_ : 0;
    target_.a.z() = elapsed < 3.0 ? std::exp((elapsed - 3.0) * 6.0) * 7.0 - 7.0 : 0;
    if (elapsed > 3.0 && std::abs(odom_.p.z() - (started_z_ + height_)) < 0.1 && std::abs(odom_.v.z()) < 0.2) {
      hold();
      transition(State::HOVER);
    }
  } else if (state_ == State::HOVER) {
    if ((no_rc_ || rc_command_) && command_valid(t)) {
      transition(State::COMMAND);
      target_ = command_;
    } else {
      if (!no_rc_) {
        target_.p.x() += sticks_[1] * manual_velocity_ * dt;
        target_.p.y() += sticks_[0] * manual_velocity_ * dt;
        target_.p.z() += sticks_[2] * manual_velocity_ * dt;
        target_.yaw += sticks_[3] * manual_velocity_ * dt;
      }
      if (!trigger_sent_ && t - hover_at_ > 2.0) {
        geometry_msgs::msg::PoseStamped trigger;
        trigger.header.stamp = now();
        trigger.header.frame_id = odom_frame_;
        trigger.pose.position.x = odom_.p.x();
        trigger.pose.position.y = odom_.p.y();
        trigger.pose.position.z = odom_.p.z();
        trigger.pose.orientation.w = odom_.q.w();
        trigger.pose.orientation.x = odom_.q.x();
        trigger.pose.orientation.y = odom_.q.y();
        trigger.pose.orientation.z = odom_.q.z();
        trigger_pub_->publish(trigger);
        trigger_sent_ = true;
      }
    }
  } else if (state_ == State::COMMAND) {
    if (!command_valid(t) || (!no_rc_ && !rc_command_)) {
      hold();
      transition(State::HOVER);
    } else {
      target_ = command_;
    }
  } else if (state_ == State::LAND) {
    target_.p.z() = started_z_ - speed_ * (reference - entered_reference_);
    target_.v.z() = -speed_;
    if (!no_rc_ && !rc_command_) {
      hold();
      transition(State::HOVER);
    } else {
      const bool local_landed = target_.p.z() - odom_.p.z() < -0.5 && odom_.v.norm() < 0.1;
      if (!local_landed) {
        land_still_at_ = -1;
      } else if (land_still_at_ < 0) {
        land_still_at_ = reference;
      }
      if (land_still_at_ >= 0 && reference - land_still_at_ >= 3 && grounded(t)) {
        transition(State::WAIT_DISARM);
        request_arm(false);
      } else if (reference - entered_reference_ > 60) {
        abort("landing confirmation timeout");
        return;
      }
    }
  } else if (state_ == State::WAIT_DISARM && operation_done_ && operation_ok_ && !fcu_.armed) {
    transition(State::EXIT_MODE);
    request_mode(previous_mode_);
  } else if (state_ == State::EXIT_MODE && operation_done_ && operation_ok_ && fcu_.mode == previous_mode_) {
    transition(State::MANUAL);
    return;
  }
  if (state_ == State::ABORT) {
    return;
  }
  if (state_ == State::WAIT_DISARM || state_ == State::EXIT_MODE) {
    Output u;
    u.q = imu_q_;
    u.thrust = fcu_.armed ? 0.04 : 0;
    publish(u);
    return;
  }
  try {
    if ((state_ == State::HOVER || state_ == State::COMMAND) && imu_stamp_ != last_estimate_imu_stamp_) {
      last_estimate_imu_stamp_ = imu_stamp_;
      controller_.estimate(imu_a_.z(), reference);
    }
    publish(controller_.calculate(target_, odom_, imu_q_, reference));
  } catch (const std::exception &e) {
    abort(e.what());
  }
}

}  // namespace px4ctrl2
