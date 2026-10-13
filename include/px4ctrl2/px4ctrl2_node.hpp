#pragma once
// ROS 2 migration of Fast-Drone px4ctrl. GPL-3.0-only.
#include "px4ctrl2/controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "px4ctrl2/msg/position_command.hpp"
#include "px4ctrl2/msg/takeoff_land.hpp"
#include "px4ctrl2/msg/controller_debug.hpp"
#include <chrono>
#include <array>
#include <functional>
using namespace std::chrono_literals;
// ROS 2 migration of Fast-Drone px4ctrl. GPL-3.0-only.
namespace px4ctrl2 {
class Node : public rclcpp::Node {
  enum class State { MANUAL, PRESTREAM, WAIT_MODE, WAIT_ARM, TAKEOFF, HOVER, COMMAND, LAND, WAIT_DISARM, EXIT_MODE, ABORT };
  State state_{State::MANUAL};
  LinearControl controller_;
  Feedback odom_;
  Desired command_, target_;
  Eigen::Quaterniond imu_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d imu_a_{Eigen::Vector3d::Zero()};
  mavros_msgs::msg::State fcu_;
  mavros_msgs::msg::ExtendedState extended_;
  std::array<double,4> sticks_{};
  std::array<bool,4> rc_reverse_{};
  bool rc_hover_{false}, rc_command_{false}, no_rc_, auto_arm_, velocity_body_;
  bool have_odom_{false}, have_imu_{false}, have_state_{false}, have_rc_{false}, have_extended_{false}, have_command_{false};
  bool takeoff_requested_{true};
  int64_t odom_stamp_{-1}, imu_stamp_{-1};
  bool trigger_sent_{false}, operation_done_{false}, operation_ok_{false};
  uint64_t generation_{0};
  double odom_at_{0}, imu_at_{0}, state_at_{0}, rc_at_{0}, extended_at_{0}, command_at_{0};
  double entered_{0}, last_tick_{0}, started_z_{0}, land_still_at_{-1}, hover_at_{0};
  double frequency_, feedback_timeout_, state_timeout_, command_timeout_, operation_timeout_, takeoff_timeout_;
  double height_, speed_, manual_velocity_;
  std::string previous_mode_{"POSCTL"}, abort_reason_;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
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
  std::string odom_frame_;
  double monotonic() {return steady_.now().seconds();}
  static Eigen::Quaterniond quaternion(const geometry_msgs::msg::Quaternion &q) {return {q.w,q.x,q.y,q.z};}
  static const char *name(State s) {
    switch(s) {
      case State::MANUAL:return "MANUAL_CTRL"; case State::PRESTREAM:return "PRESTREAM";
      case State::WAIT_MODE:return "WAIT_OFFBOARD"; case State::WAIT_ARM:return "WAIT_ARM";
      case State::TAKEOFF:return "AUTO_TAKEOFF"; case State::HOVER:return "AUTO_HOVER";
      case State::COMMAND:return "CMD_CTRL"; case State::LAND:return "AUTO_LAND";
      case State::WAIT_DISARM:return "WAIT_DISARM"; case State::EXIT_MODE:return "EXIT_OFFBOARD";
      case State::ABORT:return "ABORT";
    } return "UNKNOWN";
  }
  void transition(State s) {
    RCLCPP_INFO(get_logger(), "%s -> %s",name(state_),name(s));
    state_=s; entered_=monotonic(); operation_done_=false; operation_ok_=false;
  }
  bool feedback(double t) const {return have_odom_ && have_imu_ && t-odom_at_<feedback_timeout_ && t-imu_at_<feedback_timeout_;}
  bool connected(double t) const {return have_state_ && fcu_.connected && t-state_at_<state_timeout_;}
  bool rc_valid(double t) const {return no_rc_ || (have_rc_ && t-rc_at_<feedback_timeout_ && rc_hover_);}
  bool command_valid(double t) const {return have_command_ && t-command_at_<command_timeout_;}
  bool grounded(double t) const {return have_extended_ && t-extended_at_<state_timeout_ && extended_.landed_state==mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;}
  void hold() {target_.p=odom_.p; target_.v.setZero(); target_.a.setZero(); target_.yaw=yaw(odom_.q); hover_at_=monotonic(); trigger_sent_=false;}
  void abort(const std::string &reason) {
    abort_reason_=reason; ++generation_; mode_client_->prune_pending_requests(); arm_client_->prune_pending_requests();
    transition(State::ABORT);
    RCLCPP_ERROR(get_logger(),"Control stopped: %s. PX4 retains failsafe authority; no automatic OFFBOARD retry.",reason.c_str());
  }
  void request_mode(const std::string &mode) {
    const auto token=++generation_;
    auto req=std::make_shared<mavros_msgs::srv::SetMode::Request>(); req->custom_mode=mode;
    mode_client_->async_send_request(req,[this,token](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future){
      if(token!=generation_) return;
      operation_done_=true; operation_ok_=future.get()->mode_sent;
      if(!operation_ok_) abort("mode request rejected");
    });
  }
  void request_arm(bool value) {
    const auto token=++generation_;
    auto req=std::make_shared<mavros_msgs::srv::CommandBool::Request>();req->value=value;
    arm_client_->async_send_request(req,[this,token](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future){
      if(token!=generation_) return;
      const auto result=future.get(); operation_done_=true;operation_ok_=result->success;
      if(!operation_ok_) abort("arm/disarm request rejected, MAV_RESULT="+std::to_string(result->result));
    });
  }
  std::pair<bool,std::string> takeoff() {
    const double t=monotonic();
    if(state_!=State::MANUAL && state_!=State::ABORT) return {false,"controller is busy"};
    if(!connected(t)||!feedback(t)) return {false,"fresh connected state, odom and IMU required"};
    if(!grounded(t)) return {false,"fresh PX4 on-ground confirmation required"};
    if(!rc_valid(t)) return {false,"RC hover switch and fresh RC required"};
    if(!no_rc_ && (!rc_command_ || std::any_of(sticks_.begin(),sticks_.end(),[](double v){return std::abs(v)>1e-5;}))) return {false,"RC command switch and centered sticks required"};
    if(command_valid(t)) return {false,"stop trajectory commands before takeoff"};
    if(odom_.v.norm()>0.1) return {false,"non-static odometry"};
    if(!mode_client_->service_is_ready()||!arm_client_->service_is_ready()) return {false,"MAVROS mode/arming services unavailable"};
    if(fcu_.mode=="OFFBOARD") return {false,"already OFFBOARD; release other control source first"};
    if(!auto_arm_ && !fcu_.armed) return {false,"auto arm disabled; arm externally first"};
    takeoff_requested_=true; ++generation_; controller_.reset(); hold();started_z_=odom_.p.z();
    previous_mode_=fcu_.mode.empty()?"POSCTL":fcu_.mode;
    transition(State::PRESTREAM); return {true,"takeoff accepted; await state and actual flight feedback"};
  }
  std::pair<bool,std::string> land() {
    if(state_!=State::HOVER) return {false,"land requires AUTO_HOVER; call hover first and stop trajectory stream"};
    hold();started_z_=odom_.p.z();land_still_at_=-1;transition(State::LAND);
    return {true,"landing accepted"};
  }
  std::pair<bool,std::string> hover() {
    if(state_==State::MANUAL || state_==State::ABORT) {
      const double t=monotonic();
      if(!connected(t)||!feedback(t)||!rc_valid(t)||!fcu_.armed||grounded(t)) return {false,"in-flight hover takeover requires armed vehicle, fresh feedback and RC authorization"};
      if(fcu_.mode=="OFFBOARD") return {false,"another OFFBOARD source must release control first"};
      if(!mode_client_->service_is_ready()) return {false,"MAVROS mode service unavailable"};
      takeoff_requested_=false; previous_mode_=fcu_.mode; ++generation_;controller_.reset();hold();transition(State::PRESTREAM);
      return {true,"hover takeover accepted"};
    }
    if(state_!=State::COMMAND && state_!=State::HOVER) return {false,"hover requires active hover/command control"};
    hold();have_command_=false;transition(State::HOVER);return {true,"hover accepted; stop trajectory stream before landing"};
  }
  void publish(const Output &u) {
    mavros_msgs::msg::AttitudeTarget out;out.header.stamp=now();out.header.frame_id="base_link";
    out.type_mask=out.IGNORE_ROLL_RATE|out.IGNORE_PITCH_RATE|out.IGNORE_YAW_RATE;
    out.orientation.w=u.q.w();out.orientation.x=u.q.x();out.orientation.y=u.q.y();out.orientation.z=u.q.z();
    out.thrust=static_cast<float>(u.thrust);attitude_pub_->publish(out);
    msg::ControllerDebug d;d.header=out.header;d.state=name(state_);d.desired_orientation=out.orientation;
    d.desired_acceleration.x=u.acceleration.x();d.desired_acceleration.y=u.acceleration.y();d.desired_acceleration.z=u.acceleration.z();
    d.thrust=u.thrust;d.thrust_to_acceleration=controller_.scale();debug_pub_->publish(d);
  }
  void tick() {
    const double t=monotonic();const double dt=std::clamp(t-last_tick_,0.0,0.05);last_tick_=t;
    std_msgs::msg::String state_msg;state_msg.data=name(state_);state_pub_->publish(state_msg);
    if(state_==State::MANUAL || state_==State::ABORT) return;
    if(!connected(t)||!feedback(t)||!rc_valid(t)) {
      RCLCPP_ERROR(get_logger(),
        "Feedback failure: state_valid=%d connected=%d state_age=%.3fs (limit %.3fs); "
        "odom_valid=%d odom_age=%.3fs; imu_valid=%d imu_age=%.3fs (limit %.3fs); "
        "no_rc=%d rc_valid=%d rc_age=%.3fs",
        have_state_,fcu_.connected,t-state_at_,state_timeout_,
        have_odom_,t-odom_at_,have_imu_,t-imu_at_,feedback_timeout_,
        no_rc_,rc_valid(t),t-rc_at_);
      abort("connection, odom, IMU or RC unavailable/stale");return;
    }
    const bool active=state_==State::TAKEOFF||state_==State::HOVER||state_==State::COMMAND||state_==State::LAND||state_==State::WAIT_DISARM;
    if(active && (fcu_.mode!="OFFBOARD" || (!fcu_.armed && state_!=State::WAIT_DISARM))) {abort("PX4 exited OFFBOARD or disarmed");return;}
    if((state_==State::WAIT_MODE||state_==State::WAIT_ARM||state_==State::WAIT_DISARM||state_==State::EXIT_MODE) && t-entered_>operation_timeout_) {abort("flight operation confirmation timeout");return;}
    if(state_==State::PRESTREAM && t-entered_>=1.2) {transition(State::WAIT_MODE);request_mode("OFFBOARD");}
    else if(state_==State::WAIT_MODE && operation_done_ && operation_ok_ && fcu_.mode=="OFFBOARD") {
      if(fcu_.armed) transition(takeoff_requested_?State::TAKEOFF:State::HOVER);
      else {transition(State::WAIT_ARM);request_arm(true);}
    } else if(state_==State::WAIT_ARM && operation_done_ && operation_ok_ && fcu_.armed && fcu_.mode=="OFFBOARD") transition(State::TAKEOFF);
    else if(state_==State::TAKEOFF) {
      if(t-entered_>takeoff_timeout_) {abort("takeoff altitude timeout");return;}
      const double elapsed=t-entered_;
      target_.p.z()=started_z_+std::min(height_,speed_*std::max(0.0,elapsed-3.0));
      target_.v.z()=(elapsed>3.0 && target_.p.z()<started_z_+height_)?speed_:0;
      target_.a.z()=elapsed<3.0?std::exp((elapsed-3.0)*6.0)*7.0-7.0:0;
      if(elapsed>3.0 && std::abs(odom_.p.z()-(started_z_+height_))<0.1 && std::abs(odom_.v.z())<0.2) {hold();transition(State::HOVER);}
    } else if(state_==State::HOVER) {
      if((no_rc_||rc_command_) && command_valid(t)) {transition(State::COMMAND);target_=command_;}
      else {
        if(!no_rc_) {target_.p.x()+=sticks_[1]*manual_velocity_*dt;target_.p.y()+=sticks_[0]*manual_velocity_*dt;target_.p.z()+=sticks_[2]*manual_velocity_*dt;target_.yaw+=sticks_[3]*manual_velocity_*dt;}
        if(!trigger_sent_ && t-hover_at_>2.0) {
          geometry_msgs::msg::PoseStamped trigger;trigger.header.stamp=now();trigger.header.frame_id=odom_frame_;
          trigger.pose.position.x=odom_.p.x();trigger.pose.position.y=odom_.p.y();trigger.pose.position.z=odom_.p.z();
          trigger.pose.orientation.w=odom_.q.w();trigger.pose.orientation.x=odom_.q.x();trigger.pose.orientation.y=odom_.q.y();trigger.pose.orientation.z=odom_.q.z();
          trigger_pub_->publish(trigger);trigger_sent_=true;
        }
      }
    } else if(state_==State::COMMAND) {
      if(!command_valid(t)||(!no_rc_&&!rc_command_)) {hold();transition(State::HOVER);} else target_=command_;
    } else if(state_==State::LAND) {
      target_.p.z()=started_z_-speed_*(t-entered_);target_.v.z()=-speed_;
      if(!no_rc_&&!rc_command_) {hold();transition(State::HOVER);}
      else {
        const bool local_landed=target_.p.z()-odom_.p.z()<-0.5 && odom_.v.norm()<0.1;
        if(!local_landed) land_still_at_=-1;else if(land_still_at_<0) land_still_at_=t;
        if(land_still_at_>=0 && t-land_still_at_>=3 && grounded(t)) {transition(State::WAIT_DISARM);request_arm(false);}
        else if(t-entered_>60) {abort("landing confirmation timeout");return;}
      }
    } else if(state_==State::WAIT_DISARM && operation_done_ && operation_ok_ && !fcu_.armed) {
      transition(State::EXIT_MODE);request_mode(previous_mode_);
    } else if(state_==State::EXIT_MODE && operation_done_ && operation_ok_ && fcu_.mode==previous_mode_) {transition(State::MANUAL);return;}
    if(state_==State::ABORT) return;
    if(state_==State::WAIT_DISARM||state_==State::EXIT_MODE) {
      Output u;u.q=imu_q_;u.thrust=fcu_.armed?0.04:0;publish(u);return;
    }
    try {
      if(state_==State::HOVER||state_==State::COMMAND) controller_.estimate(imu_a_.z(),t);
      publish(controller_.calculate(target_,odom_,imu_q_,t));
    } catch(const std::exception &e) {abort(e.what());}
  }
public:
  Node():rclcpp::Node("px4ctrl2") {
    frequency_=declare_parameter("control_frequency",100.0);
    feedback_timeout_=declare_parameter("feedback_timeout",0.5);state_timeout_=declare_parameter("state_timeout",2.0);
    command_timeout_=declare_parameter("command_timeout",0.5);operation_timeout_=declare_parameter("operation_timeout",10.0);
    takeoff_timeout_=declare_parameter("takeoff_timeout",30.0);height_=declare_parameter("takeoff_height",1.0);
    speed_=declare_parameter("takeoff_land_speed",0.3);manual_velocity_=declare_parameter("max_manual_velocity",0.8);
    rc_reverse_[0]=declare_parameter("rc_reverse.roll",false);rc_reverse_[1]=declare_parameter("rc_reverse.pitch",false);
    rc_reverse_[2]=declare_parameter("rc_reverse.throttle",false);rc_reverse_[3]=declare_parameter("rc_reverse.yaw",false);
    no_rc_=declare_parameter("no_rc",true);auto_arm_=declare_parameter("enable_auto_arm",true);velocity_body_=declare_parameter("velocity_in_body",false);
    controller_.hover=declare_parameter("hover_percentage",0.5);controller_.max_thrust=declare_parameter("max_thrust",0.8);
    controller_.max_angle=declare_parameter("max_angle_deg",25.0)*std::acos(-1.0)/180;
    const auto kp=declare_parameter("kp",std::vector<double>{1.5,1.5,1.5});const auto kv=declare_parameter("kv",std::vector<double>{1.5,1.5,1.5});
    if(kp.size()!=3||kv.size()!=3) throw std::invalid_argument("kp/kv require three values");
    controller_.kp=Eigen::Vector3d(kp[0],kp[1],kp[2]);controller_.kv=Eigen::Vector3d(kv[0],kv[1],kv[2]);
    const std::array<double,12> numbers{frequency_,feedback_timeout_,state_timeout_,command_timeout_,operation_timeout_,takeoff_timeout_,height_,speed_,manual_velocity_,controller_.hover,controller_.max_thrust,controller_.max_angle};
    for(double n:numbers) if(!std::isfinite(n)||n<=0) throw std::invalid_argument("positive finite parameters required");
    if(frequency_<10||frequency_>500||controller_.hover>=controller_.max_thrust||controller_.max_thrust>1||controller_.max_angle>0.8 || !controller_.kp.allFinite() || !controller_.kv.allFinite() || (controller_.kp.array()<0).any() || (controller_.kv.array()<0).any()) throw std::invalid_argument("invalid controller bounds/gains");
    controller_.reset();
    const auto sensor_qos=rclcpp::SensorDataQoS();
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic",std::string("/odom")),sensor_qos,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      const int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
      if(stamp<=0 || stamp==odom_stamp_) return;
      if(stamp<odom_stamp_) {have_odom_=false;odom_stamp_=stamp;if(state_!=State::MANUAL&&state_!=State::ABORT) abort("odometry time moved backwards");return;}
      odom_stamp_=stamp;
      Feedback f;f.p={m->pose.pose.position.x,m->pose.pose.position.y,m->pose.pose.position.z};f.v={m->twist.twist.linear.x,m->twist.twist.linear.y,m->twist.twist.linear.z};f.q=quaternion(m->pose.pose.orientation);
      if(!f.p.allFinite()||!f.v.allFinite()||!validQuaternion(f.q)||m->header.frame_id.empty()) {have_odom_=false;return;}
      if(have_odom_ && m->header.frame_id!=odom_frame_ && state_!=State::MANUAL && state_!=State::ABORT) {have_odom_=false;abort("odometry frame changed during control");return;}
      f.q.normalize();if(velocity_body_) f.v=f.q*f.v;odom_=f;odom_frame_=m->header.frame_id;have_odom_=true;odom_at_=monotonic();
    });
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data",sensor_qos,[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
      const int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
      if(stamp<=0 || stamp==imu_stamp_) return;
      if(stamp<imu_stamp_) {have_imu_=false;imu_stamp_=stamp;if(state_!=State::MANUAL&&state_!=State::ABORT) abort("IMU time moved backwards");return;}
      imu_stamp_=stamp;
      const auto q=quaternion(m->orientation);Eigen::Vector3d a(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z);
      if(m->orientation_covariance[0]<0||!validQuaternion(q)||!a.allFinite()) {have_imu_=false;return;}
      imu_q_=q.normalized();imu_a_=a;have_imu_=true;imu_at_=monotonic();
    });
    state_sub_=create_subscription<mavros_msgs::msg::State>("/mavros/state",sensor_qos,[this](mavros_msgs::msg::State::ConstSharedPtr m){fcu_=*m;have_state_=true;state_at_=monotonic();});
    extended_sub_=create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state",sensor_qos,[this](mavros_msgs::msg::ExtendedState::ConstSharedPtr m){extended_=*m;have_extended_=true;extended_at_=monotonic();});
    if(!no_rc_) rc_sub_=create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",sensor_qos,[this](mavros_msgs::msg::RCIn::ConstSharedPtr m){
      if(m->channels.size()<8) {have_rc_=false;return;}
      for(size_t i=0;i<4;++i){double v=std::clamp((m->channels[i]-1500.0)/500.0,-1.0,1.0);sticks_[i]=(std::abs(v)<=0.25?0:std::copysign((std::abs(v)-0.25)/0.75,v))*(rc_reverse_[i]?-1:1);}
      rc_hover_=m->channels[4]>1750;rc_command_=m->channels[5]>1750;have_rc_=true;rc_at_=monotonic();
    });
    command_sub_=create_subscription<msg::PositionCommand>(declare_parameter("command_topic",std::string("/px4ctrl2/command")),rclcpp::QoS(10),[this](msg::PositionCommand::ConstSharedPtr m){
      if(state_!=State::HOVER && state_!=State::COMMAND) {have_command_=false;return;}
      Desired d;d.p={m->position.x,m->position.y,m->position.z};d.v={m->velocity.x,m->velocity.y,m->velocity.z};d.a={m->acceleration.x,m->acceleration.y,m->acceleration.z};d.yaw=m->yaw;
      if(m->header.frame_id!=odom_frame_||!d.p.allFinite()||!d.v.allFinite()||!d.a.allFinite()||!std::isfinite(d.yaw)) {have_command_=false;RCLCPP_WARN(get_logger(),"Rejected command: frame/values invalid");return;}
      command_=d;have_command_=true;command_at_=monotonic();
    });
    takeoff_sub_=create_subscription<msg::TakeoffLand>("~/takeoff_land",rclcpp::QoS(10),[this](msg::TakeoffLand::ConstSharedPtr m){
      std::pair<bool,std::string> r;
      if(m->takeoff_land_cmd==msg::TakeoffLand::TAKEOFF) r=takeoff();else if(m->takeoff_land_cmd==msg::TakeoffLand::LAND) r=land();else r={false,"unknown takeoff_land command"};
      RCLCPP_INFO(get_logger(),"%s: %s",r.first?"accepted":"rejected",r.second.c_str());
    });
    attitude_pub_=create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude",rclcpp::QoS(10));
    trigger_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/traj_start_trigger",10);
    debug_pub_=create_publisher<msg::ControllerDebug>("~/debug",10);state_pub_=create_publisher<std_msgs::msg::String>("~/state",rclcpp::QoS(1).transient_local());
    mode_client_=create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");arm_client_=create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto bind_service=[this](const std::string &service,auto action) {
      return create_service<std_srvs::srv::Trigger>(service,[this,action](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){const auto r=(this->*action)();res->success=r.first;res->message=r.second;});
    };
    takeoff_srv_=bind_service("~/takeoff",&Node::takeoff);land_srv_=bind_service("~/land",&Node::land);hover_srv_=bind_service("~/hover",&Node::hover);
    last_tick_=monotonic();timer_=create_wall_timer(std::chrono::duration<double>(1.0/frequency_),[this]{tick();});
    RCLCPP_INFO(get_logger(),"px4ctrl2 ready. No automatic launch-time arming. MAVROS + valid external odom required.");
  }
};
}
