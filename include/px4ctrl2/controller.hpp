#pragma once
// Derived from px4ctrl/src/controller.cpp (GPL-3.0), commit 7f3b0c8.
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
namespace px4ctrl2 {
struct Feedback {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()}, v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
};
struct Desired : Feedback {
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  double yaw{0};
};
struct Output {
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  double thrust{0};
};
inline double yaw(const Eigen::Quaterniond &q) {
  return std::atan2(2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z()));
}
inline bool validQuaternion(const Eigen::Quaterniond &q) {
  return q.coeffs().allFinite() && q.norm() > 0.5 && q.norm() < 1.5;
}
class LinearControl {
public:
  Eigen::Vector3d kp{1.5,1.5,1.5}, kv{1.5,1.5,1.5};
  double gravity{9.81}, hover{0.5}, max_angle{0.436332}, max_thrust{0.8};
  void reset() { scale_ = gravity/hover; covariance_ = 1e6; history_.clear(); }
  double scale() const { return scale_; }
  Output calculate(const Desired &des, const Feedback &odom,
                   const Eigen::Quaterniond &imu_q, double now) {
    if (!des.p.allFinite() || !des.v.allFinite() || !des.a.allFinite() ||
        !odom.p.allFinite() || !odom.v.allFinite() || !validQuaternion(odom.q) ||
        !validQuaternion(imu_q) || !std::isfinite(des.yaw))
      throw std::runtime_error("invalid control feedback or desired state");
    Output u;
    u.acceleration = des.a + kv.cwiseProduct(des.v-odom.v) + kp.cwiseProduct(des.p-odom.p);
    u.acceleration.z() += gravity;
    const double heading = yaw(odom.q);
    double roll = (u.acceleration.x()*std::sin(heading)-u.acceleration.y()*std::cos(heading))/gravity;
    double pitch = (u.acceleration.x()*std::cos(heading)+u.acceleration.y()*std::sin(heading))/gravity;
    roll = std::clamp(roll,-max_angle,max_angle);
    pitch = std::clamp(pitch,-max_angle,max_angle);
    Eigen::Quaterniond desired_q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
    u.q = (imu_q.normalized()*odom.q.normalized().inverse()*desired_q).normalized();
    u.thrust = std::clamp(u.acceleration.z()/scale_,0.0,max_thrust);
    history_.emplace_back(now,u.thrust);
    while(history_.size()>100) history_.pop_front();
    return u;
  }
  // IMU acceleration includes gravity, expressed in body FLU (MAVROS imu/data).
  bool estimate(double acceleration_z, double now) {
    if (!std::isfinite(acceleration_z)) return false;
    while (!history_.empty()) {
      const auto sample=history_.front(); const double age=now-sample.first;
      if(age<0) {history_.clear(); return false;}
      if(age>0.045) {history_.pop_front(); continue;}
      if(age<0.035) return false;
      history_.pop_front();
      if(sample.second<0.05 || sample.second>=max_thrust) return false;
      const double gain=covariance_*sample.second/(0.998+sample.second*covariance_*sample.second);
      const double candidate=scale_+gain*(acceleration_z-sample.second*scale_);
      if(!std::isfinite(candidate) || candidate<gravity/0.9 || candidate>gravity/0.05) return false;
      scale_=candidate;
      covariance_=(1-gain*sample.second)*covariance_/0.998;
      return true;
    }
    return false;
  }
private:
  double scale_{19.62}, covariance_{1e6};
  std::deque<std::pair<double,double>> history_;
};
} // namespace px4ctrl2
