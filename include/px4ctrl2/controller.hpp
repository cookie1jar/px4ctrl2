#pragma once
// Derived from px4ctrl/src/controller.cpp (GPL-3.0), commit 7f3b0c8.

#include <Eigen/Geometry>
#include <deque>
#include <utility>

namespace px4ctrl2 {

struct Feedback {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
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

double yaw(const Eigen::Quaterniond &q);
bool validQuaternion(const Eigen::Quaterniond &q);

class LinearControl {
 public:
  Eigen::Vector3d kp{1.5, 1.5, 1.5};
  Eigen::Vector3d kv{1.5, 1.5, 1.5};
  double gravity{9.81};
  double hover{0.5};
  double max_angle{0.436332};
  double max_thrust{0.8};

  void reset();
  double scale() const;
  Output calculate(const Desired &des, const Feedback &odom,
                   const Eigen::Quaterniond &imu_q, double now);

  // Includes gravity; acceleration is expressed in body FLU (MAVROS imu/data).
  bool estimate(double acceleration_z, double now);

 private:
  double scale_{19.62};
  double covariance_{1e6};
  std::deque<std::pair<double, double>> history_;
};

}  // namespace px4ctrl2
