#include "px4ctrl2/controller.hpp"
#include <iostream>
#include <stdexcept>
using namespace px4ctrl2;
void require(bool b,const char *message) {if(!b) throw std::runtime_error(message);}
int main() {
  LinearControl c;c.reset();Feedback f;Desired d;
  const auto stationary=c.calculate(d,f,Eigen::Quaterniond::Identity(),0);
  require(std::abs(stationary.thrust-0.5)<1e-9,"stationary hover thrust");
  require(stationary.q.angularDistance(Eigen::Quaterniond::Identity())<1e-9,"level hover attitude");
  d.p.x()=1;auto forward=c.calculate(d,f,Eigen::Quaterniond::Identity(),0.01);
  require((forward.q*Eigen::Vector3d::UnitZ()).x()>0,"forward command tilts thrust forward");
  d.p={1000,1000,1000};auto bounded=c.calculate(d,f,Eigen::Quaterniond::Identity(),0.02);
  require(std::abs(bounded.thrust-c.max_thrust)<1e-9,"upper thrust saturation");
  const auto rotation=bounded.q.toRotationMatrix();const double pitch=std::asin(-rotation(2,0));const double roll=std::atan2(rotation(2,1),rotation(2,2));
  require(std::abs(pitch)<=c.max_angle+1e-9 && std::abs(roll)<=c.max_angle+1e-9,"angle bounds");
  d.p={0,0,-1000};require(c.calculate(d,f,Eigen::Quaterniond::Identity(),0.03).thrust==0,"lower thrust saturation");
  d=Desired{};d.yaw=0.4;f.q=Eigen::AngleAxisd(0.3,Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond imu(Eigen::AngleAxisd(0.8,Eigen::Vector3d::UnitZ()));
  require(std::abs(yaw(c.calculate(d,f,imu,0.04).q)-0.9)<1e-9,"IMU/odom heading alignment");
  d.p.x()=NAN;bool threw=false;try{c.calculate(d,f,imu,0.05);}catch(const std::runtime_error&){threw=true;}
  require(threw,"invalid feedback rejected");
  c.reset();d=Desired{};f=Feedback{};c.calculate(d,f,Eigen::Quaterniond::Identity(),1);
  require(!c.estimate(9.81,1.02),"model does not use too recent samples");
  require(c.estimate(9.81,1.04),"model uses delayed samples");
  require(std::abs(c.scale()-19.62)<1e-9,"hover model stays stable");
  c.calculate(d,f,Eigen::Quaterniond::Identity(),2);
  require(!c.estimate(NAN,2.04),"nonfinite acceleration rejected");
  require(!c.estimate(9.81,1.5),"backward time discards model history");
  std::cout<<"Controller tests passed\n";
}
