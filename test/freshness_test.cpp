#include "px4ctrl2/freshness.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using px4ctrl2::freshSample;
void require(bool result, const char *message) { if (!result) throw std::runtime_error(message); }
int main() {
  require(!freshSample(false,.6,.1,.01,.5,2.), "hardware must retain wall deadline");
  require(freshSample(true,.6,.1,.01,.5,2.), "slow simulation caused false timeout");
  require(freshSample(true,1.1,.2,.03,.5,2.), "10 Hz simulation at low real-time factor rejected");
  require(!freshSample(true,.05,.6,.01,.5,2.), "fresh relay of stale odometry accepted");
  require(!freshSample(true,2.1,.1,.01,.5,2.), "stalled data stream accepted");
  require(!freshSample(true,.1,.1,2.1,.5,2.), "stalled clock accepted");
  require(!freshSample(true,.1,-1.,.01,.5,2.), "future/replayed clock accepted");
  require(!freshSample(true,.1,std::numeric_limits<double>::infinity(),.01,.5,2.), "missing clock accepted");
  std::cout << "Hardware/slow-simulation/stale-sensor/stalled-clock freshness checks passed\n";
}
