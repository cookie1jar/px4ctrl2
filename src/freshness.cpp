// Clock-domain freshness checks. GPL-3.0-only.
#include "px4ctrl2/freshness.hpp"
#include <cmath>

namespace px4ctrl2 {
bool freshSample(bool simulation, double wall_age, double simulation_age,
                 double clock_wall_age, double timeout, double wall_limit) {
  if (!std::isfinite(wall_age) || wall_age < 0.0) return false;
  if (!simulation) return wall_age < timeout;
  return std::isfinite(simulation_age) && std::isfinite(clock_wall_age) &&
         wall_age < wall_limit && clock_wall_age >= 0.0 && clock_wall_age < wall_limit &&
         simulation_age >= -0.1 && simulation_age < timeout;
}
}
