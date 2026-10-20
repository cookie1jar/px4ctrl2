#pragma once
// Clock-domain freshness checks. GPL-3.0-only.

namespace px4ctrl2 {
bool freshSample(bool simulation, double wall_age, double simulation_age,
                 double clock_wall_age, double timeout, double wall_limit);
}
