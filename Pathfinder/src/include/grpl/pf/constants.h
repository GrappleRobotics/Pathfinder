#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

namespace grpl {
namespace pf {
  const size_t POSITION     = 0;
  const size_t VELOCITY     = 1;
  const size_t ACCELERATION = 2;
  const size_t JERK = 3;

  namespace constants {
    constexpr double PI         = M_PI;
    constexpr double epsilon    = 1e-10;
    constexpr double almost_inf = 1e10;

    constexpr double default_acceptable_error = 1e-5;

    const size_t profile_segment_order = 3; // Pos, Vel, Acc
    const size_t profile_limits_order  = 4; // <>, Vel, Acc, Jerk
  }  // namespace constants
}  // namespace pf
}  // namespace grpl