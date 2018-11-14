#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <cstddef>

namespace grpl {
namespace pf {
  //! Vector index for position (metres)
  const size_t POSITION     = 0;
  //! Vector index for velocity (metres per second, ms^-1)
  const size_t VELOCITY     = 1;
  //! Vector index for acceleration (metres per second per second, ms^-2)
  const size_t ACCELERATION = 2;
  //! Vector index for jerk (ms^-3)
  const size_t JERK         = 3;

  namespace constants {
    constexpr double PI         = M_PI;
    //! Comparation threshold for floating point numbers
    constexpr double epsilon    = 1e-10;
    //! Comparison threshold for inf
    constexpr double almost_inf = 1e10;

    //! Default acceptable error when comparing setpoints and process variables.
    constexpr double default_acceptable_error = 1e-5;

    const size_t profile_kinematics_order = 3;  // Pos, Vel, Acc
    const size_t profile_limits_order  = 4;  // <>, Vel, Acc, Jerk
  }                                          // namespace constants
}  // namespace pf
}  // namespace grpl