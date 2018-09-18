#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

namespace grpl {
namespace pf {
  namespace constants {
    constexpr double PI         = M_PI;
    constexpr double epsilon    = 1e-10;
    constexpr double almost_inf = 1e10;

    constexpr double default_acceptable_error = 1e-5;
  }  // namespace constants
}  // namespace pf
}  // namespace grpl