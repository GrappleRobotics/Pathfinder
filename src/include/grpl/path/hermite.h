#pragma once

#include "grpl/vec.h"

#include "grpl/units.h"
#include <cmath>

namespace grpl {
namespace path {

  template <typename VECTYPE>
  class hermite {
  public:
    struct waypoint {
      VECTYPE point, tangent;
    };

    hermite(waypoint &wp0, waypoint &wp1) : _wp0(wp0), _wp1(wp1) {}

    VECTYPE calculate(double t) const {
      double h00 = (2 * t*t*t - 3 * t*t + 1);
      double h10 = (t*t*t - 2 * t*t + t);
      double h01 = (-2 * t*t*t + 3 * t*t);
      double h11 = (t*t*t - t*t);

      return h00*_wp0.point + h10*_wp0.tangent + h01*_wp1.point + h11*_wp1.tangent;
    }
  protected:
    waypoint _wp0, _wp1;
  };

}
}