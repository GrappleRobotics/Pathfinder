#pragma once

#include "grpl/path/path.h"

namespace grpl {
namespace path {

  template <typename VECTYPE, typename UNIT>
  class hermite : public path<VECTYPE, UNIT> {
  public:
    struct waypoint {
      VECTYPE point, tangent;
    };

    hermite(waypoint &wp0, waypoint &wp1, unsigned int arclength_samples) 
      : _wp0(wp0), _wp1(wp1), _al_samples(arclength_samples) {}

    VECTYPE calculate(double t) const override {
      double h00 = (2 * t*t*t - 3 * t*t + 1);
      double h10 = (t*t*t - 2 * t*t + t);
      double h01 = (-2 * t*t*t + 3 * t*t);
      double h11 = (t*t*t - t*t);

      return h00*_wp0.point + h10*_wp0.tangent + h01*_wp1.point + h11*_wp1.tangent;
    }

    VECTYPE calculate_slope(double t) const override {
      double h00 = (6 * t*t - 6 * t);
      double h10 = (3*t*t - 4 * t + 1);
      double h01 = (-6 * t*t + 6 * t);
      double h11 = (3 * t*t - 2 * t);

      return h00*_wp0.point + h10*_wp0.tangent + h01*_wp1.point + h11*_wp1.tangent;
    }

    UNIT calculate_arc_length() const override {
      double t = 0, dt = (1.0 / _al_samples);
      double dydt = static_cast<double>(calculate_slope(t).mag());
      double integrand = 0;
      double arc_length = 0;
      double last_integrand = sqrt(1 + dydt*dydt) * dt;

      for (t = 0; t <= 1; t += dt) {
        dydt = static_cast<double>(calculate_slope(t).mag());
        integrand = sqrt(1 + dydt*dydt) * dt;
        arc_length += (integrand + last_integrand) / 2;
        last_integrand = integrand;
      }

      return static_cast<UNIT>(arc_length);
    }

  protected:
    waypoint _wp0, _wp1;
    unsigned int _al_samples = 10000;
  };

}
}