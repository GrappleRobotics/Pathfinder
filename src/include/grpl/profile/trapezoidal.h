#pragma once

#include "grpl/profile/profile.h"

namespace grpl {
namespace profile {

  namespace intern {
    class trapezoidal_internal : public profile_internal {
    public:
      void configure_raw(double vel_max, double acc_max) {
        _vel_max = vel_max;
        _acc_max = acc_max;
      }

      void calculate_single(segment_raw *out, segment_raw *last, double time) override;
    protected:
      double _vel_max, _acc_max;
    };
  }

  template <typename UNIT_DIST, typename UNIT_TIME, typename UNIT_VEL, typename UNIT_ACC>
  class trapezoidal : public profile<intern::trapezoidal_internal, UNIT_DIST, UNIT_TIME, UNIT_VEL, UNIT_ACC> {
  public:
    void configure(UNIT_VEL vel_max, UNIT_ACC acc_max) {
      intern::trapezoidal_internal::configure_raw(static_cast<double>(vel_max), static_cast<double>(acc_max));
    }
  };

} // namespace profile
} // namespace grpl