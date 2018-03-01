#pragma once

#include "grpl/profile/profile.h"

namespace grpl {
namespace profile {

  template <typename UNIT_DIST, typename UNIT_TIME, typename UNIT_VEL, typename UNIT_ACC>
  class trapezoidal : public profile<UNIT_DIST, UNIT_TIME, UNIT_VEL, UNIT_ACC> {
  public:
    void calculate_single(segment *out, segment *last, UNIT_TIME time) const override {
      segment cache = { last->time, last->dist, last->vel, last->acc };
      UNIT_TIME dt = time - cache.time;
      
      out->time = time;

      UNIT_DIST error = cache.dist - _goal;
      UNIT_ACC accel = (cache.dist < _goal ? _acc_max : -_acc_max);
      UNIT_VEL v_projected = cache.vel + (accel * dt);
      v_projected = v_projected > _vel_max ? _vel_max : v_projected < -_vel_max ? -_vel_max : v_projected;
      UNIT_TIME decel_time = v_projected / accel;
      UNIT_DIST decel_dist = v_projected * decel_time - 0.5*accel*decel_time*decel_time;
      UNIT_DIST decel_error = cache.dist + decel_dist - _goal;

      out->acc = accel;
      bool decel_change_error_phase = (static_cast<double>(error) > 0 && static_cast<double>(decel_error) < 0) || (static_cast<double>(error) < 0 && static_cast<double>(decel_error) > 0);
      bool decel_not_in_progress = (cache.dist < _goal && static_cast<double>(cache.vel) > 0) || (cache.dist > _goal && static_cast<double>(cache.vel) < 0);

      if (decel_change_error_phase && decel_not_in_progress) {
        out->acc = -out->acc;
      }
      UNIT_VEL vel = cache.vel + (out->acc * dt);
      out->vel = vel > _vel_max ? _vel_max : vel < -_vel_max ? -_vel_max : vel;
      out->dist = cache.dist + (cache.vel * dt) + (0.5 * out->acc * dt *dt);
    }
  };

} // namespace profile
} // namespace grpl