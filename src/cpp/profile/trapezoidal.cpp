#include "grpl/profile/trapezoidal.h"

#include <cmath>

using namespace grpl::profile::intern;

void trapezoidal_internal::calculate_single(trapezoidal_internal::segment_raw *out, trapezoidal_internal::segment_raw *last, double time) const {
  // In the case 'out' and 'last' point to the same memory address, we need to cache
  // these in their own variables.
  // We could store these in another struct (copied), but there's no real advantage
  double l_time = last->time, l_dist = last->dist, l_vel = last->vel, l_acc = last->acc;
  double dt = time - l_time;

  out->time = time;

  double error = last->dist - _goal;
  double accel = (l_dist < _goal ? _acc_max : -_acc_max);
  double v_projected = l_vel + (accel * dt);
  v_projected = v_projected > _vel_max ? _vel_max : v_projected < -_vel_max ? -_vel_max : v_projected;
  double decel_time = v_projected / accel;
  double decel_dist = v_projected * decel_time - 0.5*accel*decel_time*decel_time;
  double decel_error = l_dist + decel_dist - _goal;

  out->acc = accel;
  // Do we end up on the other side of the goal if we start decelerating now?
  bool decel_change_error_phase = (error > 0 && decel_error < 0) || (error < 0 && decel_error > 0);
  // Are we not currently decelerating?
  bool decel_not_in_progress = (l_dist < _goal && l_vel > 0) || (l_dist > _goal && l_vel < 0);

  if (decel_change_error_phase && decel_not_in_progress) {
    // If we need to decelerate, invert the acceleration
    out->acc = -out->acc;
  }
  double vel = l_vel + (out->acc * dt);
  out->vel = vel > _vel_max ? _vel_max : vel < -_vel_max ? -_vel_max : vel;
  out->dist = l_dist + (l_vel * dt) + (0.5 * out->acc * dt * dt);
}