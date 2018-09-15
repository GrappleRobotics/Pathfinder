#pragma once

#include "grpl/profile/profile.h"

#include <iostream>

namespace grpl {
namespace profile {

  class trapezoidal : public profile<ACCELERATION> {
   public:
    segment_t calculate(segment_t &last, double time) const override {
      double dt          = time - last.time;
      double timestep    = dt;
      int    slice_count = 1;

      if (_timeslice > 0) {
        double slice_count_d = static_cast<double>(dt / _timeslice);

        slice_count = static_cast<int>(slice_count_d);
        if (slice_count_d - slice_count > 0.9) slice_count++;
        if (slice_count < 1) slice_count++;

        timestep = _timeslice;
      }

      double vel_min   = _limits(0, 1);
      double vel_max   = _limits(1, 1);
      double accel_min = _limits(0, 2);
      double accel_max = _limits(1, 2);

      segment_t seg = last;

      double start_time = seg.time;

      for (int i = 1; i <= slice_count; i++) {
        double t = start_time + (i * timestep);
        if (t > time) t = time;
        dt = t - seg.time;

        kinematics_t &k = seg.kinematics;

        double error = k[0] - _goal;
        double accel = (error < 0 ? accel_max : accel_min);

        // TODO: Find point at which we reach v_max and if it's less than dt, split
        // this slice into half.
        double v_projected = k[1] + accel * dt;
        v_projected = v_projected > vel_max ? vel_max : v_projected < vel_min ? vel_min : v_projected;

        double decel_time  = v_projected / -accel_min;
        double decel_dist  = v_projected * decel_time + 0.5 * accel_min * decel_time * decel_time;
        double decel_error = k[0] + decel_dist - _goal;

        // TODO: make this better
        // If we decelerate now, do we cross the zero of the error function?
        bool decel_cross_error_zeros = (error > 0 && decel_error < 0) || (error < 0 && decel_error > 0);
        // Are we not currently decelerating?
        bool decel_not_in_progress = (error < 0 && k[1] > 0) || (error > 0 && k[1] < 0);

        if (decel_cross_error_zeros && decel_not_in_progress)
          accel = (accel < 0 ? accel_max : accel_min);
        else if (fabs(k[1] - vel_max) < 0.0001)
          accel = 0;
        else if (fabs(error) < 0.0001)
          accel = 0;

        double vel = k[1] + (accel * dt);
        k[0]       = k[0] + (k[1] * dt) + (0.5 * accel * dt * dt);
        k[1]       = vel > vel_max ? vel_max : vel < vel_min ? vel_min : vel;
        k[2]       = accel;
        seg.time   = t;
      }
      return seg;
    }
  };

}  // namespace profile
}  // namespace grpl