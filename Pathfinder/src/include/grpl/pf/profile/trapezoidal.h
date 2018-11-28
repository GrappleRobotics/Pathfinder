#pragma once

#include "profile.h"

#include <iostream>

namespace grpl {
namespace pf {
  namespace profile {
    /**
     * Implementation of a trapezoidal (limited acceleration) motion profile.
     *
     * A trapezoidal motion profile is a motion profile limited by acceleration (therefore, infinite jerk).
     * The profile can be described by three distinct sections: ramp-up, hold and ramp-down.
     *
     * During ramp-up, the system is accelerating towards its max velocity.
     *
     * During hold, the system is not accelerating, and holds its max velocity. Depending on the setpoint,
     * the system may not have time to reach hold before it must ramp-down, resulting in a trianglular
     * velocity profile.
     *
     * During ramp-down, the system is decelerating towards 0.
     *
     * See @ref grpl::pf::profile::profile
     */
    class trapezoidal : public profile {
     public:
      const size_t limited_term() const override { return ACCELERATION; }

      state calculate(state &last, double time) override {
        double dt          = time - last.time;
        double timestep    = dt;
        int    slice_count = 1;

        if (this->_timeslice > 0) {
          double slice_count_d = static_cast<double>(dt / this->_timeslice);

          slice_count = static_cast<int>(slice_count_d);
          if (slice_count_d - slice_count > 0.9) slice_count++;
          if (slice_count < 1) slice_count++;

          timestep = this->_timeslice;
        }

        double vel_min   = this->_limits(0, 1);
        double vel_max   = this->_limits(1, 1);
        double accel_min = this->_limits(0, 2);
        double accel_max = this->_limits(1, 2);

        state cur = last;

        double start_time = cur.time;

        for (int i = 1; i <= slice_count; i++) {
          double t = start_time + (i * timestep);
          if (t > time) t = time;
          dt = t - cur.time;

          auto &kin = cur.kinematics;

          double error = kin[POSITION] - this->_goal;
          double accel = (error < 0 ? accel_max : accel_min);

          // TODO: Find point at which we reach v_max and if it's less than dt, split
          // this slice into half.
          double v_projected = kin[VELOCITY] + accel * dt;
          v_projected = v_projected > vel_max ? vel_max : v_projected < vel_min ? vel_min : v_projected;

          double decel_time  = v_projected / -accel_min;
          double decel_dist  = v_projected * decel_time + 0.5 * accel_min * decel_time * decel_time;
          double decel_error = kin[POSITION] + decel_dist - this->_goal;

          // TODO: make this better
          // If we decelerate now, do we cross the zero of the error function?
          bool decel_cross_error_zeros = (error > 0 && decel_error < 0) || (error < 0 && decel_error > 0);
          // Are we not currently decelerating?
          bool decel_not_in_progress = (error < 0 && kin[VELOCITY] > 0) || (error > 0 && kin[VELOCITY] < 0);

          if (decel_cross_error_zeros && decel_not_in_progress)
            accel = (accel < 0 ? accel_max : accel_min);
          else if (fabs(kin[1] - vel_max) < constants::default_acceptable_error)
            accel = 0;
          else if (fabs(error) < constants::default_acceptable_error)
            accel = 0;

          double vel        = kin[VELOCITY] + (accel * dt);
          kin[POSITION]     = kin[POSITION] + (kin[VELOCITY] * dt) + (0.5 * accel * dt * dt);
          kin[VELOCITY]     = vel > vel_max ? vel_max : vel < vel_min ? vel_min : vel;
          kin[ACCELERATION] = accel;
          cur.time          = t;
        }
        return cur;
      }
    };
  }  // namespace profile
}  // namespace pf
}  // namespace grpl