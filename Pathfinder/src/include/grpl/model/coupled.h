#pragma once

#include "grpl/model/transmission.h"
#include "grpl/path/curve.h"
#include "grpl/profile/profile.h"
#include "grpl/util/constants.h"

#include <iostream>
#include <utility>

namespace grpl {
namespace model {

  // TODO: The structure of this class needs some work. It's doing too much all at the same time.
  template <typename profile_t>
  class coupled {
   public:
    // Robot Configuration:
    //  x, y, heading
    using configuration_t = Eigen::Vector3d;
    // Motion Information:
    //  distance, velocity, acceleration
    using kinematics_t = Eigen::Vector3d;

    using vector_t = Eigen::Vector2d;
    using trans_t  = transmission::dc_transmission;

    struct state {
      double          time, curvature, curvature_prime;
      configuration_t configuration;
      kinematics_t    kinematics;
      bool            finished;
    };

    struct wheel_state {
      double   time;
      vector_t position;
      // We don't use kinematics_t since keeping track of distance invokes a memory on the system
      // when split, since the distance of the sides can't be reasonably deduced from the instantaneous
      // centre path alone. It's better to change the data structure than leave kinematics[0] as undefined
      // behaviour.
      // TODO: Actually, is bundling into kinematics better since it allows us to genericize passing into a
      // control loop?
      double velocity, acceleration;
      double voltage, current;
      bool   finished;
    };

    state initial_state() {
      state st;
      st.time             = 0;
      st.curvature        = 0;
      st.configuration[0] = 0;
      st.configuration[1] = 0;
      st.configuration[2] = 0;
      st.kinematics[0]    = 0;
      st.kinematics[1]    = 0;
      st.kinematics[2]    = 0;
      st.finished         = false;
      return st;
    }

    coupled(trans_t &left, trans_t &right, double wheel_r, double track_r, double mass, double max_voltage)
        : _transmission_left(left),
          _transmission_right(right),
          _wheel_r(wheel_r),
          _track_r(track_r),
          _mass(mass),
          _max_voltage(max_voltage) {}

    template <typename iterator_curve_t>
    inline bool find_curve(double targ_len, const iterator_curve_t curve_begin,
                           const iterator_curve_t curve_end, iterator_curve_t &curve_out,
                           double &curve_len_out, double &total_len_out) {
      curve_len_out = targ_len;
      total_len_out = 0;
      bool found    = false;

      for (iterator_curve_t it = curve_begin; it != curve_end; it++) {
        double len = it->length();
        // If we haven't found a curve, and the current length of the curve will put us ahead
        // of our distance target.
        if (!found && (len + total_len_out) >= targ_len) {
          found         = true;
          curve_len_out = targ_len - total_len_out;
          curve_out     = it;
        }
        total_len_out += len;
      }
      return found;
    }

    double linear_vel_limit(configuration_t &config, double curv) const {
      // Infinite curvature, point turn (purely angular), therefore no linear velocity.
      // We return eagerly.
      if (std::abs(curv) > 1e10) return 0;

      // Wheel tangent speed, maximum possible speeds
      // Ordered right, left.
      Eigen::Vector2d maximum_vels{_transmission_right.get_free_speed(_max_voltage) * _wheel_r,
                                   _transmission_left.get_free_speed(_max_voltage) * _wheel_r};

      // Wheel tangent speed, actual values.
      // Ordered right, left
      Eigen::Vector2d wheel_vels;

      if (curv < 1e-10) {
        // No curvature, straight forward (purely linear). Need to take minimum as for
        // differing transmissions there may be a non-equal speed limit. We can skip all
        // the logic below to be more efficient. We return eagerly.
        return maximum_vels.minCoeff();
      } else {
        // Take the maximum speed of each wheel, then limit the opposing one, ensuring
        // we are within the limits above.

        // v_c = 0.5*(v_r + v_l)                [1] Linear Velocity
        // w   = 0.5*(v_r - v_l) / r            [2] Angular Velocity
        // k = w / v_c                          [3] Curvature
        // note w / v_c has 0.5 as common factor, cancels out for the following
        // k = ((v_r - v_l)/r) / (v_r + v_l)
        // rk = (v_r - v_l) / (v_r + v_l)
        // v_r(1 - rk) = v_l(1 + rk)            [4]
        // let ratio = (1 - rk) / (1 + rk)
        double ratio = (1 - _track_r * curv) / (1 + _track_r * curv);
        // v_l = v_r * ratio                    [5]
        // v_r = v_l / ratio                    [6]
        // TODO: This can all be compressed using some clever linalg
        if (curv > 0) {
          // Banking counter-clockwise (steering to the left, right side dominant)
          // therefore, v_r = maximum freespeed
          wheel_vels[0] = maximum_vels[0];
          // v_l = v_r * ratio                  via [5]
          wheel_vels[1] = wheel_vels[0] * ratio;

          if (wheel_vels[1] > maximum_vels[1]) {
            // Left side is over it's maximum speed - cap it to its maximum speed, then
            // limit the right side.
            wheel_vels[1] = maximum_vels[1];
            // v_r = v_l / ratio                via [6]
            wheel_vels[0] = wheel_vels[1] / ratio;
          }
        } else {
          // Banking clockwise (steering to the right, left side dominant)
          // therefore, v_l = maximum freespeed
          wheel_vels[1] = maximum_vels[1];
          // v_r = v_l / ratio                  via [6]
          wheel_vels[0] = wheel_vels[1] / ratio;

          if (wheel_vels[1] > maximum_vels[1]) {
            // Right side is over it's maximum speed - cap it to its maximum speed, then
            // limit the left side.
            wheel_vels[0] = maximum_vels[0];
            // v_l = v_r * ratio                via [5]
            wheel_vels[1] = wheel_vels[0] * ratio;
          }
        }

        // Maximum Linear Velocity              via [1]
        return wheel_vels.sum() / 2.0;
      }
    }

    std::pair<double, double> acceleration_limits(configuration_t &config, double curv,
                                                  double velocity) const {
      double linear = velocity;
      // k = w / v, w = v * k
      double angular = velocity * curv;
      // v_diff = w * r
      double differential = angular * _track_r;

      // v_r = v + v_diff, w_r = v_r / r_wheel
      // v_l = v - v_diff, w_l = v_l / r_wheel
      // Ordered right, left.
      Eigen::Vector2d wheels{(linear + differential) / _wheel_r, (linear - differential) / _wheel_r};

      // Calculate fwd torque limits for each side
      Eigen::Vector2d fwd_torque_limits{
          _transmission_right.get_torque(_transmission_right.get_current(_max_voltage, wheels[0])),
          _transmission_left.get_torque(_transmission_left.get_current(_max_voltage, wheels[1]))};

      Eigen::Vector2d fwd_accel_limits = fwd_torque_limits / (_mass * _wheel_r);

      double max = fwd_accel_limits.sum() / 2.0;

      // Calculate rvs torque limits for each side
      Eigen::Vector2d rvs_torque_limits{
          _transmission_right.get_torque(-_transmission_right.get_current(_max_voltage, wheels[0])),
          _transmission_left.get_torque(-_transmission_left.get_current(_max_voltage, wheels[1]))};
      
      Eigen::Vector2d rvs_accel_limits = rvs_torque_limits / (_mass * _wheel_r);

      double min = rvs_accel_limits.sum() / 2.0;

      return std::pair<double, double>{min, max};
    }

    void solve_electrical(wheel_state &wheel, trans_t &transmission) {
      double speed        = wheel.velocity / _wheel_r;
      double free_voltage = transmission.get_free_voltage(speed);

      double torque          = _mass * wheel.acceleration * _wheel_r;
      double current         = transmission.get_torque_current(torque);
      double current_voltage = transmission.get_current_voltage(current);

      double total_voltage = free_voltage + current_voltage;

      wheel.voltage = total_voltage;
      wheel.current = current;
    }

    double config_distance(configuration_t &c0, configuration_t &c1) const { return (c0 - c1).norm(); }

    template <typename iterator_curve_t>
    state generate(const iterator_curve_t curve_begin, const iterator_curve_t curve_end, profile_t &profile,
                   state &last, double time) {
      iterator_curve_t curve;
      state            output;
      double           total_length, curve_distance;
      double           distance = last.kinematics[0];

      bool curve_found = find_curve(distance, curve_begin, curve_end, curve, curve_distance, total_length);

      // TODO: The epsilon of the profile causes this to never advance, meaning the path
      // is never marked as 'finished' on some timesteps.
      if (!curve_found) {
        output          = last;
        output.finished = true;
        return output;
      }

      profile.set_goal(total_length);

      vector_t centre     = curve->position(curve_distance);
      vector_t centre_rot = curve->rotation(curve_distance);
      double   curvature  = curve->curvature(curve_distance);
      double   dcurvature = curve->curvature_prime(curve_distance);

      double          heading = atan2(centre_rot.y(), centre_rot.x());
      configuration_t config{centre.x(), centre.y(), heading};
      double          ds = config_distance(config, last.configuration);

      output.time          = time;
      output.configuration = config;

      // TODO: Allow multiple constraints (like current limits)
      // TODO: Enforce minimum acceleration constraints in profiles.
      // TODO: Does limiting jerk prevent oscillation
      double limit_vel = linear_vel_limit(config, curvature);
      std::pair<double, double> limit_acc = acceleration_limits(config, curvature, last.kinematics[1]);

      profile.apply_limit(1, -limit_vel, limit_vel);
      profile.apply_limit(2, limit_acc.first, limit_acc.second);

      typename profile_t::segment_t segment;
      segment.time       = last.time;
      segment.kinematics = last.kinematics;

      segment = profile.calculate(segment, time);

      output.kinematics      = segment.kinematics;
      output.curvature       = curvature;
      output.curvature_prime = dcurvature;

      output.finished = false;
      return output;
    }

    std::pair<wheel_state, wheel_state> split(state centre) {
      wheel_state left, right;

      left.time = right.time = centre.time;
      left.finished = right.finished = centre.finished;

      // Split positions
      vector_t position{centre.configuration.x(), centre.configuration.y()};
      double   heading  = centre.configuration[2];
      vector_t p_offset = vector_t{0, _track_r};

      Eigen::Matrix<double, 2, 2> rotation;
      rotation << cos(heading), -sin(heading), sin(heading), cos(heading);

      // Rotate the wheel offsets by the heading of the robot, adding it to the
      // centre position, this 'splits' the centre path into two paths constrained
      // by the configuration (heading + position) and track radius.
      left.position  = position + rotation * p_offset;
      right.position = position - rotation * p_offset;

      // Split velocities
      double v_linear       = centre.kinematics[1];
      double v_angular      = v_linear * centre.curvature;
      double v_differential = v_angular * _track_r;
      left.velocity         = v_linear - v_differential;
      right.velocity        = v_linear + v_differential;

      // Split accelerations
      double a_linear = centre.kinematics[2];
      // This is a bit of a tricky one, so don't blink
      // a_angular = dw / dt (where w = v_angular)
      // a_angular = d/dt (v * k) (from v_angular above, w = vk)
      // Then, by product rule:
      //    a_angular = dv/dt * k + v * dk/dt     (note dv/dt is acceleration)
      //    a_angular = a * k + v * dk/dt         [1]
      // We don't have dk/dt, but we do have dk/ds. By chain rule:
      //    dk/dt = dk/ds * ds/dt                 (note ds/dt is velocity)
      //    dk/dt = dk/ds * v                     [2]
      // Therefore, by composing [1] and [2],
      //    a_angular = a * k + v^2 * dk/ds
      // Isn't that just a gorgeous piece of math?
      double a_angular      = a_linear * centre.curvature + v_linear * v_linear * centre.curvature_prime;
      double a_differential = a_angular * _track_r;
      left.acceleration     = a_linear - a_differential;
      right.acceleration    = a_linear + a_differential;

      solve_electrical(left, _transmission_left);
      solve_electrical(right, _transmission_right);

      return std::pair<wheel_state, wheel_state>{left, right};
    }

   private:
    trans_t &_transmission_left, &_transmission_right;
    double   _max_voltage, _wheel_r, _track_r, _mass;
  };
}  // namespace model
}  // namespace grpl