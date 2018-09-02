#pragma once

#include "grpl/model/transmission.h"
#include "grpl/path/curve.h"
#include "grpl/profile/profile.h"
#include "grpl/util/constants.h"

#include <iostream>
#include <utility>

namespace grpl {
namespace model {

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
      double          time, curvature;
      configuration_t configuration;
      kinematics_t    kinematics;
      bool            finished;
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

    coupled(trans_t &left, trans_t &right, double wheel_r, double track_r, double weight, double max_voltage)
        : _transmission_left(left),
          _transmission_right(right),
          _wheel_r(wheel_r),
          _track_r(track_r),
          _weight(weight),
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

      // Calculate torque limits for each side
      Eigen::Vector2d torque_limits{
          _transmission_right.get_torque(_transmission_right.get_current(_max_voltage, wheels[0])),
          _transmission_left.get_torque(_transmission_left.get_current(_max_voltage, wheels[1]))};

      // T = F * r, F = ma
      // T = ma * r
      // a = T / (m*r)
      Eigen::Vector2d accel_limits = torque_limits / (_weight * _wheel_r);

      double max = accel_limits.sum() / 2.0;
      // TODO: Reverse Accel
      return std::pair<double, double>{-max, max};
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

      if (!curve_found) {
        output          = last;
        output.finished = true;
        return output;
      }

      profile.set_goal(total_length);

      vector_t centre     = curve->position(curve_distance);
      vector_t centre_rot = curve->rotation(curve_distance);
      double   curvature  = curve->curvature(curve_distance);

      double          heading = atan2(centre_rot.y(), centre_rot.x());
      configuration_t config{centre.x(), centre.y(), heading};
      double          ds = config_distance(config, last.configuration);

      output.time          = time;
      output.configuration = config;

      // TODO: Allow multiple constraints.
      // TODO: Enforce minimum acceleration constraints in profiles.
      double max_vel = linear_vel_limit(config, curvature);
      double max_acc = acceleration_limits(config, curvature, last.kinematics[1]).second;

      Eigen::Matrix<double, 1, 3> limits{0, max_vel, max_acc};
      profile.set_limits(limits);

      typename profile_t::segment_t segment;
      segment.time       = last.time;
      segment.kinematics = last.kinematics;

      segment = profile.calculate(segment, time);

      output.kinematics = segment.kinematics;
      output.curvature  = curvature;

      output.finished = false;
      return output;
    }

   private:
    trans_t &_transmission_left, &_transmission_right;
    double   _max_voltage, _wheel_r, _track_r, _weight;
  };
}  // namespace model
}  // namespace grpl