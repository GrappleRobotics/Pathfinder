#pragma once

#include "chassis.h"
#include "grpl/pf/model/transmission.h"
#include "grpl/pf/util/constants.h"
#include "state.h"

namespace grpl {
namespace pf {
  namespace coupled {
    class chassis {
     public:
      using transmission_t = transmission::dc_transmission;

      chassis(transmission_t &transmission_left, transmission_t &transmission_right, double wheel_radius,
              double track_radius, double mass)
          : _trans_left(transmission_left),
            _trans_right(transmission_right),
            _wheel_radius(wheel_radius),
            _track_radius(track_radius),
            _mass(mass) {}

      double mass() const { return _mass; }
      double track_radius() const { return _track_radius; }
      double wheel_radius() const { return _wheel_radius; }

      transmission_t &transmission_left() const { return _trans_left; }
      transmission_t &transmission_right() const { return _trans_right; }

      double linear_vel_limit(configuration &config, double curvature) const {
        // Infinite curvature, point turn (purely angular), therefore no linear velocity.
        if (std::abs(curvature) > constants::almost_inf) return 0;

        // Wheel linear speed, maximum possible speeds
        // Ordered left, right.
        Eigen::Vector2d maximum_vels{_trans_left.get_free_speed(_trans_left.nominal_voltage()) * _wheel_radius,
                                     _trans_right.get_free_speed(_trans_right.nominal_voltage()) * _wheel_radius};

        if (std::abs(curvature) < constants::epsilon) {
          // No curvature, straight forward (purely linear). Need to take minimum as for
          // differing transmissions there may be a non-equal speed limit. We can skip all
          // the logic below to be more efficient. We return eagerly.
          return maximum_vels.minCoeff();
        } else {
          // Take the maximum speed of each wheel, then limit the opposing one, ensuring
          // we are within the limits above.

          // Wheel linear speed, actual values.
          // Ordered left, right.
          Eigen::Vector2d wheel_vels;

          // v_c = 0.5*(v_r + v_l)                [1] Linear Velocity
          // w   = 0.5*(v_r - v_l) / r            [2] Angular Velocity
          // k = w / v_c                          [3] Curvature
          // note w / v_c has 0.5 as common factor, cancels out for the following
          // k = ((v_r - v_l)/r) / (v_r + v_l)
          // rk = (v_r - v_l) / (v_r + v_l)
          // v_r(1 - rk) = v_l(1 + rk)            [4]
          // let ratio = (1 - rk) / (1 + rk)
          double ratio = (1 - _track_radius * curvature) / (1 + _track_radius * curvature);
          // v_l = v_r * ratio                    [5]
          // v_r = v_l / ratio                    [6]

          // TODO: This can all be compressed using some clever linalg
          if (curvature > 0) {
            // Banking counter-clockwise (steering to the left, right side dominant)
            // therefore, v_r = maximum freespeed
            wheel_vels[1] = maximum_vels[1];
            // v_l = v_r * ratio                  via [5]
            wheel_vels[0] = wheel_vels[1] * ratio;

            if (wheel_vels[0] > maximum_vels[0]) {
              // Left side is over it's maximum speed - cap it to its maximum speed, then
              // limit the right side.
              wheel_vels[0] = maximum_vels[0];
              // v_r = v_l / ratio                via [6]
              wheel_vels[1] = wheel_vels[0] / ratio;
            }
          } else {
            // Banking clockwise (steering to the right, left side dominant)
            // therefore, v_l = maximum freespeed
            wheel_vels[0] = maximum_vels[0];
            // v_r = v_l / ratio                  via [6]
            wheel_vels[1] = wheel_vels[0] / ratio;

            if (wheel_vels[0] > maximum_vels[0]) {
              // Right side is over it's maximum speed - cap it to its maximum speed, then
              // limit the left side.
              wheel_vels[1] = maximum_vels[1];
              // v_l = v_r * ratio                via [5]
              wheel_vels[0] = wheel_vels[1] * ratio;
            }
          }

          // Maximum Linear Velocity              via [1]
          return wheel_vels.sum() / 2.0;
        }
      }

      std::pair<double, double> acceleration_limits(configuration &config, double curvature,
                                                    double velocity) const {
        double linear = velocity;
        // k = w / v, w = v * k
        double angular = velocity * curvature;
        // v_diff = w * r
        double differential = angular * _track_radius;

        // v_r = v + v_diff, w_r = v_r / r_wheel
        // v_l = v - v_diff, w_l = v_l / r_wheel
        // Ordered right, left.
        Eigen::Vector2d wheels{(linear + differential) / _wheel_radius, (linear - differential) / _wheel_radius};

        // Calculate fwd torque limits for each side
        Eigen::Vector2d fwd_torque_limits{
            _trans_right.get_torque(_trans_right.get_current(_trans_right.nominal_voltage(), wheels[0])),
            _trans_left.get_torque(_trans_left.get_current(_trans_left.nominal_voltage(), wheels[1]))};

        Eigen::Vector2d fwd_accel_limits = fwd_torque_limits / (_mass * _wheel_radius);

        double max = fwd_accel_limits.sum() / 2.0;

        // Calculate rvs torque limits for each side
        Eigen::Vector2d rvs_torque_limits{
            _trans_right.get_torque(-_trans_right.get_current(_trans_right.nominal_voltage(), wheels[0])),
            _trans_left.get_torque(-_trans_left.get_current(_trans_left.nominal_voltage(), wheels[1]))};

        Eigen::Vector2d rvs_accel_limits = rvs_torque_limits / (_mass * _wheel_radius);

        double min = rvs_accel_limits.sum() / 2.0;

        return std::pair<double, double>{min, max};
      }

      void solve_electrical(wheel_state &left, wheel_state &right) {
        do_solve_electrical(left, _trans_left);
        do_solve_electrical(right, _trans_right);
      }

     private:
      // TODO: Make this part of transmission_t
      void do_solve_electrical(wheel_state &wheel, transmission_t &transmission) {
        double speed        = wheel.kinematics[1] / _wheel_radius;
        double free_voltage = transmission.get_free_voltage(speed);

        double torque          = _mass * wheel.kinematics[2] * _wheel_radius;
        double current         = transmission.get_torque_current(torque);
        double current_voltage = transmission.get_current_voltage(current);

        double total_voltage = free_voltage + current_voltage;

        wheel.voltage = total_voltage;
        wheel.current = current;
      }

      double _mass, _track_radius, _wheel_radius;
      // TODO: Not reference
      transmission_t &_trans_left, &_trans_right;
    };
  }  // namespace coupled
}  // namespace pf
}  // namespace grpl