#pragma once

#include "chassis.h"
#include "grpl/pf/constants.h"
#include "grpl/pf/transmission/dc.h"
#include "state.h"

namespace grpl {
namespace pf {
  namespace coupled {

    /**
     * @brief
     * Mathematical model representation of a coupled (tank / differential) drivetrain.
     *
     * Chassis contains members for the transmissions (motors), as well as other configurations
     * regarding the chassis (track radius, wheel radius, mass, etc).
     *
     * The chassis mirrors the physical "layout" of the drivetrain.
     *
     * @ref grpl::pf::coupled::drivetrain
     */
    class chassis {
     public:
      using transmission_t = transmission::dc_transmission;

      /**
       * @brief
       * Construct a coupled chassis
       *
       * @param transmission_left   The left side transmission, held internally as a reference.
       * @param transmission_right  The right side transmission, held internally as a reference.
       * @param wheel_radius        The wheel radius, in metres. Should be emperically measured
       *                            for best performance. Note that both the left and right
       *                            transmission wheels must be the same radius.
       * @param track_radius        The track radius, a.k.a half the chassis width, in metres.
       *                            Measured as half the distance between the centres of the left
       *                            and right transmissions at their points of contact with the ground.
       * @param mass                The mass of the chassis, in kilograms.
       */
      chassis(transmission_t &transmission_left, transmission_t &transmission_right, double wheel_radius,
              double track_radius, double mass)
          : _trans_left(transmission_left),
            _trans_right(transmission_right),
            _wheel_radius(wheel_radius),
            _track_radius(track_radius),
            _mass(mass) {}

      /**
       * @return The mass of the chassis, in kilograms
       */
      double mass() const { return _mass; }

      /**
       * @return  The track radius (half the chassis width) in metres. Measured
       *          between the left and right transmissions.
       */
      double track_radius() const { return _track_radius; }

      /**
       * @return  The wheel radius, in metres.
       */
      double wheel_radius() const { return _wheel_radius; }

      /**
       * @return  Reference to the left-side transmission of the chassis.
       */
      transmission_t &transmission_left() const { return _trans_left; }

      /**
       * @return  Reference to the right-side transmission of the chassis.
       */
      transmission_t &transmission_right() const { return _trans_right; }

      /**
       * @brief
       * Calculate the absolute linear (translational) velocity limit of the chassis in metres
       * per second (ms^-1).
       *
       * This calculation relates purely to the free-speed of the motors, meaning for a fully
       * constrained calculation, @ref acceleration_limits(configuration_state&, double, double)
       * should be called and used to constrain the velocity if necessary.
       *
       * @param config    The configuration of the chassis
       * @param curvature The instantaneous curvature, in metres^-1, expected of the chassis,
       *                  such as the curvature of a path being followed.
       *
       * @return  The absolute linear (translational) velocity limit in metres per second (ms^-2).
       */
      double linear_vel_limit(const configuration_state &config, double curvature) const {
        // Infinite curvature, point turn (purely angular), therefore no linear velocity.
        if (std::abs(curvature) > constants::almost_inf) return 0;

        // Wheel linear speed, maximum possible speeds
        // Ordered left, right.
        Eigen::Vector2d maximum_vels{
            _trans_left.get_free_speed(_trans_left.nominal_voltage()) * _wheel_radius,
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

      /**
       * @brief
       * Calculate the minimum and maximum linear (translational) acceleration limits of the chassis,
       * in metres per second per second (ms^-2).
       *
       * This calculation uses torque limits of the transmissions, meaning speed limits
       * are not directly taken into account. For a fully constrained representation,
       * @ref linear_vel_limit(configuration_state&, double) must also be called and used to constrain
       * if necessary.
       *
       * @param config    The configuration of the chassis
       * @param curvature The instantaneous curvature, in metres^-1, expected of the chassis,
       *                  such as the curvature of a path being followed.
       * @param velocity  The current linear velocity of the chassis, in metres per second (ms^-1).
       *
       * @return  A pair, ordered [min, max], of the linear acceleration limits, in metres per second
       *          per second (ms^-2).
       */
      std::pair<double, double> acceleration_limits(const configuration_state &config, double curvature,
                                                    double velocity) const {
        double linear = velocity;
        // k = w / v, w = v * k
        double angular = velocity * curvature;
        // v_diff = w * r
        double differential = angular * _track_radius;

        // v_r = v + v_diff, w_r = v_r / r_wheel
        // v_l = v - v_diff, w_l = v_l / r_wheel
        // Ordered right, left.
        Eigen::Vector2d wheels{(linear + differential) / _wheel_radius,
                               (linear - differential) / _wheel_radius};

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

      /**
       * @brief
       * Split a centre state of this chassis into the left and right transmission state components.
       *
       * @return  A pair, ordered [left, right], of @ref wheel_state.
       */
      std::pair<wheel_state, wheel_state> split(const state centre) const {
        wheel_state left, right;

        left.time = right.time = centre.time;
        left.finished = right.finished = centre.finished;

        // Split positions
        wheel_state::vector_t position{centre.config.x(), centre.config.y()};
        double                heading = centre.config[2];
        wheel_state::vector_t p_offset{0, _track_radius};

        Eigen::Matrix<double, 2, 2> rotation;
        rotation << cos(heading), -sin(heading), sin(heading), cos(heading);

        // Rotate the wheel offsets by the heading of the robot, adding it to the
        // centre position, this 'splits' the centre path into two paths constrained
        // by the configuration (heading + position) and track radius.
        left.position  = position + rotation * p_offset;
        right.position = position - rotation * p_offset;

        // Split velocities
        double v_linear            = centre.kinematics[VELOCITY];
        double v_angular           = v_linear * centre.curvature;
        double v_differential      = v_angular * _track_radius;
        left.kinematics[VELOCITY]  = v_linear - v_differential;
        right.kinematics[VELOCITY] = v_linear + v_differential;

        // Split accelerations
        double a_linear = centre.kinematics[ACCELERATION];
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
        double a_angular              = a_linear * centre.curvature + v_linear * v_linear * centre.dcurvature;
        double a_differential         = a_angular * _track_radius;
        left.kinematics[ACCELERATION] = a_linear - a_differential;
        right.kinematics[ACCELERATION] = a_linear + a_differential;

        solve_electrical(left, right);

        return std::pair<wheel_state, wheel_state>{left, right};
      }

     private:
      void solve_electrical(wheel_state &left, wheel_state &right) const {
        do_solve_electrical(left, _trans_left);
        do_solve_electrical(right, _trans_right);
      }

      // TODO: Make this part of transmission_t
      void do_solve_electrical(wheel_state &wheel, transmission_t &transmission) const {
        double speed        = wheel.kinematics[VELOCITY] / _wheel_radius;
        double free_voltage = transmission.get_free_voltage(speed);

        double torque          = _mass * wheel.kinematics[ACCELERATION] * _wheel_radius;
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