#pragma once

#include "grpl/util/constants.h"

#include <Eigen/Dense>
#include <iostream>

namespace grpl {
namespace model {

  class motor_model {
   public:
    motor_model(double v_nom, double free_speed, double free_current, double stall_current,
                double stall_torque)
        : _v_nom(v_nom),
          _free_speed(free_speed),
          _free_current(free_current),
          _stall_current(stall_current),
          _stall_torque(stall_torque) {}

    double internal_resistance() const { return _v_nom / _stall_current; }

    double get_current(double voltage, double angular_velocity) const {
      double r_int = internal_resistance();
      double kv    = (_v_nom - _free_current * r_int) / _free_speed;
      // V_omega = kv * omega
      double vel_voltage = kv * angular_velocity;
      // V = IR + kv*omega = IR + V_vel
      // I = (V - V_vel) / R
      return (voltage - vel_voltage) / r_int;
    }

    double get_torque(double current) const {
      // I = kt * t, t = I / kt
      double kt = _stall_current / _stall_torque;
      return current / kt;
    }

    double get_current_2(double torque) const {
      // I = kt * t
      double kt = _stall_current / _stall_torque;
      return kt * torque;
    }

    double get_voltage(double current, double angular_velocity) const {
      double r_int = internal_resistance();
      double kv    = (_v_nom - _free_current * r_int) / _free_speed;

      // V = IR + kv*omega
      return current * r_int + kv * angular_velocity;
    }

   private:
    double _v_nom = 12;     // V
    double _free_speed;     // rad/s
    double _free_current;   // A
    double _stall_current;  // A
    double _stall_torque;   // Nm
  };

  class cdt_model {
   public:
    struct limits {
      Eigen::Matrix<double, 2, 1> wheel_vels;
      Eigen::Matrix<double, 2, 1> current_limits;
      Eigen::Matrix<double, 2, 1> torque_limits;
      Eigen::Matrix<double, 2, 1> accel_limits;
      Eigen::Matrix<double, 2, 1> velocity_limits;
    };

    cdt_model(motor_model &motor, double max_voltage, double wheel_diam, double weight, double trackwidth) 
      : _motor(motor), _max_voltage(max_voltage), _wheel_r(wheel_diam / 2.0), _weight(weight), _trackwidth(trackwidth) {} 

    void set_max_current(double cur_limit) {
      _max_current = cur_limit;
    }

    struct limits calculate_limits_2(double angular_velocity, double dt) {
      struct limits l;
      double vel_diff = angular_velocity * _trackwidth/2.0;

      Eigen::Matrix<double, 2, 1> diffs;
      diffs << -vel_diff, vel_diff;

      l.wheel_vels = (_vel + diffs) / (_wheel_r);

      Eigen::Matrix<double, 2, 1> voltages;
      voltages(0, 0) = _motor.get_voltage(_max_current, l.wheel_vels(0, 0));
      voltages(1, 0) = _motor.get_voltage(_max_current, l.wheel_vels(1, 0));

      if (voltages(0, 0) > _max_voltage || voltages(1, 0) > _max_voltage) {
        voltages(0, 0) = _max_voltage;
        voltages(1, 0) = _max_voltage;
      }

      l.current_limits(0, 0) = _motor.get_current(voltages(0, 0), l.wheel_vels(0, 0));
      l.current_limits(1, 0) = _motor.get_current(voltages(1, 0), l.wheel_vels(1, 0));

      l.torque_limits(0, 0) = _motor.get_torque(l.current_limits(0, 0));
      l.torque_limits(1, 0) = _motor.get_torque(l.current_limits(1, 0));

      l.accel_limits = l.torque_limits / _wheel_r / _weight;
      // TODO: work backwards to fit in accel limits.
      l.velocity_limits = l.accel_limits * dt;

      return l;
    }

    struct limits calculate_limits(double angular_velocity, double dt) {
      struct limits l;
      double vel_diff = angular_velocity * _trackwidth/2.0;

      Eigen::Matrix<double, 2, 1> diffs;
      diffs << -vel_diff, vel_diff;

      l.wheel_vels = (_vel + diffs) / (_wheel_r);

      l.current_limits(0, 0) = _motor.get_current(_max_voltage, l.wheel_vels(0, 0));
      l.current_limits(1, 0) = _motor.get_current(_max_voltage, l.wheel_vels(1, 0));

      l.torque_limits(0, 0) = _motor.get_torque(l.current_limits(0, 0));
      l.torque_limits(1, 0) = _motor.get_torque(l.current_limits(1, 0));

      l.accel_limits = l.torque_limits / _wheel_r / _weight;
      // TODO: work backwards to fit in accel limits.
      l.velocity_limits = l.accel_limits * dt;

      return l;
    }

    void set_vel(Eigen::Matrix<double, 2, 1> vel) {
      _vel = vel;
    }

   private:
    motor_model _motor;
    double      _wheel_r;
    double      _max_voltage;
    double      _weight;
    double      _trackwidth;
    // Other
    double      _max_current;
    // State
    Eigen::Matrix<double, 2, 1> _vel;
  };
}  // namespace model
}  // namespace grpl