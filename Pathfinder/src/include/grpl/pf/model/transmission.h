#pragma once

namespace grpl {
namespace pf {
  namespace transmission {
    // Purely Functional DC Motor.
    class dc_transmission {
     public:
      // TODO: Need a better naming scheme
      virtual double get_current(double voltage, double speed) const = 0;
      virtual double get_torque(double current) const                = 0;
      virtual double get_free_speed(double voltage) const            = 0;

      virtual double get_free_voltage(double speed) const      = 0;
      virtual double get_current_voltage(double current) const = 0;
      virtual double get_torque_current(double torque) const   = 0;

      virtual double nominal_voltage() const = 0;
    };

    // Basic DC Motor Model, dervied from the ideal resistive motor model with Back EMF
    // (+ ---[ R ]---( V_w )--- -), where R = V / I_stall (as V_w = 0 at stall), and
    // V_w = kv*w.
    class dc_motor : public dc_transmission {
     public:
      dc_motor(double v_nom, double free_speed, double free_current, double stall_current,
               double stall_torque)
          : _v_nom(v_nom),
            _free_speed(free_speed),
            _free_current(free_current),
            _stall_current(stall_current),
            _stall_torque(stall_torque) {}

      inline double internal_resistance() const { return _v_nom / _stall_current; }
      inline double kv() const { return (_v_nom - _free_current * _v_nom / _stall_current) / _free_speed; }
      inline double kt() const { return _stall_current / _stall_torque; }

      double nominal_voltage() const override { return _v_nom; }

      double get_current(double voltage, double speed) const override {
        // V_w = kv * w
        double vel_voltage = kv() * speed;
        // V = IR + kv*w = IR + V_vel
        // I = (V - V_vel) / R
        return (voltage - vel_voltage) / internal_resistance();
      }

      double get_torque(double current) const override {
        // I = kt * t, t = I / kt
        return current / kt();
      }

      double get_free_speed(double voltage) const override {
        // V = kv * w, w = V / kv
        return voltage / kv();
      }

      double get_free_voltage(double speed) const override {
        // V_w = kv * w
        return kv() * speed;
      }

      double get_current_voltage(double current) const override {
        // V_I = IR
        return current * internal_resistance();
      }

      double get_torque_current(double torque) const override {
        // I = kt * t
        return kt() * torque;
      }

     private:
      double _v_nom = 12;
      double _free_speed;
      double _free_current;
      double _stall_current;
      double _stall_torque;
    };
  }  // namespace transmission
}  // namespace pf
}  // namespace grpl