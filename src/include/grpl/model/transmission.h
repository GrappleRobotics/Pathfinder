#pragma once

namespace grpl {
namespace transmission {
  // Purely Functional DC Motor.
  // TODO: Are there any cases where our transmission model isn't pure?
  class dc_transmission {
   public:
    virtual double get_current(double voltage, double speed) const = 0;
    virtual double get_torque(double current) const                = 0;
    virtual double get_free_speed(double voltage) const            = 0;
  };

  // Basic DC Motor Model, dervied from the ideal resistive motor model with Back EMF
  // (+ ---[ R ]---( V_w )--- -), where R = V / I_stall (as V_w = 0 at stall), and
  // V_w = kv*w.
  class dc_motor : public dc_transmission {
   public:
    dc_motor(double v_nom, double free_speed, double free_current, double stall_current, double stall_torque)
        : _v_nom(v_nom),
          _free_speed(free_speed),
          _free_current(free_current),
          _stall_current(stall_current),
          _stall_torque(stall_torque) {}

    double internal_resistance() const { return _v_nom / _stall_current; }

    double get_current(double voltage, double speed) const override {
      double r_int = internal_resistance();
      double kv    = (_v_nom - _free_current * r_int) / _free_speed;
      // V_w = kv * w
      double vel_voltage = kv * speed;
      // V = IR + kv*w = IR + V_vel
      // I = (V - V_vel) / R
      return (voltage - vel_voltage) / r_int;
    }

    double get_torque(double current) const override {
      // I = kt * t, t = I / kt
      double kt = _stall_current / _stall_torque;
      return current / kt;
    }

    double get_free_speed(double voltage) const override {
      double r_int = internal_resistance();
      double kv    = (_v_nom - _free_current * r_int) / _free_speed;
      // V = kv * w, w = V / kv
      return voltage / kv;
    }

   private:
    double _v_nom = 12;
    double _free_speed;
    double _free_current;
    double _stall_current;
    double _stall_torque;
  };
}  // namespace transmission
}  // namespace grpl