#pragma once

#include "transmission.h"

namespace grpl {
namespace pf {
  namespace transmission {
    /**
     * Mathematical Model of a DC Brushed Motor
     *
     * Basic DC Motor Model, dervied from the ideal resistive motor model with Back EMF
     * (+ ---[ R ]---( V_w )--- -), where R = V / I_stall (as V_w = 0 at stall), and
     * V_w = kw*w.
     */
    class dc_motor : public transmission {
     public:
      /**
       * Construct a DC Brushed Motor Model.
       *
       * @param v_nom         The nominal operating voltage of the motor, in Volts
       * @param free_speed    The maximum free (no load) speed of the motor at v_nom, in rad/s
       * @param free_current  The current drawn when the motor is running at free_speed with no load
       * @param stall_current The current drawn when v_nom is applied with a locked rotor (stalled)
       * @param stall_torque  The torque applied by the motor at v_nom with a locked rotor (stalled)
       */
      dc_motor(double v_nom, double free_speed, double free_current, double stall_current,
               double stall_torque)
          : _v_nom(v_nom),
            _free_speed(free_speed),
            _free_current(free_current),
            _stall_current(stall_current),
            _stall_torque(stall_torque) {
        _rint = _v_nom / _stall_current;
        _kt   = _stall_current / _stall_torque;
        _kw   = (_v_nom - _free_current * _v_nom / _stall_current) / _free_speed;
      }

      // === MOTOR PARAMETERS === //

      /**
       * Get the nominal voltage of the motor (the typical maximum operating voltage).
       *
       * @return The nominal voltage of the motor, in Volts.
       */
      double nominal_voltage() const { return _v_nom; }

      /**
       * Get the internal resistance of the motor (the resistance of its windings).
       *
       * @return The internal winding resistance of the motor, in Ohms.
       */
      double internal_resistance() const { return _rint; }

      /**
       * Get the angular velocity voltage coefficient of the motor, determined by
       * V = kw*w, kw = (V_nom - I_free * V_nom / I_stall) / w_free.
       *
       * @return The angular velocity voltage coefficient of the motor, in Vs/rad.
       */
      double kw() const { return _kw; }

      /**
       * Get the torque current coefficient of the motor, determined by I = kt*t,
       * kt = I_stall / t_stall.
       *
       * @return The torque current coeffient of the motor, in A/(Nm).
       */
      double kt() const { return _kt; }

      // === FORWARD MODEL === //

      double free_speed(double signal) override {
        // V = kw * w, w = V / kw
        return (signal * _v_nom) / _kw;
      }

      double torque(double speed, double signal) override {
        // V_w = kw * w
        double vel_voltage = _kw * speed;
        // V = IR + kw*w = IR + V_vel
        // I = (V - V_vel) / R
        double current = (signal * _v_nom - vel_voltage) / _rint;
        // I = kt * t, t = I / kt
        return current / _kt;
      }

      // === INVERSE MODEL === //

      double partial_signal_at_speed(double free_speed) const override {
        // V_w = kw * w
        return (_kw * free_speed) / _v_nom;
      }

      double partial_signal_at_torque(double torque) const override {
        // I = kt * t
        double current = _kt * torque;
        // V_I = IR
        return (current * _rint) / _v_nom;
      }

      // === ELECTRICAL CHARACTERISTICS === //

      /**
       * Convert a given control signal to a voltage applied to the motor.
       *
       * @param signal  The control signal applied to the motor.
       * @return  The voltage applied given the control signal, in Volts.
       */
      double signal_to_voltage(double signal) const { return signal * _v_nom; }

      /**
       * Calculate the current drawn by the motor given a control signal and speed.
       *
       * @param speed   The angular speed of the motor, in rad/s.
       * @param signal  The control signal applied to the motor.
       * @return The current drawn by the motor, in Amps.
       */
      double signal_to_current(double speed, double signal) const {
        double voltage = (signal - partial_signal_at_speed(speed)) * _v_nom;
        return voltage / _rint;
      }

      /**
       * Convert the torque applied by the motor to the current drawn by the motor.
       *
       * @param torque  The torque applied by the motor, in Nm.
       * @return The current drawn by the motor, in Amps.
       */
      double torque_to_current(double torque) {
        // I = kt * t
        return _kt * torque;
      }

     private:
      //! The nominal operating voltage of the motor, in Volts
      double _v_nom;
      //! The maximum free (no load) speed of the motor at v_nom, in rad/s
      double _free_speed;
      //! The current drawn when the motor is running at free speed with no load
      double _free_current;
      //! The current drawn when v_nom is applied with a locked rotor (stalled)
      double _stall_current;
      //! The torque applied by the motor at v_nom with a locked rotor (stalled)
      double _stall_torque;

      //! Internal resistance of the motor, in Ohms.
      double _rint;
      //! The speed-voltage coefficient of the motor, in Vs/rad (kw in V = kw*w)
      double _kw;
      //! The torque-current coefficient of the motor, in A/(Nm) (kt in I = kt*t)
      double _kt;
    };
  }  // namespace transmission
}  // namespace pf
}  // namespace grpl