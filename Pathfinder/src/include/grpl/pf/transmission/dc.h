#pragma once

namespace grpl {
namespace pf {
  /**
   * Motor transmissions.
   *
   * The grpl::pf::transmission namespace contains implementations of transmissions, which convert
   * control signals to motion (e.g. motors).
   */
  namespace transmission {

    /**
     * Base class of a DC electric transmission, usually @ref grpl::pf::transmission::dc_motor.
     */
    class dc_transmission {
     public:
      // TODO: Need a better naming scheme
      virtual ~dc_transmission(){};

      /**
       * Calculate the free (no load) speed of the transmission at an applied voltage.
       *
       * @param voltage The voltage applied to the transmission, in Volts
       * @return The free speed of the transmission, in rad/s
       */
      virtual double get_free_speed(double voltage) const = 0;

      /**
       * Calculate the current draw of the transmission at an applied voltage and speed.
       *
       * @param voltage The voltage applied to the transmission, in Volts
       * @param speed   The current speed of the transmission, in rad/s
       * @return The current drawn by the transmission, in Amps
       */
      virtual double get_current(double voltage, double speed) const = 0;

      /**
       * Calculate the torque applied by the transmission at a given current draw.
       *
       * @param current The current drawn by the transmission, calculated in
       *                @ref get_current(double, double) const, in Amps
       * @return The torque applied by the transmission, in Nm
       */
      virtual double get_torque(double current) const = 0;

      /**
       * Calculate the component voltage applied to the transmission in order to obtain
       * a free speed.
       *
       * To obtain the full applied voltage, sum @ref get_free_voltage(double) const and
       * @ref get_current_voltage(double) const.
       *
       * @param speed The speed of the transmission, in rad/s
       * @return The free voltage component of the transmission, in Volts
       */
      virtual double get_free_voltage(double speed) const = 0;

      /**
       * Calculate the component voltage applied to the transmission in order to draw
       * a current.
       *
       * Current is usually provided by @ref get_torque_current(double) const
       *
       * To obtain the full applied voltage, sum @ref get_free_voltage(double) const and
       * @ref get_current_voltage(double) const.
       *
       * @param current The current drawn by the transmission, in Amps
       * @return The current voltage component of the transmission, in Volts
       */
      virtual double get_current_voltage(double current) const = 0;

      /**
       * Calculate the current draw of the transmission given a torque.
       *
       * @param torque The torque applied by the transmission, in Nm.
       * @return The current draw required to apply the torque, in Amps.
       */
      virtual double get_torque_current(double torque) const = 0;

      /**
       * Get the nominal, operating voltage of the transmission.
       *
       * @return The nominal, operating voltage of the transmission, in Volts
       */
      virtual double nominal_voltage() const = 0;
    };

    /**
     * Mathematical Model of a DC Brushed Motor
     *
     * Basic DC Motor Model, dervied from the ideal resistive motor model with Back EMF
     * (+ ---[ R ]---( V_w )--- -), where R = V / I_stall (as V_w = 0 at stall), and
     * V_w = kv*w.
     */
    class dc_motor : public dc_transmission {
     public:
      /**
       * Construct a DC Brushed Motor Model.
       *
       * @param v_nom         The nominal operating voltage of the motor, in Volts
       * @param free_speed    The maximum free (no load) speed of the motor at v_nom, in rad/s
       * @param free_current  The current drawn when the motor is running at free_speed
       *                      with no load
       * @param stall_current The current drawn when v_nom is applied with a locked rotor (stalled)
       * @param stall_torque  The torque applied by the motor at v_nom with a locked rotor (stalled)
       */
      dc_motor(double v_nom, double free_speed, double free_current, double stall_current,
               double stall_torque)
          : _v_nom(v_nom),
            _free_speed(free_speed),
            _free_current(free_current),
            _stall_current(stall_current),
            _stall_torque(stall_torque) {}

      /**
       * Calculate the internal resistance of the motor windings
       *
       * @return the internal resistance of the motor, in Ohms
       */
      inline double internal_resistance() const { return _v_nom / _stall_current; }

      /**
       * Calculate the speed-voltage coefficient of the motor (kv in V = kv*w)
       *
       * @return The speed-voltage coefficient of the motor, in Vs/rad
       */
      inline double kv() const { return (_v_nom - _free_current * _v_nom / _stall_current) / _free_speed; }

      /**
       * Calculate the torque-current coefficient of the motor (kt in I = kt*t)
       *
       * @return The torque-current coefficient of the motor, in A/(Nm)
       */
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