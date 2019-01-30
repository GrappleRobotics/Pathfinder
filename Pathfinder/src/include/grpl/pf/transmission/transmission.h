#pragma once

namespace grpl {
namespace pf {
  /**
   * Transmissions.
   *
   * The grpl::pf::transmission namespace contains implementations of transmissions, which convert
   * control signals to motion (e.g. motors).
   */
  namespace transmission {
    
    inline double rpm_to_rad(double rpm) {
      return rpm * 2.0 * constants::PI / 60.0;
    }

    inline double rad_to_rpm(double rad_per_sec) {
      return rad_per_sec * 60.0 / (2 * constants::PI);
    }

    /**
     * Interface for any transmission
     *
     * A transmission converts a scalar control signal to motion, in the form of free angular speed
     * (rad/s) and torque (Nm).
     *
     * A transmission functions with respect to the control signal, which lays in the range of -1 to 1.
     * The control signal is the percentage of maximum throttle / input signal. In a DC motor, it is
     * the same as the percentage of nominal voltage. The control signal is set to be between -1 and 1
     * in order to stay generic for different forms of transmission (be it electrical, automotive or
     * otherwise).
     */
    class transmission {
     public:
      virtual ~transmission(){};

      /**
       * Calculate the free (no load) speed of the transmission at the applied control signal.
       *
       * @param signal  The control signal of the transmission, in the range -1 <= signal <= 1.
       * @return The angular free speed of the transmission, in rad/s.
       */
      virtual double free_speed(double signal = 1.0) = 0;

      /**
       * Calculate the torque applied by the transmission at a given speed and control signal.
       *
       * @param speed   The angular speed of the transmission, in rad/s.
       * @param signal  The control signal of the transmission, in the range -1 <= signal <= 1.
       * @return The torque applied by the transmission, in Nm.
       */
      virtual double torque(double speed, double signal = 1.0) = 0;

      /**
       * Obtain the partial control signal given the free angular speed of the transmission.
       * 
       * Inverse of @ref free_speed(double). Sum with @ref partial_signal_at_torque(double) const
       * in order to get the full signal.
       * 
       * @param free_speed  The angular speed of the transmission, in rad/s.
       * @return The partial control signal for the speed component of the transmission.
       */
      virtual double partial_signal_at_speed(double free_speed) const = 0;

      /**
       * Obtain the partial control signal given the torque of the transmission.
       * 
       * Inverse of @ref torque(double, double). Sum with @ref partial_signal_at_speed(double) const
       * in order to get the full signal.
       * 
       * @param torque  The torque applied by the transmission, in Nm.
       * @return The partial control signal for the torque component of the transmission.
       */
      virtual double partial_signal_at_torque(double torque) const    = 0;
    };
  }  // namespace transmission
}  // namespace pf
}  // namespace grpl