package grpl.pathfinder.transmission;

/**
 * Interface class of a DC electric transmission, usually {@link DcMotor}
 */
public interface DcTransmission {

    /**
     * Calculate the free (no load) speed of the transmission at an applied voltage.
     *
     * @param voltage The voltage applied to the transmission, in Volts
     * @return The free speed of the transmission, in rad/s
     */
    double getFreeSpeed(double voltage);

    /**
     * Calculate the current draw of the transmission at an applied voltage and speed.
     *
     * @param voltage The voltage applied to the transmission, in Volts
     * @param speed   The current speed of the transmission, in rad/s
     * @return The current drawn by the transmission, in Amps
     */
    double getCurrent(double voltage, double speed);

    /**
     * Calculate the torque applied by the transmission at a given current draw.
     *
     * @param current The current drawn by the transmission, calculated in
     *                {@link #getCurrent(double, double)}, in Amps
     * @return The torque applied by the transmission, in Nm
     */
    double getTorque(double current);

    /**
     * Calculate the component voltage applied to the transmission in order to obtain
     * a free speed.
     * <p>
     * To obtain the full applied voltage, sum {@link #getFreeVoltage(double)} and
     * {@link #getCurrentVoltage(double)}.
     *
     * @param speed The speed of the transmission, in rad/s
     * @return The free voltage component of the transmission, in Volts
     */
    double getFreeVoltage(double speed);

    /**
     * Calculate the component voltage applied to the transmission in order to draw
     * a current.
     * <p>
     * Current is usually provided by {@link #getTorqueCurrent(double)}
     * <p>
     * To obtain the full applied voltage, sum {@link #getFreeVoltage(double)} and
     * {@link #getCurrentVoltage(double)}.
     *
     * @param current The current drawn by the transmission, in Amps
     * @return The current voltage component of the transmission, in Volts
     */
    double getCurrentVoltage(double current);

    /**
     * Calculate the current draw of the transmission given a torque.
     *
     * @param torque The torque applied by the transmission, in Nm.
     * @return The current draw required to apply the torque, in Amps.
     */
    double getTorqueCurrent(double torque);

    /**
     * Get the nominal, operating voltage of the transmission.
     *
     * @return The nominal, operating voltage of the transmission, in Volts
     */
    double getNominalVoltage();

}
