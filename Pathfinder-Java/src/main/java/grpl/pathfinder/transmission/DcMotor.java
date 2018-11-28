package grpl.pathfinder.transmission;

/**
 * Mathematical Model of a DC Brushed Motor
 * <p>
 * Basic DC Motor Model, dervied from the ideal resistive motor model with Back EMF
 * (+ ---[ R ]---( V_w )--- -), where R = V / I_stall (as V_w = 0 at stall), and
 * V_w = kv*w.
 */
public class DcMotor extends AbstractDcTransmission {

    /**
     * Construct a DC Brushed Motor Model.
     *
     * @param voltageNom   The nominal operating voltage of the motor, in Volts
     * @param freeSpeed    The maximum free (no load) speed of the motor at v_nom, in rad/s
     * @param freeCurrent  The current drawn when the motor is running at free_speed
     *                     with no load
     * @param stallCurrent The current drawn when v_nom is applied with a locked rotor (stalled)
     * @param stallTorque  The torque applied by the motor at v_nom with a locked rotor (stalled)
     */
    public DcMotor(double voltageNom, double freeSpeed, double freeCurrent, double stallCurrent, double stallTorque) {
        super(allocate(voltageNom, freeSpeed, freeCurrent, stallCurrent, stallTorque));
    }

    private static native long allocate(double voltageNom, double freeSpeed, double freeCurrent, double stallCurrent, double stallTorque);

}
