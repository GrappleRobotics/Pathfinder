package grpl.pathfinder.transmission;

public class DcMotor extends AbstractDcTransmission {

    public DcMotor(double voltageNom, double freeSpeed, double freeCurrent, double stallCurrent, double stallTorque) {
        super(allocate(voltageNom, freeSpeed, freeCurrent, stallCurrent, stallTorque));
    }

    private static native long allocate(double voltageNom, double freeSpeed, double freeCurrent, double stallCurrent, double stallTorque);

}
