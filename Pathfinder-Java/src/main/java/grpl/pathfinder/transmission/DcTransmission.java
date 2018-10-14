package grpl.pathfinder.transmission;

public interface DcTransmission {

    double getFreeSpeed(double voltage);
    double getCurrent(double voltage, double speed);
    double getTorque(double current);

    double getFreeVoltage(double speed);
    double getCurrentVoltage(double current);
    double getTorqueCurrent(double torque);

    double getNominalVoltage();

}
