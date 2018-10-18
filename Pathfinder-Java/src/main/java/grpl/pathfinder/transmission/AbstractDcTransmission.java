package grpl.pathfinder.transmission;

import grpl.pathfinder.util.NativeResource;

public class AbstractDcTransmission extends NativeResource implements DcTransmission {

    public AbstractDcTransmission(long handle) {
        super(handle);
    }

    @Override
    public double getFreeSpeed(double voltage) {
        return freeSpeed(nativeHandle(), voltage);
    }

    @Override
    public double getCurrent(double voltage, double speed) {
        return current(nativeHandle(), voltage, speed);
    }

    @Override
    public double getTorque(double current) {
        return torque(nativeHandle(), current);
    }

    @Override
    public double getFreeVoltage(double speed) {
        return freeVoltage(nativeHandle(), speed);
    }

    @Override
    public double getCurrentVoltage(double current) {
        return currentVoltage(nativeHandle(), current);
    }

    @Override
    public double getTorqueCurrent(double torque) {
        return torqueCurrent(nativeHandle(), torque);
    }

    @Override
    public double getNominalVoltage() {
        return nominalVoltage(nativeHandle());
    }

    @Override
    public void close() {
        free(nativeHandle());
        zeroHandle();
    }

    /* JNI */
    private static native double freeSpeed(long h, double voltage);
    private static native double current(long h, double voltage, double speed);
    private static native double torque(long h, double current);

    private static native double freeVoltage(long h, double speed);
    private static native double currentVoltage(long h, double current);
    private static native double torqueCurrent(long h, double torque);

    private static native double nominalVoltage(long h);

    private static native void free(long h);
}
