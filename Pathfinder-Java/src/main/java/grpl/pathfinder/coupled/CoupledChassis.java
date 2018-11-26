package grpl.pathfinder.coupled;

import grpl.pathfinder.transmission.DcTransmission;
import grpl.pathfinder.transmission.JavaDcTransmissionAdapter;
import grpl.pathfinder.util.NativeResource;

public class CoupledChassis extends NativeResource {

    private DcTransmission transLeft, transRight;
    private NativeResource nativeLeft, nativeRight;
    private double mass, trackRadius, wheelRadius;

    public CoupledChassis(DcTransmission left, DcTransmission right, double wheelRadius, double trackRadius, double mass) {
        super(allocateStub());
        this.transLeft = left;
        this.transRight = right;
        this.nativeLeft = left instanceof NativeResource ? (NativeResource) left : new JavaDcTransmissionAdapter(left);
        this.nativeRight = right instanceof NativeResource ? (NativeResource) right : new JavaDcTransmissionAdapter(right);
        this.mass = mass;
        this.trackRadius = trackRadius;
        this.wheelRadius = wheelRadius;
        construct(nativeHandle(), nativeLeft.nativeHandle(), nativeRight.nativeHandle(), wheelRadius, trackRadius, mass);
    }

    public double getMass() {
        return mass;
    }

    public double getTrackRadius() {
        return trackRadius;
    }

    public double getWheelRadius() {
        return wheelRadius;
    }

    public DcTransmission getLeft() {
        return transLeft;
    }

    public DcTransmission getRight() {
        return transRight;
    }

    double linearVelocityLimit(CoupledConfigurationState configuration, double curvature) {
        return linvelLimit(nativeHandle(), configuration.toArray(), curvature);
    }

    double[] accelerationLimit(CoupledConfigurationState configuration, double curvature, double velocity) {
        return accLimit(nativeHandle(), configuration.toArray(), curvature, velocity);
    }

    public CoupledWheelState[] split(CoupledState centre) {
        double[] left = new double[9];
        double[] right = new double[9];

        splitNative(nativeHandle(), centre.toArray(), left, right);

        return new CoupledWheelState[] {
                new CoupledWheelState(left), new CoupledWheelState(right)
        };
    }

    @Override
    public void close() {
        free(nativeHandle());
        zeroHandle();

        if (nativeLeft instanceof JavaDcTransmissionAdapter)
            nativeLeft.close();
        if (nativeRight instanceof JavaDcTransmissionAdapter)
            nativeRight.close();
    }

    /* JNI */
    private static native long allocateStub();
    private static native void construct(long h, long left, long right, double wheelR, double trackR, double mass);
    private static native void free(long h);

    private static native double linvelLimit(long h, double[] config, double curvature);
    private static native double[] accLimit(long h, double[] config, double curvature, double vel);

    private static native void splitNative(long h, double[] centre, double[] outLeft, double[] outRight);
}
