package grpl.pathfinder.coupled;

import grpl.pathfinder.transmission.DcTransmission;
import grpl.pathfinder.transmission.JavaDcTransmissionAdapter;
import grpl.pathfinder.util.NativeResource;

/**
 * Mathematical model representation of a coupled (tank / differential) drivetrain.
 * <p>
 * Chassis contains members for the transmissions (motors), as well as other configurations
 * regarding the chassis (track radius, wheel radius, mass, etc).
 * </p>
 * <p>
 * The chassis mirrors the physical "layout" of the drivetrain.
 * </p>
 * <p>
 * {@link CoupledCausalTrajGen}
 */
public class CoupledChassis extends NativeResource {

    private DcTransmission transLeft, transRight;
    private NativeResource nativeLeft, nativeRight;
    private double mass, trackRadius, wheelRadius;

    /**
     * Construct a coupled chassis
     *
     * @param left        The left side transmission.
     * @param right       The right side transmission.
     * @param wheelRadius The wheel radius, in metres. Should be emperically measured
     *                    for best performance. Note that both the left and right
     *                    transmission wheels must be the same radius.
     * @param trackRadius The track radius, a.k.a half the chassis width, in metres.
     *                    Measured as half the distance between the centres of the left
     *                    and right transmissions at their points of contact with the ground.
     * @param mass        The mass of the chassis, in kilograms.
     */
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

    /**
     * @return The mass of the chassis, in kilograms
     */
    public double getMass() {
        return mass;
    }

    /**
     * @return The track radius (half the chassis width) in metres. Measured
     * between the left and right transmissions.
     */
    public double getTrackRadius() {
        return trackRadius;
    }

    /**
     * @return The wheel radius, in metres.
     */
    public double getWheelRadius() {
        return wheelRadius;
    }

    /**
     * @return The left-side transmission of the chassis.
     */
    public DcTransmission getLeft() {
        return transLeft;
    }

    /**
     * @return The right-side transmission of the chassis.
     */
    public DcTransmission getRight() {
        return transRight;
    }

    /**
     * Calculate the absolute linear (translational) velocity limit of the chassis in metres
     * per second (ms^-1).
     * <p>
     * This calculation relates purely to the free-speed of the motors, meaning for a fully
     * constrained calculation, {@link #accelerationLimit(CoupledConfigurationState, double, double)}
     * should be called and used to constrain the velocity if necessary.
     * </p>
     *
     * @param configuration The configuration of the chassis
     * @param curvature     The instantaneous curvature, in metres^-1, expected of the chassis,
     *                      such as the curvature of a path being followed.
     * @return The absolute linear (translational) velocity limit in metres per second (ms^-2).
     */
    public double linearVelocityLimit(CoupledConfigurationState configuration, double curvature) {
        return linvelLimit(nativeHandle(), configuration.toArray(), curvature);
    }

    /**
     * Calculate the minimum and maximum linear (translational) acceleration limits of the chassis,
     * in metres per second per second (ms^-2).
     * <p>
     * This calculation uses torque limits of the transmissions, meaning speed limits
     * are not directly taken into account. For a fully constrained representation,
     * </p>
     * {@link #linearVelocityLimit(CoupledConfigurationState, double)} must also be called to constrain
     * if necessary.
     *
     * @param configuration The configuration of the chassis
     * @param curvature     The instantaneous curvature, in metres^-1, expected of the chassis,
     *                      such as the curvature of a path being followed.
     * @param velocity      The current linear velocity of the chassis, in metres per second (ms^-1).
     * @return A pair, ordered [min, max], of the linear acceleration limits, in metres per second
     * per second (ms^-2).
     */
    public double[] accelerationLimit(CoupledConfigurationState configuration, double curvature, double velocity) {
        return accLimit(nativeHandle(), configuration.toArray(), curvature, velocity);
    }

    /**
     * Split a centre state of this chassis into the left and right transmission state components.
     *
     * @return A pair, ordered [left, right], of {@link CoupledWheelState}.
     */
    public CoupledWheelState[] split(CoupledState centre) {
        double[] left = new double[9];
        double[] right = new double[9];

        splitNative(nativeHandle(), centre.toArray(), left, right);

        return new CoupledWheelState[]{
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
