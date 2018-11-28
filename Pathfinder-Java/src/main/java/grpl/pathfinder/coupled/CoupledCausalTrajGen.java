package grpl.pathfinder.coupled;

import grpl.pathfinder.path.Curve2d;
import grpl.pathfinder.profile.Profile;
import grpl.pathfinder.util.INativeResource;
import grpl.pathfinder.util.NativeResource;

import java.util.List;

/**
 * Causal trajectory generator, given the chassis, path and motion profile.
 * <p>
 * This class provides the necessary plumbing in order to generate trajectories from a given path,
 * applying motion and timing information based on the limits and constraints of a drivetrain. The
 * generator applies a motion profile to a path, while fitting within given constraints.
 * </p>
 * <p>
 * This generator is causal and will generate a single state (output) with knowledge only of the current
 * state of the system. This generator can be used on-the-fly, and is primarily advantageous in speed and
 * flexibility. The cost is accuracy towards the end of the path, as a sudden deceleration may be required
 * with insufficient time steps, as the end velocity may not be zero.
 * </p>
 * <p>
 * It is recommended to use this generator if on-the-fly generation is required, or memory and
 * computational resources are sparse. This generator also enables the ability to provide feedback
 * information on chassis kinematics to the next generation step, making it possible to embed a feedback
 * loop into the generation phase.
 * </p>
 */
public class CoupledCausalTrajGen extends NativeResource {

    private CoupledChassis chassis;
    private long nativeCurveBuffer = 0L;

    private Profile profile;
    private NativeResource nativeProfile;

    public static final int LEFT = 0;
    public static final int RIGHT = 1;

    /**
     * Create a new Causal Trajectory Generator for a given chassis.
     *
     * @param chassis The coupled chassis, used to provide limits for the trajectory the trajectory
     *                kinematics.
     */
    public CoupledCausalTrajGen(CoupledChassis chassis) {
        super(allocate());
        this.chassis = chassis;
    }

    /**
     * Configure the curves and profile to use for the Causal Trajectory Generator.
     * <p>
     * This must be called before any calls to {@link #generate(CoupledState, double)}
     * </p>
     *
     * @param curves  The collection of curves to use for the trajectory (the path)
     * @param profile The profile to use for the trajectory. {@link grpl.pathfinder.profile.TrapezoidalProfile}
     *                is recommended.
     */
    public void configure(List<? extends Curve2d> curves, Profile profile) {
        this.profile = profile;

        if (profile instanceof NativeResource) {
            this.nativeProfile = (NativeResource) profile;
        } else {
            throw new UnsupportedOperationException("Non-native profiles are not yet supported. See issue #11");
        }

        if (this.nativeCurveBuffer != 0L)
            releaseBuffer(this.nativeCurveBuffer);
        this.nativeCurveBuffer = acquireBuffer();

        pack(this.nativeCurveBuffer, curves);
    }

    /**
     * Generate the next state of the trajectory given the current state.
     * <p>
     * Requires a call to {@link #configure(List, Profile)} to setup the system.
     * </p>
     *
     * @param lastState The current ("last") state of the trajectory. If this is the first call to
     *                  generate, this may be considered the "initial conditions".
     * @param time      The time of the next point (lastState.time + dt), in seconds.
     */
    public CoupledState generate(CoupledState lastState, double time) {
        return new CoupledState(generateNative(nativeHandle(), chassis.nativeHandle(), nativeCurveBuffer, nativeProfile.nativeHandle(), lastState.toArray(), time));
    }

    @Override
    public void close() {
        if (nativeCurveBuffer != 0L)
            releaseBuffer(nativeCurveBuffer);

        free(nativeHandle());
        zeroHandle();
    }

    private static void pack(long buffer, List<? extends Curve2d> curves) {
        for (Curve2d curve : curves) {
            if (curve instanceof INativeResource) {
                enqueueNative(buffer, ((INativeResource) curve).nativeHandle());
            } else {
                enqueueAdapter(buffer, curve);
            }
        }
    }

    /* JNI */
    private static native long allocate();

    private static native void free(long handle);

    private static native double[] generateNative(long h, long chassisHandle, long buf, long prof, double[] arr, double time);

    // Curve buffer
    private static native long acquireBuffer();

    private static native void enqueueNative(long bufferHandle, long curveHandle);

    private static native void enqueueAdapter(long bufferHandle, Curve2d curve);

    private static native void releaseBuffer(long bufferHandle);
}
