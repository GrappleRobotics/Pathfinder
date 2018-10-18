package grpl.pathfinder.coupled;

import grpl.pathfinder.path.Curve2d;
import grpl.pathfinder.path.Spline2d;
import grpl.pathfinder.profile.Profile;
import grpl.pathfinder.util.INativeResource;
import grpl.pathfinder.util.NativeResource;

import java.util.List;

public class CoupledDrivetrain extends NativeResource {

    // Keep CoupledChassis in here so we still have ownership (it won't get GC'd
    // out of nowhere)
    private CoupledChassis chassis;
    private long nativeCurveBuffer = 0L;

    private Profile profile;
    private NativeResource nativeProfile;

    public static final int LEFT = 0;
    public static final int RIGHT = 1;

    public CoupledDrivetrain(CoupledChassis chassis) {
        super(allocate(chassis.nativeHandle()));
        this.chassis = chassis;
    }

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

    public CoupledState generate(CoupledState lastState, double time) {
        return new CoupledState(generateNative(nativeHandle(), nativeCurveBuffer, nativeProfile.nativeHandle(), lastState.toArray(), time));
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
    private static native long allocate(long chassisHandle);
    private static native void free(long handle);

    private static native double[] generateNative(long h, long buf, long prof, double[] arr, double time);
    private static native void splitNative(long h, double[] centre, double[] outLeft, double[] outRight);

    // Curve buffer
    private static native long acquireBuffer();
    private static native void enqueueNative(long bufferHandle, long curveHandle);
    private static native void enqueueAdapter(long bufferHandle, Curve2d curve);
    private static native void releaseBuffer(long bufferHandle);
}
