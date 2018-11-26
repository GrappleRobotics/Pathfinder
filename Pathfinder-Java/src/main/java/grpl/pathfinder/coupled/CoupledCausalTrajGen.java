package grpl.pathfinder.coupled;

import grpl.pathfinder.path.Curve2d;
import grpl.pathfinder.path.Spline2d;
import grpl.pathfinder.profile.Profile;
import grpl.pathfinder.util.INativeResource;
import grpl.pathfinder.util.NativeResource;

import java.util.List;

public class CoupledCausalTrajGen extends NativeResource {

    private CoupledChassis chassis;
    private long nativeCurveBuffer = 0L;

    private Profile profile;
    private NativeResource nativeProfile;

    public static final int LEFT = 0;
    public static final int RIGHT = 1;

    public CoupledCausalTrajGen(CoupledChassis chassis) {
        super(allocate());
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
