package grpl.pathfinder.coupled;

import grpl.pathfinder.path.Curve2d;
import grpl.pathfinder.path.Spline2d;
import grpl.pathfinder.profile.Profile;
import grpl.pathfinder.util.NativeResource;

import java.util.List;

public class CoupledDrivetrain extends NativeResource {

    // Keep CoupledChassis in here so we still have ownership (it won't get GC'd
    // out of nowhere)
    private CoupledChassis chassis;

    public CoupledDrivetrain(CoupledChassis chassis) {
        super(allocate(chassis.nativeHandle()));
        this.chassis = chassis;
    }

    // TODO: Profile and curves should be in the constructor, in Java it's expensive to convert them every
    // iteration.
    public CoupledState generate(List<Curve2d> curves, Profile profile, CoupledState lastState, double time) {
        return null;
    }

    public CoupledWheelState[] split(CoupledState centre) {
        return null;
    }

    @Override
    public void close() throws Exception {
        free(nativeHandle());
        zeroHandle();
    }

    /* JNI */
    private static native long allocate(long chassisHandle);
    private static native void free(long handle);

    // Curve buffer
    private static native long acquireBuffer();
    private static native void enqueueNative(long bufferHandle, long curveHandle);
    private static native void enqueueAdapter(long bufferHandle, Curve2d curve);
    private static native void releaseBuffer(long bufferHandle);
}
