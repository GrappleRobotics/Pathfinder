package grpl.pathfinder.coupled;

import grpl.pathfinder.path.Curve2d;
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
}
