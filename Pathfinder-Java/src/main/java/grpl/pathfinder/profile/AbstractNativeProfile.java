package grpl.pathfinder.profile;

import grpl.pathfinder.util.NativeResource;

public class AbstractNativeProfile extends NativeResource implements Profile {
    private int _limitedTerm;

    AbstractNativeProfile(long handle, int limitedTerm) {
        super(handle);
        this._limitedTerm = limitedTerm;
    }

    @Override
    public State createState() {
        return new State(this.getLimitedTerm());
    }

    @Override
    public void setGoal(double goal) {
        setGoal(nativeHandle(), goal);
    }

    @Override
    public double getGoal() {
        return getGoal(nativeHandle());
    }

    @Override
    public void setTimeslice(double timeslice) {
        setTimeslice(nativeHandle(), timeslice);
    }

    @Override
    public double getTimeslice() {
        return getTimeslice(nativeHandle());
    }

    @Override
    public void applyLimit(int derivative, double min, double max) {
        applyLimit(nativeHandle(), derivative, min, max);
    }

    @Override
    public State calculate(State last, double time) {
        State seg = createState();
        seg.time = time;
        seg.kinematics = calculateNative(nativeHandle(), last.kinematics, last.time, time);
        return seg;
    }

    @Override
    public int getLimitedTerm() {
        return _limitedTerm;
    }

    @Override
    public void close() {
        free(nativeHandle());
        zeroHandle();
    }

    /* JNI */
    private static native double[] calculateNative(long h, double[] last, double lastTime, double time);

    private static native void setGoal(long h, double g);
    private static native double getGoal(long h);
    private static native void setTimeslice(long h, double ts);
    private static native double getTimeslice(long h);
    private static native void applyLimit(long h, int d, double min, double max);

    private static native void free(long h);

}
