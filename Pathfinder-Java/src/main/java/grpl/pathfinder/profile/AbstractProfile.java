package grpl.pathfinder.profile;

import grpl.pathfinder.util.Destroyable;

public abstract class AbstractProfile extends Destroyable implements Profile {
    private long _handle = 0;

    AbstractProfile(long handle) {
        this._handle = handle;
    }

    @Override
    public native void setGoal(double goal);

    @Override
    public native double getGoal();

    @Override
    public native void setTimeslice(double timeslice);

    @Override
    public native double getTimeslice();

    @Override
    public native void applyLimit(int derivative, double min, double max);

    @Override
    public native void destroy();

    @Override
    public boolean destroyed() {
        return _handle == 0;
    }
}
