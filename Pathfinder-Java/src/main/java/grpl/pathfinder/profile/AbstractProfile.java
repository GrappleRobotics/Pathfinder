package grpl.pathfinder.profile;

import grpl.pathfinder.util.NativeResource;

public abstract class AbstractProfile extends NativeResource implements Profile {
    private long _handle = 0;
    private int _limitedTerm;

    AbstractProfile(long handle, int limitedTerm) {
        this._handle = handle;
        this._limitedTerm = limitedTerm;
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
    public Segment createSegment() {
        return new Segment(this.getLimitedTerm());
    }

    @Override
    public int getLimitedTerm() {
        return _limitedTerm;
    }

    @Override
    public int getOrder() {
        return _limitedTerm + 1;
    }

    @Override
    public native void close();

    @Override
    public boolean closed() {
        return _handle == 0;
    }
}
