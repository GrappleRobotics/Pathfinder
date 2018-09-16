package grpl.pathfinder.profile;

public class TrapezoidalProfile extends AbstractProfile {

    TrapezoidalProfile() {
        super(allocate(), ACCELERATION);
    }

    private static native long allocate();

    @Override
    protected native double[] calculateNative(long h, double[] last, double lastTime, double time);
}
