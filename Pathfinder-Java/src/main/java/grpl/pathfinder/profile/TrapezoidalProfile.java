package grpl.pathfinder.profile;

public class TrapezoidalProfile extends AbstractProfile {

    TrapezoidalProfile() {
        super(allocate(), ACCELERATION);
    }

    private static native long allocate();

    @Override
    public native Segment calculate(Segment last, double time);

}
