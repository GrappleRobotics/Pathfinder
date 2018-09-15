package grpl.pathfinder.profile;

public class TrapezoidalProfile extends AbstractProfile {

    TrapezoidalProfile() {
        super(allocate());
    }

    private static native long allocate();

    @Override
    public native Segment calculate(Segment last, double time);

}
