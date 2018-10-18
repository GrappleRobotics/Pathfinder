package grpl.pathfinder.profile;

public class TrapezoidalProfile extends AbstractNativeProfile {

    public TrapezoidalProfile() {
        super(allocate(), ACCELERATION);
    }

    private static native long allocate();

}
