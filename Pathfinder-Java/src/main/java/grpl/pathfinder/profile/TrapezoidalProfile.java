package grpl.pathfinder.profile;

public class TrapezoidalProfile extends AbstractProfile {

    TrapezoidalProfile() {
        super(allocate(), ACCELERATION);
    }

    private static native long allocate();

}
