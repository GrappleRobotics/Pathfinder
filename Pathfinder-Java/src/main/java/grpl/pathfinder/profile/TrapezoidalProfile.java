package grpl.pathfinder.profile;

/**
 * Implementation of a trapezoidal (limited acceleration) motion profile.
 * <p>
 * A trapezoidal motion profile is a motion profile limited by acceleration (therefore, infinite jerk).
 * The profile can be described by three distinct sections: ramp-up, hold and ramp-down.
 * <p>
 * During ramp-up, the system is accelerating towards its max velocity.
 * <p>
 * During hold, the system is not accelerating, and holds its max velocity. Depending on the setpoint,
 * the system may not have time to reach hold before it must ramp-down, resulting in a trianglular
 * velocity profile.
 * <p>
 * During ramp-down, the system is decelerating towards 0.
 *
 * See {@link Profile}
 */
public class TrapezoidalProfile extends AbstractNativeProfile {

    public TrapezoidalProfile() {
        super(allocate(), ACCELERATION);
    }

    private static native long allocate();

}
