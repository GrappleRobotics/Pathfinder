package grpl.pathfinder.profile;

/**
 * Interface class for all motion profile types.
 * <p>
 * A motion profile describes some function that forms the shape of the position-time curve
 * and its derivatives, allowing velocity, acceleration and jerk to be shaped in ways that may
 * be desirable, such as optimizing for speed, safety, or smooth operation.
 * <p>
 * Profiles are predictive in Pathfinder, meaning each calculation does not require a full history
 * of the profile. This allows it to adjust to changing system conditions and limits during a
 * profile, increasing flexibility and efficiency.
 * <p>
 * Since the system is predictive, it may result in a small oscillation or sudden deceleration
 * if a sufficient timestep is not used. For this reason, a timeslice mechanism is included in the
 * profile.
 */
public interface Profile {

    public static final int POSITION = 0;
    public static final int VELOCITY = 1;
    public static final int ACCELERATION = 2;

    /**
     * A single state (sample point) of a motion profile. This contains the kinematics of
     * the profile sampled at a given time.
     */
    public static class State {

        /**
         * The time point of this state, in seconds
         */
        public double time = 0;

        /**
         * The kinematic state of the system at the time of the state.
         *
         * {@link #POSITION}
         * {@link #VELOCITY}
         * {@link #ACCELERATION}
         */
        public double[] kinematics;

        protected State(int limited_term) {
            kinematics = new double[limited_term + 1];
        }
    }

    /**
     * Create a basic 0 state for {@link State}
     * @return A basic, 0 state.
     */
    State createState();

    /**
     * Set the goal (setpoint) of the profile.
     *
     * @param goal The goal (setpoint) of the profile, in metres.
     */
    void setGoal(double goal);

    /**
     * Get the goal (setpoint) of the profile.
     *
     * @return The goal (setpoint) of the profile, in metres.
     */
    double getGoal();

    /**
     * Set the timeslice period.
     * <p>
     * The timeslice is used in {@link #calculate(State, double)}, where a single
     * call will be translated into N smaller calls, where N is ceil(T_calc / T_slice), where T_calc is the
     * period at which {@link #calculate(State, double)} is called, and T_slice is the timeslice period.
     * <p>
     * Timeslice is used to counteract slow calls to {@link #calculate(State, double)} in order to still
     * produce smooth curves.
     *
     * @param timeslice The timeslice period T_slice, in seconds.
     */
    void setTimeslice(double timeslice);

    /**
     * Get the timeslice period.
     *
     * @return The timeslice period, T_slice, in seconds.
     */
    double getTimeslice();

    /**
     * Apply a constrained limit to the profile. This will limit the maximum and minimum value of
     * this term during the profile (e.g. maximum velocity / acceleration).
     * <p>
     * These limits may be changed between calls to {@link #calculate(State, double)}, but if the maximum
     * or minimum is lower in magnitude, it may cause sudden decelerations.
     *
     * @param derivative The term to apply the limit to.
     * @param min        The minimum value of the term, in the units of the term
     * @param max        The maximum value of the term, in the units of the term
     */
    void applyLimit(int derivative, double min, double max);

    /**
     * Calculate a single state of the motion profile, in a predictive manner.
     * <p>
     * This method function uses the last (current) state of the system in order to generate the following
     * state at the time given. It does this in a predictive manner, meaning it predicts when it must
     * begin slowing down in order to not overshoot the setpoint. This means the profile calculation does
     * not require a full history of the profile, allowing it to adjust to changing system conditions and
     * limits.
     */
    State calculate(State last, double time);

    /**
     * Get the index of the limited term (the highest order, non-infinite term).
     */
    int getLimitedTerm();

}
