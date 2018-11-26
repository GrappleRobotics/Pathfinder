package grpl.pathfinder.profile;

public interface Profile {

    static final int POSITION = 0;
    static final int VELOCITY = 1;
    static final int ACCELERATION = 2;

    public static class State
    {
        public double time = 0;
        public double[] kinematics;

        protected State(int limited_term) {
            kinematics = new double[limited_term + 1];
        }
    }

    State createState();

    void setGoal(double goal);
    double getGoal();

    void setTimeslice(double timeslice);
    double getTimeslice();

    void applyLimit(int derivative, double min, double max);

    State calculate(State last, double time);

    int getLimitedTerm();

}
