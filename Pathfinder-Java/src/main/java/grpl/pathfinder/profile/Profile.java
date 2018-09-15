package grpl.pathfinder.profile;

public interface Profile {

    static final int POSITION = 0;
    static final int VELOCITY = 1;
    static final int ACCELERATION = 2;

    public static class Segment {
        public double time = 0;
        public double[] kinematics = new double[3];

        public Segment() {}
    }

    void setGoal(double goal);
    double getGoal();

    void setTimeslice(double timeslice);
    double getTimeslice();

    void applyLimit(int derivative, double min, double max);

    Segment calculate(Segment last, double time);

}
