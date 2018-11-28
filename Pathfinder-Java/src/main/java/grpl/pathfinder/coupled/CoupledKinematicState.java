package grpl.pathfinder.coupled;

/**
 * Drivetrain kinematic state, describing the movement and motion of the chassis.
 */
public class CoupledKinematicState {

    /**
     * The distance covered by the drivetrain, in metres.
     */
    public double distance;

    /**
     * The linear velocity of the drivetrain, in metres per second (ms^-1).
     */
    public double velocity;

    /**
     * The linear acceleration of the drivetrain, in metres per second
     * per second (ms^-2).
     */
    public double acceleration;

    public CoupledKinematicState() {
        this.distance = 0;
        this.velocity = 0;
        this.acceleration = 0;
    }

    public CoupledKinematicState(double dist, double vel, double acc) {
        this.distance = dist;
        this.velocity = vel;
        this.acceleration = acc;
    }

    public CoupledKinematicState(double[] arr) {
        this(arr[0], arr[1], arr[2]);
    }

    public double[] toArray() {
        return new double[]{distance, velocity, acceleration};
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof CoupledKinematicState))
            return false;
        CoupledKinematicState k2 = (CoupledKinematicState) o;
        return Math.abs(k2.distance - distance) < 1e-6 &&
                Math.abs(k2.velocity - velocity) < 1e-6 &&
                Math.abs(k2.acceleration - acceleration) < 1e-6;
    }
}
