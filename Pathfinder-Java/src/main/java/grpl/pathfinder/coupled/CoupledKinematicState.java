package grpl.pathfinder.coupled;

public class CoupledKinematicState
{

    private final double distance, velocity, acceleration;

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

    public double s() {
        return distance;
    }

    public double v() {
        return velocity;
    }

    public double a() {
        return acceleration;
    }

    public double[] toArray() {
        return new double[] { distance, velocity, acceleration };
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof CoupledKinematicState))
            return false;
        CoupledKinematicState k2 = (CoupledKinematicState) o;
        return Math.abs(k2.s() - s()) < 1e-6 &&
                Math.abs(k2.v() - v()) < 1e-6 &&
                Math.abs(k2.a() - a()) < 1e-6;
    }
}
