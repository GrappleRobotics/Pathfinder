package grpl.pathfinder.coupled;

import grpl.pathfinder.Vec2;

public class CoupledConfiguration {

    private final Vec2 xy;
    private final double heading;

    public CoupledConfiguration() {
        this.xy = Vec2.zero();
        this.heading = 0;
    }

    public CoupledConfiguration(Vec2 xy, double heading) {
        this.xy = xy;
        this.heading = heading;
    }

    public CoupledConfiguration(double x, double y, double heading) {
        this(Vec2.cartesian(x, y), heading);
    }

    public CoupledConfiguration(double[] arr) {
        this(arr[0], arr[1], arr[2]);
    }

    public Vec2 position() {
        return xy;
    }

    public double heading() {
        return heading;
    }

    public double[] toArray() {
        return new double[] { xy.x(), xy.y(), heading };
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof CoupledConfiguration))
            return false;
        CoupledConfiguration k2 = (CoupledConfiguration) o;
        return k2.position().equals(position()) &&
                Math.abs(k2.heading() - heading()) < 1e-6;
    }
}
