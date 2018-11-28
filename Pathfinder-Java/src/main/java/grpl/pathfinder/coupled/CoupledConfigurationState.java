package grpl.pathfinder.coupled;

import grpl.pathfinder.Vec2;

/**
 * Drivetrain configuration state, describing the configuration of the chassis.
 * <p>
 * Formally, the 'configuration' of a body is all variables required to fully define the position
 * of the body and its manipulators. For a coupled drivetrain, this is simply its position in the
 * the work space, as well as its heading. In the case of the coupled drivetrain, this may be referred
 * to as the "transform" of the chassis.
 * </p>
 */
public class CoupledConfigurationState {

    /**
     * The position of the centre of the drivetrain, in metres.
     */
    public Vec2 position;

    /**
     * The heading of the drivetrain, in radians.
     */
    public double heading;

    public CoupledConfigurationState() {
        this.position = Vec2.zero();
        this.heading = 0;
    }

    public CoupledConfigurationState(Vec2 position, double heading) {
        this.position = position;
        this.heading = heading;
    }

    public CoupledConfigurationState(double x, double y, double heading) {
        this(Vec2.cartesian(x, y), heading);
    }

    public CoupledConfigurationState(double[] arr) {
        this(arr[0], arr[1], arr[2]);
    }

    public double[] toArray() {
        return new double[]{position.x(), position.y(), heading};
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof CoupledConfigurationState))
            return false;
        CoupledConfigurationState k2 = (CoupledConfigurationState) o;
        return k2.position.equals(position) &&
                Math.abs(k2.heading - heading) < 1e-6;
    }
}
