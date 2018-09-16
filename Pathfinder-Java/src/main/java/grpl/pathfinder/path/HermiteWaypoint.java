package grpl.pathfinder.path;

public class HermiteWaypoint {
    public Vec2 position;
    public Vec2 tangent;
    public Vec2 tangent_slope;

    public HermiteWaypoint(Vec2 position, Vec2 tangent) {
        this(position, tangent, Vec2.zero());
    }

    public HermiteWaypoint(Vec2 position, Vec2 tangent, Vec2 tangent_slope) {
        this.position = position;
        this.tangent = tangent;
        this.tangent_slope = tangent_slope;
    }

    protected double[] toNative() {
        return new double[] {
            position.x(), position.y(),
            tangent.x(), tangent.y(),
            tangent_slope.x(), tangent_slope.y()
        };
    }
}
