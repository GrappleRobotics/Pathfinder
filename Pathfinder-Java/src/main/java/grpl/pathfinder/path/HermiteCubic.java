package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

public class HermiteCubic extends AbstractSpline2d {

    HermiteCubic(Waypoint start, Waypoint end) {
        super(allocate(start.toNative(), end.toNative()));
    }

    public static class Waypoint {
        public Vec2 position;
        public Vec2 tangent;

        public Waypoint(Vec2 position, Vec2 tangent) {
            this.position = position;
            this.tangent = tangent;
        }

        protected double[] toNative() {
            return new double[] {
                position.x(), position.y(),
                tangent.x(), tangent.y()
            };
        }
    }

    private static native long allocate(double[] start, double[] end);
}
