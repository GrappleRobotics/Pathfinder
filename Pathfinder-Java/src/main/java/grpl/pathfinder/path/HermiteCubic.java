package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

/**
 * Implementation of a cubic (order 3) hermite spline.
 *
 * A Cubic Hermite Spline is a spline of order 3, as defined here
 * https://en.wikipedia.org/wiki/Cubic_Hermite_spline.
 *
 * The cubic spline is defined in regards to its waypoints, which are defined in terms of position and
 * tangent.
 */
public class HermiteCubic extends AbstractSpline2d {

    /**
     * Construct a cubic hermite spline, given a start and end point.
     *
     * @param start The waypoint of the start of the spline
     * @param end   The waypoint of the end of the spline.
     */
    HermiteCubic(Waypoint start, Waypoint end) {
        super(allocate(start.toNative(), end.toNative()));
    }

    /**
     * Waypoint for a cubic hermite spline.
     */
    public static class Waypoint {

        /**
         * The 2D position of the waypoint, in metres. Ordered x, y.
         */
        public Vec2 position;

        /**
         * The 2D tangent to the waypoint, in metres. Ordered x, y.
         */
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
