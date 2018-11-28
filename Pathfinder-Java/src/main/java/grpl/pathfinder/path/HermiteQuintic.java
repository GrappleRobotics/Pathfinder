package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

/**
 * Implementation of a quintic (order 5) hermite spline.
 *
 * The quintic spline is defined in regards to its waypoints, which are defined in terms of position,
 * tangent and the derivative of the tangent.
 */
public class HermiteQuintic extends AbstractSpline2d {

    /**
     * Construct a quintic hermite spline, given a start and end point.
     *
     * @param start The waypoint of the start of the spline
     * @param end   The waypoint of the end of the spline.
     */
    HermiteQuintic(Waypoint start, Waypoint end) {
        super(allocate(start.toNative(), end.toNative()));
    }

    /**
     * Waypoint for a quintic hermite spline.
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

        /**
         * 2D derivative of the tangent to the waypoint, in metres. Ordered x, y.
         */
        public Vec2 dtangent;

        public Waypoint(Vec2 position, Vec2 tangent, Vec2 dtangent) {
            this.position = position;
            this.tangent = tangent;
            this.dtangent = dtangent;
        }

        protected double[] toNative() {
            return new double[] {
                position.x(), position.y(),
                tangent.x(), tangent.y(),
                dtangent.x(), dtangent.y()
            };
        }
    }

    private static native long allocate(double[] start, double[] end);

}
