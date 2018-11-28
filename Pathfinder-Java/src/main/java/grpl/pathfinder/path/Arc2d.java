package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

/**
 * A 2-Dimensional Circular Arc
 *
 * Implementation of a 2-Dimensional circular arc, parameterized to arc length 's'.
 */
public class Arc2d extends AbstractCurve2d {

    /**
     * Create a circular arc from a set of 3 points (start, any, and end).
     *
     * @param start The start point of the curve, in x,y metres.
     * @param mid   Any point along the curve sitting between start and end,
     *              in x,y metres.
     * @param end   The end point of the curve, in x,y metres.
     */
    Arc2d(Vec2 start, Vec2 mid, Vec2 end) {
        super(allocate(start.xy(), mid.xy(), end.xy()));
    }

    private Arc2d(long handle) {
        super(handle);
    }

    protected static Arc2d fromNativeHandle(long handle) {
        return new Arc2d(handle);
    }

    private static native long allocate(double[] start, double[] mid, double[] end);

}
