package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

public class Arc2d extends AbstractCurve2d {

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
