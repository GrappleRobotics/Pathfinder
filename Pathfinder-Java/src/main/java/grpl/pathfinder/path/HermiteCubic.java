package grpl.pathfinder.path;

public class HermiteCubic extends AbstractSpline2d {

    HermiteCubic(HermiteWaypoint start, HermiteWaypoint end) {
        super(allocate(start.toNative(), end.toNative()));
    }

    private static native long allocate(double[] start, double[] end);
}
