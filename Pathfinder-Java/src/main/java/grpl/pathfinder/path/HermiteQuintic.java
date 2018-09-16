package grpl.pathfinder.path;

public class HermiteQuintic extends AbstractSpline2d {

    HermiteQuintic(HermiteWaypoint start, HermiteWaypoint end) {
        super(allocate(start.toNative(), end.toNative()));
    }

    private static native long allocate(double[] start, double[] end);

}
