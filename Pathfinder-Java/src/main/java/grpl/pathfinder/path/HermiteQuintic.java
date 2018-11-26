package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

public class HermiteQuintic extends AbstractSpline2d {

    HermiteQuintic(Waypoint start, Waypoint end) {
        super(allocate(start.toNative(), end.toNative()));
    }

    public static class Waypoint {
        public Vec2 position;
        public Vec2 tangent;
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
