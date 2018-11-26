package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;
import grpl.pathfinder.util.NativeResource;

public class AbstractSpline2d extends NativeResource implements Spline2d {

    AbstractSpline2d(long handle) {
        super(handle);
    }

    @Override
    public Vec2 position(double t) {
        return Vec2.cartesian(position(nativeHandle(), t));
    }

    @Override
    public Vec2 derivative(double t) {
        return Vec2.cartesian(derivative(nativeHandle(), t));
    }

    @Override
    public Vec2 rotation(double t) {
        return Vec2.cartesian(rotation(nativeHandle(), t));
    }

    @Override
    public double curvature(double t) {
        return curvature(nativeHandle(), t);
    }

    @Override
    public void close() {
        free(nativeHandle());
        zeroHandle();
    }

    /* JNI */
    private static native double[] position(long h, double t);
    private static native double[] derivative(long h, double t);
    private static native double[] rotation(long h, double t);
    private static native double curvature(long h, double t);

    private static native void free(long h);
}
