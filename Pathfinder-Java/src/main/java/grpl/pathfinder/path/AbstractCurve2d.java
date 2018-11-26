package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;
import grpl.pathfinder.util.NativeResource;

public class AbstractCurve2d extends NativeResource implements Curve2d {
    public AbstractCurve2d(long handle) {
        super(handle);
    }

    @Override
    public Vec2 position(double s)  {
        return Vec2.cartesian(position(nativeHandle(), s));
    }

    @Override
    public Vec2 derivative(double s)  {
        return Vec2.cartesian(derivative(nativeHandle(), s));
    }

    @Override
    public Vec2 rotation(double s)  {
        return Vec2.cartesian(rotation(nativeHandle(), s));
    }

    @Override
    public double curvature(double s)  {
        return curvature(nativeHandle(), s);
    }

    @Override
    public double dcurvature(double s)   {
        return dcurvature(nativeHandle(), s);
    }

    @Override
    public double length() {
        return length(nativeHandle());
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
    private static native double dcurvature(long h, double t);
    private static native double length(long h);

    private static native void free(long h);
}
