package grpl.pathfinder.path;

import grpl.pathfinder.util.INativeResource;
import grpl.pathfinder.util.NativeResource;

import java.util.List;
import java.util.Vector;

/**
 * Arc Parameterizer, for reparameterizing splines to curves.
 * <p>
 * The Arc Parameterizer takes in splines, parameterized to spline parameter 't', and produces
 * arcs parameterized by arc length 's'. The parameterizer uses augmented arcs, which are of a
 * non-constant curvature in order to approximate the spline while still remaining continuous in
 * curvature at the knot points.
 * </p>
 */
public class ArcParameterizer extends NativeResource {

    public ArcParameterizer() {
        super(allocate());
    }

    /**
     * Parameterize a container of splines into augmented 2D arcs.
     * <p>
     * Reparameterizes a set of splines from being with respect to spline parameter 't', to a set of curves
     * that are with respect to arc length 's'.
     *
     * @param splines A list of splines to parameterize
     * @return The parameterized curves generated from the splines
     */
    public List<Arc2d> parameterize(List<? extends Spline2d> splines) {
        long buffer = pack(splines);
        List<Arc2d> list = new Vector<Arc2d>();
        try {
            long[] arcs = parameterize(nativeHandle(), buffer);

            for (long arc : arcs)
                list.add(Arc2d.fromNativeHandle(arc));
        }
        finally {
            releaseBuffer(buffer);
            buffer = 0L;
        }
        return list;
    }

    /**
     * Configure the parameters for the parameterizer, which are used as the criteria for deciding
     * when to produce a new arc.
     *
     * @param maxArcLength      The maximum arc length. Any arcs larger than this will be recursively
     *                          split. Unit is metres.
     * @param maxDeltaCurvature The maximum change in curvature between the start and end of the
     *                          arc. Any arcs with a large change in curvature will be recursively
     *                          split. Unit is m^-1.
     */
    public void configure(double maxArcLength, double maxDeltaCurvature) {
        configure(nativeHandle(), maxArcLength, maxDeltaCurvature);
    }

    @Override
    public void close() {
        free(nativeHandle());
        zeroHandle();
    }

    private static long pack(List<? extends Spline2d> splines) {
        long buffer = acquireBuffer();
        for (Spline2d spline : splines) {
            // If the spline is already native, it's much cheaper to pass it directly than to construct
            // an adapter in the JNI that calls back to java for each sample.
            if (spline instanceof INativeResource) {
                enqueueNative(buffer, ((INativeResource) spline).nativeHandle());
            } else {
                enqueueAdapter(buffer, spline);
            }
        }
        return buffer;
    }

    /* JNI */
    private static native long allocate();

    private static native void free(long h);

    private static native void configure(long handle, double maxArcLength, double maxDeltaCurvature);

    private static native long[] parameterize(long handle, long buffer);

    private static native long acquireBuffer();

    private static native void enqueueNative(long bufferHandle, long splineHandle);

    private static native void enqueueAdapter(long bufferHandle, Spline2d spline);

    private static native void releaseBuffer(long bufferHandle);

}
