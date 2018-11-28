package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

/**
 * A position curve parameterized to spline parameter 't'.
 * <p>
 * A spline is simply a position curve parameterized to spline parameter 't'
 * (note that this is distinct to time), which lays in the range of 0 to 1,
 * representing the start and end of the spline respectively.
 */
public interface Spline2d {

    /**
     * Calculate the position of a point on the spline, at any spline parameter
     * value 't'
     *
     * @param t The spline parameter, where 0 is the start and 1 is the end of the
     *          spline.
     * @return The position at spline parameter 't', in m.
     */
    Vec2 position(double t);

    /**
     * Calculate the derivative of a point on the spline, at any spline parameter
     * value 't' (the derivative of {@link #position(double)})
     *
     * @param t The spline parameter, where 0 is the start and 1 is the end of the
     *          spline.
     * @return The derivative at spline parameter 't', in m/t
     */
    Vec2 derivative(double t);

    /**
     * Calculate the rotation of a point on the spline, at any spline parameter
     * value 't'. This is the unit vector of {@link #derivative(double)}
     *
     * @param t The spline parameter, where 0 is the start and 1 is the end of the
     *          spline.
     * @return The rotation (unit derivative) at spline parameter 't', unitless.
     */
    Vec2 rotation(double t);

    /**
     * Calculate the curvature of the spline at any spline parameter value 't'.
     *
     * @param t The spline parameter, where 0 is the start and 1 is the end of the
     *          spline.
     * @return The curvature at spline parameter 't', in m^-1.
     */
    double curvature(double t);

}
