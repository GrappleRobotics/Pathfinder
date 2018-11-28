package grpl.pathfinder.path;

import grpl.pathfinder.Vec2;

/**
 * A position curve parameterized to arc length 's'.
 * <p>
 * A curve parameterized to its arc length, allowing extremely quick
 * calculation of the length of the curve, primarily useful when using
 * curves to generate a trajectory, as the parameter is a real-world unit
 * relating directly to the state (distance travelled).
 */
public interface Curve2d {

    /**
     * Calculate the position of a point on the curve, at any arc length 's'.
     *
     * @param s The distance along the arc.
     * @return The position of the curve at arc length 's', in m.
     */
    Vec2 position(double s);

    /**
     * Calculate the derivative of a point on the curve, at any arc length 's'.
     *
     * @param s The distance along the arc.
     * @return The derivative of the curve at arc length 's', unitless.
     */
    Vec2 derivative(double s);

    /**
     * Calculate the rotation of a point on the curve, at any arc length 's'.
     * This is the unit vector of {@link #derivative(double)}.
     *
     * @param s The distance along the arc.
     * @return The rotation of the curve at arc length 's', unitless.
     */
    Vec2 rotation(double s);

    /**
     * Calculate the curvature of the curve at any arc length 's'.
     *
     * @param s The distance along the arc
     * @return The curvature of the arc at arc length 's', in m^-1.
     */
    double curvature(double s);

    /**
     * Calculate the derivative of curvature of the curve at any arc length 's'.
     *
     * @param s The distance along the arc
     * @return The derviative of curvature of the arc at arc length 's' (dk/ds),
     * in m^-2.
     */
    double dcurvature(double s);

    /**
     * Calculate the total length of the arc.
     * <p>
     * The arc length is what defines the range of parameter 's' used throughout this class.
     *
     * @return The total length of the curve, in metres.
     */
    double length();

}
