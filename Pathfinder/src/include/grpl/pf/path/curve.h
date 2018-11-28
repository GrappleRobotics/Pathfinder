#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  namespace path {
    /**
     * A position curve parameterized to arc length 's'.
     * 
     * A curve parameterized to its arc length, allowing extremely quick 
     * calculation of the length of the curve, primarily useful when using
     * curves to generate a trajectory, as the parameter is a real-world unit
     * relating directly to the state (distance travelled).
     * 
     * @param DIM The number of dimensions of the curve, usually 2.
     */
    template <size_t DIM>
    class curve {
     public:
      using vector_t = Eigen::Matrix<double, DIM, 1>;

      virtual ~curve() {}

      //! The number of dimensions of the curve
      static const size_t DIMENSIONS = DIM;

      /**
       * Calculate the position of a point on the curve, at any arc length 's'.
       * 
       * @param s The distance along the arc.
       * @return  The position of the curve at arc length 's', in m.
       */
      virtual vector_t position(double s) const = 0;

      /**
       * Calculate the derivative of a point on the curve, at any arc length 's'.
       * 
       * @param s The distance along the arc.
       * @return  The derivative of the curve at arc length 's', unitless.
       */
      virtual vector_t derivative(double s) const = 0;

      /**
       * Calculate the rotation of a point on the curve, at any arc length 's'.
       * This is the unit vector of @ref derivative(double) const.
       * 
       * @param s The distance along the arc.
       * @return  The rotation of the curve at arc length 's', unitless.
       */
      virtual vector_t rotation(double s) {
        vector_t deriv = derivative(s);
        return deriv / deriv.norm();  // Normalize to unit vector
      };

      /**
       * Calculate the curvature of the curve at any arc length 's'.
       * 
       * @param s The distance along the arc
       * @return  The curvature of the arc at arc length 's', in m^-1.
       */
      virtual double curvature(double s) const       = 0;

      /**
       * Calculate the derivative of curvature of the curve at any arc length 's'.
       * 
       * @param s The distance along the arc
       * @return  The derviative of curvature of the arc at arc length 's' (dk/ds), 
       *          in m^-2.
       */
      virtual double dcurvature(double s) const = 0;

      /**
       * Calculate the total length of the arc.
       * 
       * The arc length is what defines the range of parameter 's' used throughout this class.
       * 
       * @return The total length of the curve, in metres.
       */
      virtual double length() const = 0;
    };
  }  // namespace path
}  // namespace pf
}  // namespace grpl