#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  /**
   * Path, spline and curve members.
   * 
   * The grpl::pf::path namespace contains all classes to deal with pathing, be 
   * it splines, curves or the utilities that accompany them. The classes in this 
   * namespace deal only with position paths, and do not include parameterizations
   * to time (i.e. trajectories).
   */
  namespace path {
    /**
     * A position curve parameterized to spline parameter 't'. 
     * 
     * A spline is simply a position curve parameterized to spline parameter 't' 
     * (note that this is distinct to time), which lays in the range of 0 to 1, 
     * representing the start and end of the spline respectively.
     * 
     * @param DIM The number of dimensions of the spline, usually 2.
     */
    template <size_t DIM>
    class spline {
     public:
      virtual ~spline() {}

      using vector_t = Eigen::Matrix<double, DIM, 1>;

      //! The number of dimensions of the spline
      static const size_t DIMENSIONS = DIM;

      /**
       * Calculate the position of a point on the spline, at any spline parameter
       * value 't'
       * 
       * @param t The spline parameter, where 0 is the start and 1 is the end of the
       *          spline.
       * @return  The position at spline parameter 't', in m.
       */
      virtual vector_t position(double t) = 0;

      /**
       * Calculate the derivative of a point on the spline, at any spline parameter
       * value 't' (the derivative of @ref position(double))
       * 
       * @param t The spline parameter, where 0 is the start and 1 is the end of the
       *          spline.
       * @return  The derivative at spline parameter 't', in m/t 
       */
      virtual vector_t derivative(double t) = 0;

      /**
       * Calculate the rotation of a point on the spline, at any spline parameter 
       * value 't'. This is the unit vector of @ref derivative(double)
       * 
       * @param t The spline parameter, where 0 is the start and 1 is the end of the
       *          spline.
       * @return  The rotation (unit derivative) at spline parameter 't', unitless.
       */
      virtual vector_t rotation(double t) {
        vector_t deriv = derivative(t);
        return deriv / deriv.norm();  // Normalize to unit vectors
      };

      /**
       * Calculate the curvature of the spline at any spline parameter value 't'.
       * 
       * @param t The spline parameter, where 0 is the start and 1 is the end of the
       *          spline.
       * @return  The curvature at spline parameter 't', in m^-1.
       */
      virtual double curvature(double t) = 0;
    };
  }  // namespace path
}  // namespace pf
}  // namespace grpl