#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace spline {

  class spline {
   public:
    using vector_t = typename Eigen::Matrix<double, 2, 1>;

    virtual vector_t calculate(double t) = 0;

    virtual vector_t calculate_derivative(double t) = 0;

    double angle(double t) {
      vector_t deriv = calculate_derivative(t);
      return atan2(deriv[1], deriv[0]);
    }

    virtual double curvature(double t) = 0;
  };

}  // namespace spline
}  // namespace grpl