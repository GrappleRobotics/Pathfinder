#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace spline {

  template <size_t DIM>
  class spline {
   public:
    using vector_t = typename Eigen::Matrix<double, DIM, 1>;

    static const size_t DIMENSIONS = DIM;

    virtual vector_t calculate(double t) = 0;

    virtual vector_t calculate_derivative(double t) = 0;

    virtual double curvature(double t) = 0;
  };

}  // namespace spline
}  // namespace grpl