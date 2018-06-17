#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace path {

  template <size_t DIM>
  class path {
   public:
    using vector_t = typename Eigen::Matrix<double, DIM, 1>;

    static const size_t DIMENSIONS = DIM;

    virtual vector_t calculate(double t) = 0;

    virtual vector_t calculate_slope(double t) = 0;

    virtual double calculate_curvature(double t) = 0;
  };

}  // namespace path
}  // namespace grpl