#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace path {

  template <size_t DIM>
  class spline {
   public:
    using vector_t = Eigen::Matrix<double, DIM, 1>;

    static const size_t DIMENSIONS = DIM;

    virtual vector_t position(double t) = 0;
    virtual vector_t velocity(double t) = 0;

    virtual vector_t rotation(double t) {
      vector_t deriv = velocity(t);
      return deriv / deriv.norm();  // Normalize to unit vectors
    };

    virtual double curvature(double t) = 0;
  };

}  // namespace path
}  // namespace grpl