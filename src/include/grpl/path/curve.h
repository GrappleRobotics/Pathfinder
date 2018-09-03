#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace path {

  template <size_t DIM>
  class curve {
   public:
    using vector_t = Eigen::Matrix<double, DIM, 1>;

    static const size_t DIMENSIONS = DIM;

    virtual vector_t position(double s) const = 0;
    virtual vector_t velocity(double s) const = 0;

    virtual vector_t rotation(double t) {
      vector_t deriv = velocity(t);
      return deriv / deriv.norm();  // Normalize to unit vector
    };

    virtual double curvature(double s) const = 0;
    virtual double curvature_prime(double s) const = 0;

    virtual double length() const = 0;
  };

}  // namespace curve
}  // namespace grpl