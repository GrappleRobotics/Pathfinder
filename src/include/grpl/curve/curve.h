#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace curve {

  template <size_t DIM>
  class curve {
   public:
    using vector_t = typename Eigen::Matrix<double, DIM, 1>;

    static const size_t DIMENSIONS = DIM;

    virtual vector_t calculate(const double s) const = 0;

    virtual vector_t calculate_derivative(const double s) const = 0;

    virtual double curvature(const double s) const = 0;

    virtual double angle(const double s) const = 0;

    virtual double length() const = 0;
  };

}  // namespace curve
}  // namespace grpl