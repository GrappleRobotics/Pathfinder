#pragma once

#include <blaze/Math.h>

namespace grpl {
namespace path {
  
  template <size_t DIM>
  class path {
  public:
    using vector_t = blaze::StaticVector<double, DIM, blaze::columnVector>;

    static const size_t DIMENSIONS = DIM;

    virtual vector_t calculate(double t) const = 0;
    virtual vector_t calculate_slope(double t) const = 0;
    virtual double calculate_curvature(double t) const = 0;
    virtual double get_arc_length() = 0;
    virtual void reset_arc_length() = 0;
  };

}
}