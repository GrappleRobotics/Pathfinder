#pragma once

#include <blaze/Math.h>

namespace grpl {
namespace path {
  
  template <typename UNIT, size_t DIM>
  class path {
  public:
    using vector_t = blaze::StaticVector<UNIT, DIM, blaze::columnVector>;

    static const size_t DIMENSIONS = DIM;

    virtual vector_t calculate(double t) const = 0;
    virtual vector_t calculate_slope(double t) const = 0;
    virtual double calculate_curvature(double t) const = 0;
    virtual UNIT calculate_arc_length() const = 0;
  };

}
}