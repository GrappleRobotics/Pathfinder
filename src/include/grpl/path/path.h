#pragma once

#include <blaze/Math.h>

namespace grpl {
namespace path {
  
  template <typename UNIT, size_t DIM>
  class path {
  public:
    virtual blaze::StaticVector<UNIT, DIM> calculate(double t) const = 0;
    virtual blaze::StaticVector<UNIT, DIM> calculate_slope(double t) const = 0;
    virtual UNIT calculate_arc_length() const = 0;
  };

}
}