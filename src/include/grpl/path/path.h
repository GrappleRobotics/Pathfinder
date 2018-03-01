#pragma once

#include "grpl/vec.h"
#include "grpl/units.h"

namespace grpl {
namespace path {
  
  template <typename VECTYPE, typename UNIT>
  class path {
  public:
    virtual VECTYPE calculate(double t) const = 0;
    virtual VECTYPE calculate_slope(double t) const = 0;
    virtual UNIT calculate_arc_length() const = 0;
  };

}
}