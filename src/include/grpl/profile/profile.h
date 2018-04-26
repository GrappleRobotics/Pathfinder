#pragma once

#include <blaze/Math.h>
#include "grpl/units.h"

namespace grpl {
namespace profile {

  template <size_t ORD>
  class profile {
  public:
    using vec_t = blaze::StaticVector<double, ORD, blaze::columnVector>;
    static const size_t ORDER = ORD;

    struct segment_t {
      vec_t vect;
      double time;
    };

    void set_goal(double sp) { _goal = sp; }
    double get_goal() const { return _goal; }

    void set_timeslice(double timeslice) { _timeslice = timeslice; }
    double get_timeslice() const { return _timeslice; }

    // TODO: Set limits mask ?
    void apply_limit(int derivative_idx, double maximum) { _limits[derivative_idx] = maximum; }
    vec_t &get_limits() { return _limits; }
    void set_limits(vec_t &other) { _limits = other; }

    virtual segment_t calculate(segment_t &last, double time) const = 0;

  protected:
    double _goal, _timeslice = 0.001;
    vec_t _limits;
  };
  
} // namespace grpl
} // namespace profile