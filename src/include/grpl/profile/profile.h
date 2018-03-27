#pragma once

#include <blaze/Math.h>
#include "grpl/units.h"

namespace grpl {
namespace profile {

  template <size_t ORD>
  class profile1 {
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

    void apply_limit(int derivative_idx, double maximum) { _limits[derivative_idx] = maximum; }
    vec_t &get_limits() { return _limits; }
    void set_limits(vec_t &other) { _limits = other; }

    virtual segment_t calculate(segment_t &last, double time) const = 0;

  protected:
    double _goal, _timeslice = 0.001;
    vec_t _limits;
  };

  template <typename UNIT_DIST, typename UNIT_TIME, typename UNIT_VEL, typename UNIT_ACC>
  class profile {
  public:
    struct segment {
      UNIT_TIME time;
      UNIT_DIST dist;
      UNIT_VEL vel;
      UNIT_ACC acc;
    };

    void goal(UNIT_DIST sp) { _goal = sp; }
    UNIT_DIST goal() const { return _goal; }

    void timeslice(UNIT_TIME timeslice_dt) { _timeslice = timeslice_dt; }
    UNIT_TIME timeslice() const { return _timeslice; }

    void max_velocity(UNIT_VEL vel_max) { _vel_max = vel_max; }
    UNIT_VEL max_velocity() { return _vel_max; }

    void max_acceleration(UNIT_ACC acc_max) { _acc_max = acc_max; }
    UNIT_ACC max_acceleration() { return _acc_max; }

    virtual void calculate_single(segment *out, segment *last, UNIT_TIME time) const = 0;

    void calculate(segment *out, segment *last, UNIT_TIME time) const {
      segment tmp = { last->time, last->dist, last->vel, last->acc };
      UNIT_TIME dt = time - last->time;
      double slice_count_d = static_cast<double>(dt / _timeslice);
      int slice_count = static_cast<int>(slice_count_d);
      if (slice_count_d - slice_count > 0.9) slice_count++;

      if (slice_count < 1) {
        calculate_single(&tmp, &tmp, time);
      } else {
        for (int i = 1; i <= slice_count; i++) {
          UNIT_TIME time = tmp.time + _timeslice;
          calculate_single(&tmp, &tmp, time);
        }
      }

      out->time = tmp.time;
      out->dist = tmp.dist;
      out->vel = tmp.vel;
      out->acc = tmp.acc;
    }
  protected:
    UNIT_DIST _goal;
    UNIT_VEL _vel_max;
    UNIT_ACC _acc_max;
    UNIT_TIME _timeslice = 0.001;
  };
} // namespace grpl
} // namespace profile