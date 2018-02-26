#pragma once

#include "grpl/units.h"

namespace grpl {
namespace profile {

  namespace intern {
    class profile_internal {
    public:
      #pragma pack(push, 8)
      struct segment_raw {
        double time, dist, vel, acc;
      };
      #pragma pack(pop)

      void goal_raw(double sp)  { _goal = sp; }
      double goal_raw() const { return _goal; }

      void timeslice_raw(double timeslice_dt) { _timeslice_dt = timeslice_dt; }
      double timeslice_raw() const { return _timeslice_dt; }

      virtual void calculate_single(segment_raw *out, segment_raw *last, double time) const = 0;
      void calculate(segment_raw *out, segment_raw *last, double time) const;

    protected:
      double _goal, _timeslice_dt = 0.001;
    };
  }

  template <typename IMPL_CLASS, typename UNIT_DIST, typename UNIT_TIME, typename UNIT_VEL, typename UNIT_ACC>
  class profile : public IMPL_CLASS {
  public:
    struct segment {
      UNIT_TIME time;
      UNIT_DIST dist;
      UNIT_VEL vel;
      UNIT_ACC acc;
    };

    void goal(UNIT_DIST sp) { goal_raw(static_cast<double>(sp)); }
    UNIT_DIST goal() const { return static_cast<UNIT_DIST>(goal_raw()); }

    void timeslice(UNIT_TIME timeslice_dt)  { timeslice_raw(static_cast<double>(timeslice_dt)); }
    UNIT_TIME timeslice() const { return static_cast<UNIT_TIME>(timelice_raw()); }

    void calculate(segment *out, segment *last, UNIT_TIME time) const {
      intern::profile_internal::segment_raw tmp = { 
        static_cast<double>(last->time),
        static_cast<double>(last->dist),
        static_cast<double>(last->vel),
        static_cast<double>(last->acc)
      };

      IMPL_CLASS::calculate(&tmp, &tmp, static_cast<double>(time));

      out->time = static_cast<UNIT_TIME>(tmp.time);
      out->dist = static_cast<UNIT_DIST>(tmp.dist);
      out->vel = static_cast<UNIT_VEL>(tmp.vel);
      out->acc = static_cast<UNIT_ACC>(tmp.acc);
    }
  };
} // namespace grpl
} // namespace profile