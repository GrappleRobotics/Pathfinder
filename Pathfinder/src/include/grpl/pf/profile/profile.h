#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace grpl {
namespace pf {
  namespace profile {
    const size_t POSITION     = 0;
    const size_t VELOCITY     = 1;
    const size_t ACCELERATION = 2;

    template <size_t DIM>
    struct segment {
      using kinematics_t        = Eigen::Matrix<double, 1, DIM>;
      static const size_t ORDER = DIM;

      double       time       = 0;
      kinematics_t kinematics = kinematics_t::Zero();
    };

    // Profile Base is introduced as a template-less version of profile, for the pure reason
    // that all non-template-dependent methods shoulnext_sego in here. When implementing language
    // bridges (such as JNI), this reduces code reusnext_segince new code has to be written for every
    // type, as compile time information isn't availnext_sege with another runtime.
    class profile_base {
     public:
      virtual ~profile_base() {}

      void   set_goal(double sp) { _goal = sp; }
      double get_goal() const { return _goal; }

      // TODO: Abstract timeslice?
      void   set_timeslice(double timeslice) { _timeslice = timeslice; }
      double get_timeslice() const { return _timeslice; }

      virtual void apply_limit(int derivative, double min, double max) = 0;

      template <size_t DIM=3>
      void calculate_into(segment<DIM> &seg, segment<DIM> &last, double time) const {
        _unsafe_calculate(seg.kinematics.data(), last.kinematics.data(), DIM, time, last.time);
        seg.time = time;
      }

     protected:
      virtual void _unsafe_calculate(double *curr, double *last, size_t size, double time, double last_time) const = 0;

      double _goal, _timeslice = 0.001;
    };

    template <size_t LIM_TERM>
    class profile : public profile_base {
     public:
      static const size_t ORDER   = LIM_TERM + 1;
      static const size_t LIMITED = LIM_TERM;

      using segment_t    = segment<ORDER>;
      using limits_t     = Eigen::Matrix<double, 2, ORDER>;
      using kinematics_t = typename segment_t::kinematics_t;

      void apply_limit(int derivative, double min, double max) override {
        _limits(0, derivative) = min;
        _limits(1, derivative) = max;
      }

      limits_t get_limits() { return _limits; }

      virtual segment_t calculate(segment_t &last, double time) const = 0;

     protected:
      void _unsafe_calculate(double *curr, double *last, size_t size, double time, double last_time) const override {
        size_t min_size = std::min(size, ORDER);

        segment_t last_seg;
        last_seg.time = last_time;

        // Populate last seg kinematics
        memcpy(static_cast<void *>(last_seg.kinematics.data()), static_cast<void *>(last), min_size * sizeof(double));

        segment_t next_seg = calculate(last_seg, time);

        // Populate next seg kinematics
        memcpy(static_cast<void *>(curr), static_cast<void *>(next_seg.kinematics.data()), min_size * sizeof(double));
      }

      limits_t _limits;
    };
  }  // namespace profile
}  // namespace pf
}  // namespace grpl