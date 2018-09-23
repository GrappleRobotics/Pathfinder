#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  namespace profile {
    const size_t POSITION     = 0;
    const size_t VELOCITY     = 1;
    const size_t ACCELERATION = 2;

    // Profile Base is introduced as a template-less version of profile, for the pure reason
    // that all non-template-dependent methods should go in here. When implementing language
    // bridges (such as JNI), this reduces code reuse since new code has to be written for every
    // type, as compile time information isn't available with another runtime.
    class profile_base {
     public:
      virtual ~profile_base() {}

      void   set_goal(double sp) { _goal = sp; }
      double get_goal() const { return _goal; }

      // TODO: Abstract timeslice?
      void   set_timeslice(double timeslice) { _timeslice = timeslice; }
      double get_timeslice() const { return _timeslice; }

      virtual void apply_limit(int derivative, double min, double max) = 0;

     protected:
      double _goal, _timeslice = 0.001;
    };

    template <size_t LIM_TERM>
    class profile : public profile_base {
     public:
      using kinematics_t = Eigen::Matrix<double, 1, LIM_TERM + 1>;
      using limits_t     = Eigen::Matrix<double, 2, LIM_TERM + 1>;

      static const size_t ORDER   = LIM_TERM + 1;
      static const size_t LIMITED = LIM_TERM;

      struct segment_t {
        using profile_t = profile;
        double       time;
        kinematics_t kinematics;
      };

      void apply_limit(int derivative, double min, double max) override {
        _limits(0, derivative) = min;
        _limits(1, derivative) = max;
      }

      limits_t get_limits() { return _limits; }

      virtual segment_t calculate(segment_t &last, double time) const = 0;

     protected:
      limits_t _limits;
    };
  }  // namespace profile
}  // namespace pf
}  // namespace grpl