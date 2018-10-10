#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  namespace profile {
    const size_t POSITION     = 0;
    const size_t VELOCITY     = 1;
    const size_t ACCELERATION = 2;
    const size_t JERK = 3;

    const size_t profile_segment_order = 3; // Pos, Vel, Acc
    const size_t profile_limits_order  = 4; // <>, Vel, Acc, Jerk

    struct segment {
      using kinematics_t = Eigen::Matrix<double, 1, profile_segment_order>;

      double       time       = 0;
      kinematics_t kinematics = kinematics_t::Zero();
    };

    class profile {
     public:
      using segment_t = segment;
      using limits_t  = Eigen::Matrix<double, 2, profile_limits_order>;

      virtual ~profile() {}

      virtual const size_t limited_term() const = 0;

      void   set_goal(double sp) { _goal = sp; }
      double get_goal() const { return _goal; }

      // TODO: Abstract timeslice?
      void   set_timeslice(double timeslice) { _timeslice = timeslice; }
      double get_timeslice() const { return _timeslice; }

      void apply_limit(int derivative, double min, double max) {
        _limits(0, derivative) = min;
        _limits(1, derivative) = max;
      }

      limits_t get_limits() { return _limits; }

      virtual segment_t calculate(segment_t &last, double time) = 0;

     protected:
      double   _goal, _timeslice = 0.001;
      limits_t _limits = limits_t::Zero();
    };

  }  // namespace profile
}  // namespace pf
}  // namespace grpl