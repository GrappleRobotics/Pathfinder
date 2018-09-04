#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace profile {

  template <size_t ORD>
  class profile {
   public:
    using kinematics_t = Eigen::Matrix<double, 1, ORD>;
    using limits_t = Eigen::Matrix<double, 2, ORD>;

    static const size_t ORDER = ORD;

    struct segment_t {
      double       time;
      kinematics_t kinematics;
    };

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

    virtual segment_t calculate(segment_t &last, double time) const = 0;

   protected:
    double       _goal, _timeslice = 0.001;
    limits_t _limits;
  };

}  // namespace profile
}  // namespace grpl