#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace profile {

  template <size_t ORD>
  class profile {
   public:
    using kinematics_1d_t = Eigen::Matrix<double, 1, ORD>;

    static const size_t ORDER = ORD;

    struct segment_t {
      kinematics_1d_t k;
      double          time;
    };

    void   set_goal(double sp) { _goal = sp; }
    double get_goal() const { return _goal; }

    void   set_timeslice(double timeslice) { _timeslice = timeslice; }
    double get_timeslice() const { return _timeslice; }

    // TODO: Set limits mask ?
    void apply_limit(int derivative_idx, double maximum) {
      _limits[derivative_idx] = maximum;
    }

    kinematics_1d_t &get_limits() { return _limits; }
    void             set_limits(kinematics_1d_t &other) { _limits = other; }

    virtual segment_t calculate(segment_t &last, double time) const = 0;

   protected:
    double          _goal, _timeslice = 0.001, _timeslice_enabled = 0;
    kinematics_1d_t _limits;
  };

}  // namespace profile
}  // namespace grpl