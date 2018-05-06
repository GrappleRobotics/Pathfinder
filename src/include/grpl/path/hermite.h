#pragma once

#include "grpl/path/path.h"

namespace grpl {
namespace path {

  template <size_t DIM, size_t ORDER=3>
  class hermite : public path<DIM> {
  public:
    using vector_t  = typename path<DIM>::vector_t;
    using basis_t   = typename Eigen::Matrix<double, ORDER+1, 1>;

    static_assert(ORDER == 3 || ORDER == 5, "ORDER must be either 3 (cubic) or 5 (quintic)");

    struct waypoint {
      vector_t point, tangent, tangent_slope;
    };

    hermite() {}

    hermite(size_t arclength_samples) {
      set_samples(arclength_samples);
    }

    hermite(waypoint &wp0, waypoint &wp1, size_t arclength_samples) {
      set_samples(arclength_samples);
      set_waypoints(wp0, wp1);
    }

    void set_samples(size_t arclength_samples) {
      _al_samples = arclength_samples;
      _al_calculated = false;
    }

    void set_waypoints(waypoint &wp0, waypoint &wp1) {
      _wp0 = wp0;
      _wp1 = wp1;
      if (ORDER == 3) {
        M.col(0) = _wp0.point;
        M.col(1) = _wp0.tangent;
        M.col(2) = _wp1.point;
        M.col(3) = _wp1.tangent;
      } else if (ORDER == 5) {
        M.col(0) = _wp0.point;
        M.col(1) = _wp0.tangent;
        M.col(2) = _wp0.tangent_slope;
        M.col(3) = _wp1.point;
        M.col(4) = _wp1.tangent;
        M.col(5) = _wp1.tangent_slope;
      }
    }

    inline basis_t basis(double t) const {
      basis_t b;
      if (ORDER == 3) {
        b <<
          (2 * t*t*t - 3 * t*t + 1),
          (t*t*t - 2 * t*t + t),
          (-2 * t*t*t + 3 * t*t),
          (t*t*t - t*t);
      } else {
        b <<
          (1 - 10*t*t*t + 15*t*t*t*t - 6*t*t*t*t*t),
          (t - 6*t*t*t + 8*t*t*t*t - 3*t*t*t*t*t),
          (0.5*t*t - 1.5*t*t*t + 1.5*t*t*t*t - 0.5*t*t*t*t*t),
          (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t),
          (7*t*t*t*t - 4*t*t*t - 3*t*t*t*t*t),
          (0.5*t*t*t - t*t*t*t + 0.5*t*t*t*t*t);
      }
      return b;
    }

    inline basis_t basis_1st(double t) const {
      basis_t b;
      if (ORDER == 3) {
        b << 
          (6 * t*t - 6 * t),
          (3*t*t - 4 * t + 1),
          (-6 * t*t + 6 * t),
          (3 * t*t - 2 * t);
      } else {
        b << 
          (-30*t*t + 60*t*t*t - 30*t*t*t*t),
          (1 - 18*t*t + 32*t*t*t - 15*t*t*t*t),
          (t - 4.5*t*t + 6*t*t*t - 2.5*t*t*t*t),
          (30*t*t - 60*t*t*t + 30*t*t*t*t),
          (28*t*t*t - 12*t*t - 15*t*t*t*t),
          (1.5*t*t - 4*t*t*t + 2.5*t*t*t*t);
      }
      return b;
    }

    inline basis_t basis_2nd(double t) const {
      basis_t b;
      if (ORDER == 3) {
        b <<
          (12 * t - 6),
          (6*t - 4),
          (-12 * t + 6),
          (6 * t - 2);
      } else {
        b << 
          (-60*t + 180*t*t - 120*t*t*t),
          (-36*t + 96*t*t - 60*t*t*t),
          (1 - 9*t + 18*t*t - 10*t*t*t),
          (60*t - 180*t*t + 120*t*t*t),
          (84*t*t - 24*t - 60*t*t*t),
          (3*t - 12*t*t + 10*t*t*t);
      }
      return b;
    }

    vector_t calculate(double t) override {
      return M * basis(t);
    }

    vector_t calculate_slope(double t) override {
      return M * basis_1st(t);
    }

    vector_t calculate_slope_second(double t) {
      return M * basis_2nd(t);
    }

    double calculate_curvature(double t) override {
      vector_t h_p = calculate_slope(t), h_pp = calculate_slope_second(t);

      return (
        sqrt( h_p.squaredNorm()*h_pp.squaredNorm() - pow(h_p.dot(h_pp), 2) )
        / pow(h_p.norm(), 3)
      );
    }

    double get_arc_length() override {
      if (!_al_calculated) {
        double t = 0, dt = (1.0 / _al_samples);

        double last_integrand = 0, arc_length = 0;
        for (t = 0; t <= 1; t += dt) {
          vector_t R = M * basis_1st(t);

          // Arc length calculation
          double integrand = sqrt(1 + R.squaredNorm()) * dt;
          arc_length += (integrand + last_integrand) / 2;
          last_integrand = integrand;
        }

        _al_last = arc_length;
        _al_calculated = true;
      }
      return _al_last;
    }

    void reset_arc_length() override {
      _al_calculated = false;
    }

  protected:
    waypoint _wp0, _wp1;
    Eigen::Matrix<double, DIM, ORDER+1> M;
    size_t _al_samples = 10000;
    bool _al_calculated = false;
    double _al_last = 0;
  };

  namespace hermite_factory {
    template <typename hermite_t, typename waypoint_t>
    size_t generate(waypoint_t *wps, size_t waypoint_count, hermite_t *hermite_out, size_t out_size) {
      size_t num_hermite = waypoint_count - 1;
      if (num_hermite < out_size) return 0;

      size_t hermite_id = 0;
      for (size_t wpid = 1; wpid < waypoint_count; wpid++) {
        waypoint_t wp0 = wps[wpid-1], wp1 = wps[wpid];
        waypoint_t wp1_new = { wp1.point, -wp1.tangent, -wp1.tangent_slope };

        hermite_out[hermite_id++].set_waypoints(wp0, wp1);
      }
      return num_hermite;
    }
  }

}
}