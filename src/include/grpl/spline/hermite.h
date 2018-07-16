#pragma once

#include "grpl/spline/spline.h"
#include "grpl/util/constants.h"

namespace grpl {
namespace spline {

  template <size_t ORDER = 3>
  class hermite : public spline {
   public:
    using vector_t = typename spline::vector_t;
    using basis_t  = typename Eigen::Matrix<double, ORDER + 1, 1>;

    static_assert(ORDER == 3 || ORDER == 5,
                  "ORDER must be either 3 (cubic) or 5 (quintic)");

    struct waypoint {
      vector_t point, tangent, tangent_slope;
    };

    hermite() {}
    hermite(waypoint &wp0, waypoint &wp1) { set_waypoints(wp0, wp1); }

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
        b << (2 * t * t * t - 3 * t * t + 1), (t * t * t - 2 * t * t + t),
            (-2 * t * t * t + 3 * t * t), (t * t * t - t * t);
      } else {
        b << (1 - 10 * t * t * t + 15 * t * t * t * t - 6 * t * t * t * t * t),
            (t - 6 * t * t * t + 8 * t * t * t * t - 3 * t * t * t * t * t),
            (0.5 * t * t - 1.5 * t * t * t + 1.5 * t * t * t * t -
             0.5 * t * t * t * t * t),
            (10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t * t * t),
            (7 * t * t * t * t - 4 * t * t * t - 3 * t * t * t * t * t),
            (0.5 * t * t * t - t * t * t * t + 0.5 * t * t * t * t * t);
      }
      return b;
    }

    inline basis_t basis_1st(double t) const {
      basis_t b;
      if (ORDER == 3) {
        b << (6 * t * t - 6 * t), (3 * t * t - 4 * t + 1), (-6 * t * t + 6 * t),
            (3 * t * t - 2 * t);
      } else {
        b << (-30 * t * t + 60 * t * t * t - 30 * t * t * t * t),
            (1 - 18 * t * t + 32 * t * t * t - 15 * t * t * t * t),
            (t - 4.5 * t * t + 6 * t * t * t - 2.5 * t * t * t * t),
            (30 * t * t - 60 * t * t * t + 30 * t * t * t * t),
            (28 * t * t * t - 12 * t * t - 15 * t * t * t * t),
            (1.5 * t * t - 4 * t * t * t + 2.5 * t * t * t * t);
      }
      return b;
    }

    inline basis_t basis_2nd(double t) const {
      basis_t b;
      if (ORDER == 3) {
        b << (12 * t - 6), (6 * t - 4), (-12 * t + 6), (6 * t - 2);
      } else {
        b << (-60 * t + 180 * t * t - 120 * t * t * t),
            (-36 * t + 96 * t * t - 60 * t * t * t),
            (1 - 9 * t + 18 * t * t - 10 * t * t * t),
            (60 * t - 180 * t * t + 120 * t * t * t),
            (84 * t * t - 24 * t - 60 * t * t * t), (3 * t - 12 * t * t + 10 * t * t * t);
      }
      return b;
    }

    vector_t calculate(double t) override { return M * basis(t); }

    vector_t calculate_derivative(double t) override { return M * basis_1st(t); }

    vector_t calculate_second_derivative(double t) { return M * basis_2nd(t); }

    double curvature(double t) override {
      vector_t h_p = calculate_derivative(t), h_pp = calculate_second_derivative(t);

      return (h_p[0] * h_pp[1] - h_p[1] * h_pp[0]) / pow(h_p.norm(), 3);
      // return (sqrt(h_p.squaredNorm() * h_pp.squaredNorm() - pow(h_p.dot(h_pp), 2)) /
      //         pow(h_p.norm(), 3)) * (angle > 0 ? 1 : -1);
    }

   protected:
    waypoint _wp0, _wp1;

    Eigen::Matrix<double, 2, ORDER + 1> M;
  };

  namespace hermite_factory {
    template <typename hermite_t, typename output_iterator_t, typename iterator_wp_t>
    size_t generate(const iterator_wp_t wp_begin, const iterator_wp_t wp_end,
                    output_iterator_t &&out, size_t max_size) {
      if (wp_begin == wp_end) return 0;
      if (max_size < std::distance(wp_begin, wp_end)) return 0;  // Not enough space!

      size_t        size    = 0;
      iterator_wp_t last_wp = wp_begin, start = wp_begin;
      for (iterator_wp_t it = ++start; it != wp_end; it++) {
        *(out++) = hermite_t{*last_wp, *it};
        last_wp  = it;
        size++;
      }
      return size;
    }
  }  // namespace hermite_factory

}  // namespace spline
}  // namespace grpl