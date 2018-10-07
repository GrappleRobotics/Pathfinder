#pragma once

#include "spline.h"
#include "grpl/pf/util/constants.h"

namespace grpl {
namespace pf {
  namespace path {

    template <size_t ORDER = 3>
    class hermite : public spline<2> {
     public:
      using vector_t = typename spline::vector_t;
      using basis_t  = typename Eigen::Matrix<double, ORDER + 1, 1>;

      static_assert(ORDER == 3 || ORDER == 5, "ORDER must be either 3 (cubic) or 5 (quintic)");

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
        if (ORDER == 3) {
          // 2t^3 - 3t^2 + 1
          // t^3 - 2t^2 + t
          // -2t^3 + 3t^2
          // t^3 - t^2
          return (basis_t() << (2 * t * t * t - 3 * t * t + 1), (t * t * t - 2 * t * t + t),
                  (-2 * t * t * t + 3 * t * t), (t * t * t - t * t))
              .finished();
        } else {
          // 1 - 10t^3 + 15t^4 - 6t^5
          // t - 6t^3 + 8t^4 - 3t^5
          // 0.5*t^2 - 1.5t^3 + 1.5t^4 - 0.5t^5
          // 10t^3 - 15t^4 + 6t^5
          // 7t^4 - 4t^3 - 3t^5
          // 0.5t^3 - t^4 + 0.5t^5
          return (basis_t() << (1 - 10 * t * t * t + 15 * t * t * t * t - 6 * t * t * t * t * t),
                  (t - 6 * t * t * t + 8 * t * t * t * t - 3 * t * t * t * t * t),
                  (0.5 * t * t - 1.5 * t * t * t + 1.5 * t * t * t * t - 0.5 * t * t * t * t * t),
                  (10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t * t * t),
                  (7 * t * t * t * t - 4 * t * t * t - 3 * t * t * t * t * t),
                  (0.5 * t * t * t - t * t * t * t + 0.5 * t * t * t * t * t))
              .finished();
        }
      }

      // derivatives of basis
      inline basis_t basis_1st(double t) const {
        if (ORDER == 3) {
          return (basis_t() << (6 * t * t - 6 * t), (3 * t * t - 4 * t + 1), (-6 * t * t + 6 * t),
                  (3 * t * t - 2 * t))
              .finished();
        } else {
          return (basis_t() << (-30 * t * t + 60 * t * t * t - 30 * t * t * t * t),
                  (1 - 18 * t * t + 32 * t * t * t - 15 * t * t * t * t),
                  (t - 4.5 * t * t + 6 * t * t * t - 2.5 * t * t * t * t),
                  (30 * t * t - 60 * t * t * t + 30 * t * t * t * t),
                  (28 * t * t * t - 12 * t * t - 15 * t * t * t * t),
                  (1.5 * t * t - 4 * t * t * t + 2.5 * t * t * t * t))
              .finished();
        }
      }

      // 2nd derivatives of basis
      inline basis_t basis_2nd(double t) const {
        if (ORDER == 3) {
          return (basis_t() << (12 * t - 6), (6 * t - 4), (-12 * t + 6), (6 * t - 2)).finished();
        } else {
          return (basis_t() << (-60 * t + 180 * t * t - 120 * t * t * t),
                  (-36 * t + 96 * t * t - 60 * t * t * t), (1 - 9 * t + 18 * t * t - 10 * t * t * t),
                  (60 * t - 180 * t * t + 120 * t * t * t), (84 * t * t - 24 * t - 60 * t * t * t),
                  (3 * t - 12 * t * t + 10 * t * t * t))
              .finished();
        }
      }

      vector_t position(double t) override { return M * basis(t); }
      vector_t velocity(double t) override { return M * basis_1st(t); }
      vector_t acceleration(double t) { return M * basis_2nd(t); }

      double curvature(double t) override {
        vector_t h_p = velocity(t), h_pp = acceleration(t);

        return (h_p[0] * h_pp[1] - h_p[1] * h_pp[0]) / pow(h_p.norm(), 3);
      }

     protected:
      waypoint _wp0, _wp1;

      Eigen::Matrix<double, 2, ORDER + 1> M;
    };

    using hermite_cubic   = hermite<3>;
    using hermite_quintic = hermite<5>;

    // TODO: More Hermite generation methods. Perhaps this should be in another file?
    namespace hermite_factory {
      template <typename hermite_t, typename output_iterator_t, typename iterator_wp_t>
      size_t generate(const iterator_wp_t wp_begin, const iterator_wp_t wp_end, output_iterator_t &&out,
                      size_t max_size) {
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
  }    // namespace path
}  // namespace pf
}  // namespace grpl