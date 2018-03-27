#pragma once

#include "grpl/path/path.h"

namespace grpl {
namespace path {

  template <size_t DIM, size_t ORDER=3>
  class hermite : public path<DIM> {
  public:
    using vector_t = typename path<DIM>::vector_t;

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
        blaze::column<0>(M) = _wp0.point;
        blaze::column<1>(M) = _wp0.tangent;
        blaze::column<2>(M) = _wp1.point;
        blaze::column<3>(M) = _wp1.tangent;
      } else if (ORDER == 5) {
        blaze::column<0>(M) = _wp0.point;
        blaze::column<1>(M) = _wp0.tangent;
        blaze::column<2>(M) = _wp0.tangent_slope;
        blaze::column<3>(M) = _wp1.point;
        blaze::column<4>(M) = _wp1.tangent;
        blaze::column<5>(M) = _wp1.tangent_slope;
      }
    }

    inline blaze::StaticVector<double, ORDER+1> basis(double t) const {
      if (ORDER == 3) {
        return blaze::StaticVector<double, ORDER+1> {
          (2 * t*t*t - 3 * t*t + 1),
          (t*t*t - 2 * t*t + t),
          (-2 * t*t*t + 3 * t*t),
          (t*t*t - t*t)
        };
      } else {
        return blaze::StaticVector<double, ORDER+1> {
          (1 - 10*t*t*t + 15*t*t*t*t - 6*t*t*t*t*t),
          (t - 6*t*t*t + 8*t*t*t*t - 3*t*t*t*t*t),
          (0.5*t*t - 1.5*t*t*t + 1.5*t*t*t*t - 0.5*t*t*t*t*t),
          (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t),
          (7*t*t*t*t - 4*t*t*t - 3*t*t*t*t*t),
          (0.5*t*t*t - t*t*t*t + 0.5*t*t*t*t*t)
        };
      }
    }

    inline blaze::StaticVector<double, ORDER+1> basis_1st(double t) const {
      if (ORDER == 3) {
        return blaze::StaticVector<double, ORDER+1> {
          (6 * t*t - 6 * t),
          (3*t*t - 4 * t + 1),
          (-6 * t*t + 6 * t),
          (3 * t*t - 2 * t)
        };
      } else {
        return blaze::StaticVector<double, ORDER+1> {
          (-30*t*t + 60*t*t*t - 30*t*t*t*t),
          (1 - 18*t*t + 32*t*t*t - 15*t*t*t*t),
          (t - 4.5*t*t + 6*t*t*t - 2.5*t*t*t*t),
          (30*t*t - 60*t*t*t + 30*t*t*t*t),
          (28*t*t*t - 12*t*t - 15*t*t*t*t),
          (1.5*t*t - 4*t*t*t + 2.5*t*t*t*t)
        };
      }
    }

    inline blaze::StaticVector<double, ORDER+1> basis_2nd(double t) const {
      if (ORDER == 3) {
        return blaze::StaticVector<double, ORDER+1> {
          (12 * t - 6),
          (6*t - 4),
          (-12 * t + 6),
          (6 * t - 2)
        };
      } else {
        return blaze::StaticVector<double, ORDER+1> {
          (-60*t + 180*t*t - 120*t*t*t),
          (-36*t + 96*t*t - 60*t*t*t),
          (1 - 9*t + 18*t*t - 10*t*t*t),
          (60*t - 180*t*t + 120*t*t*t),
          (84*t*t - 24*t - 60*t*t*t),
          (3*t - 12*t*t + 10*t*t*t)
        };
      }
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
        sqrt( sqrLength(h_p)*sqrLength(h_pp) - pow(dot(h_p, h_pp), 2) )
        / pow(length(h_p), 3)
      );
    }

    // This uses matrix 'batches' to become highly parallelized for faster computation
    double get_arc_length() override {
      if (!_al_calculated) {
        double t = 0, dt = (1.0 / _al_samples);

        const size_t batch_size = 32;
        double last_integrand = 0, arc_length = 0;
        for (t = 0; t <= 1; t += batch_size*dt) {
          blaze::StaticMatrix<double, ORDER+1, batch_size> H;

          for (size_t i = 0; i < batch_size; i++) {
            double u = t + i*dt;
            // Hermite basis functions (1st deriv) for each batch
            column(H, i) = basis_1st(u);
            // H(0, i) = (6 * u*u - 6 * u);
            // H(1, i) = (3*u*u - 4 * u + 1);
            // H(2, i) = (-6 * u*u + 6 * u);
            // H(3, i) = (3 * u*u - 2 * u);
          }
          
          // Calculate derivative points at each batch. This is where the optimization
          // comes in
          auto R = M * H;

          for (size_t i = 0; i < batch_size; i++) {
            double dydt = 0;
            // Calculate L2-norm (magnitude) of each batch column vector
            for (size_t d = 0; d < DIM; d++) {
              dydt += blaze::pow2(static_cast<double>(R(d, i)));
            }
            dydt = sqrt(dydt);

            // Arc length calculation
            double integrand = sqrt(1 + dydt*dydt) * dt;
            if (t + i*dt <= 1) {
              arc_length += (integrand + last_integrand) / 2;
            }
            last_integrand = integrand;
          }
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
    blaze::StaticMatrix<double, DIM, ORDER+1> M;
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