#pragma once

#include "grpl/path/path.h"

namespace grpl {
namespace path {

  template <size_t DIM>
  class hermite : public path<DIM> {
  public:
    using vector_t = typename path<DIM>::vector_t;
    struct waypoint {
      vector_t point, tangent;
    };

    hermite(waypoint &wp0, waypoint &wp1, unsigned int arclength_samples) 
      : _wp0(wp0), _wp1(wp1), _al_samples(arclength_samples) {
        blaze::column<0>(M) = _wp0.point;
        blaze::column<1>(M) = _wp0.tangent;
        blaze::column<2>(M) = _wp1.point;
        blaze::column<3>(M) = _wp1.tangent;
      }

    vector_t calculate(double t) const override {
      // Hermite basis functions
      blaze::StaticVector<double, 4, blaze::columnVector> h {
        (2 * t*t*t - 3 * t*t + 1),
        (t*t*t - 2 * t*t + t),
        (-2 * t*t*t + 3 * t*t),
        (t*t*t - t*t)
      };
      return M * h;
    }

    vector_t calculate_slope(double t) const override {
      // Hermite basis functions (1st derivative)
      blaze::StaticVector<double, 4, blaze::columnVector> h {
        (6 * t*t - 6 * t),
        (3*t*t - 4 * t + 1),
        (-6 * t*t + 6 * t),
        (3 * t*t - 2 * t)
      };
      return M * h;
    }

    vector_t calculate_slope_second(double t) const {
      // Hermite basis functions (2nd derivative)
      blaze::StaticVector<double, 4, blaze::columnVector> h {
        (12 * t - 6),
        (6*t - 4),
        (-12 * t + 6),
        (6 * t - 2)
      };
      return M * h;
    }

    double calculate_curvature(double t) const override {
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

        blaze::StaticMatrix<double, DIM, 4> M;
        blaze::column<0>(M) = _wp0.point;
        blaze::column<1>(M) = _wp0.tangent;
        blaze::column<2>(M) = _wp1.point;
        blaze::column<3>(M) = _wp1.tangent;

        const size_t batch_size = 32;
        double last_integrand = 0, arc_length = 0;
        for (t = 0; t <= 1; t += batch_size*dt) {
          blaze::StaticMatrix<double, 4, batch_size> H;

          for (size_t i = 0; i < batch_size; i++) {
            double u = t + i*dt;
            // Hermite basis functions (1st deriv) for each batch
            H(0, i) = (6 * u*u - 6 * u);
            H(1, i) = (3*u*u - 4 * u + 1);
            H(2, i) = (-6 * u*u + 6 * u);
            H(3, i) = (3 * u*u - 2 * u);
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
    blaze::StaticMatrix<double, DIM, 4> M;
    unsigned int _al_samples = 10000;
    bool _al_calculated = false;
    double _al_last = 0;
  };

}
}