#pragma once

#include <blaze/math/StaticVector.h>
#include "grpl/path/path.h"

namespace grpl {
namespace path {

  // TODO: Allow definition of vector type?
  //        (i.e. static or dynamic)
  template <typename UNIT, size_t DIM>
  class hermite : public path<UNIT, DIM> {
  public:
    using vector_t = blaze::StaticVector<UNIT, DIM, blaze::columnVector>;
    struct waypoint {
      vector_t point, tangent;
    };

    hermite(waypoint &wp0, waypoint &wp1, unsigned int arclength_samples) 
      : _wp0(wp0), _wp1(wp1), _al_samples(arclength_samples) {}

    vector_t calculate(double t) const override {
      blaze::StaticMatrix<UNIT, DIM, 4> M;
      blaze::column<0>(M) = _wp0.point;
      blaze::column<1>(M) = _wp0.tangent;
      blaze::column<2>(M) = _wp1.point;
      blaze::column<3>(M) = _wp1.tangent;

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
      blaze::StaticMatrix<UNIT, DIM, 4> M;
      blaze::column<0>(M) = _wp0.point;
      blaze::column<1>(M) = _wp0.tangent;
      blaze::column<2>(M) = _wp1.point;
      blaze::column<3>(M) = _wp1.tangent;

      // Hermite basis functions (1st derivative)
      blaze::StaticVector<double, 4, blaze::columnVector> h {
        (6 * t*t - 6 * t),
        (3*t*t - 4 * t + 1),
        (-6 * t*t + 6 * t),
        (3 * t*t - 2 * t)
      };
      return M * h;
    }

    // This uses matrix 'batches' to become highly parallelized for faster computation
    UNIT calculate_arc_length() const override {
      double t = 0, dt = (1.0 / _al_samples);

      blaze::StaticMatrix<UNIT, DIM, 4> M;
      blaze::column<0>(M) = _wp0.point;
      blaze::column<1>(M) = _wp0.tangent;
      blaze::column<2>(M) = _wp1.point;
      blaze::column<3>(M) = _wp1.tangent;

      const size_t batch_size = 16;
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

      return static_cast<UNIT>(arc_length);
    }

  protected:
    waypoint _wp0, _wp1;
    unsigned int _al_samples = 10000;
  };

}
}