#pragma once

#include "grpl/curve/curve.h"
#include "grpl/util/constants.h"

namespace grpl {
namespace curve {

  class arc2d : public curve<2> {
   public:
    using vector_t = typename curve<2>::vector_t;
    arc2d(vector_t start, vector_t mid, vector_t end) {
      Eigen::Matrix<double, 2, 2> coeffmatrix;
      Eigen::Matrix<double, 2, 1> rvec;

      coeffmatrix << 2 * (start[0] - end[0]), 2 * (start[1] - end[1]),
          2 * (start[0] - mid[0]), 2 * (start[1] - mid[1]);
      rvec << start.squaredNorm() - end.squaredNorm(),
          start.squaredNorm() - mid.squaredNorm();

      _center = coeffmatrix.inverse() * rvec;
      _radius = (start - _center).norm();

      _angle_start = atan2((start - _center)[1], (start - _center)[0]);
      _angle_end = atan2((end - _center)[1], (end - _center)[0]);
    }

    vector_t calculate(const double s) const override {
      double angle =
          _angle_start + (s / _radius) * ((_angle_end - _angle_start) < 0 ? -1 : 1);
      return _center + vector_t{_radius * cos(angle), _radius * sin(angle)};
    }

    // TODO: This or angle?
    // TODO: I don't think this can be considered unit in value, run a sim to check
    vector_t calculate_derivative(const double s) const {
      double sign = ((_angle_end - _angle_start) < 0 ? -1 : 1);
      double angle = _angle_start + (s / _radius) * sign;
      // Angle is normal to path
      return vector_t{ cos(angle + sign*PI/2), sin(angle + sign*PI/2) };
    }

    double curvature(const double s) const {
      return 1.0 / _radius;
    }

    double length() const {
      return fabs((_angle_end - _angle_start)) * _radius;
    }

   private:
    vector_t _center;
    double _radius;
    double _angle_start, _angle_end;
  };

}  // namespace curve
}  // namespace grpl