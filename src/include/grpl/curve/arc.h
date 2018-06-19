#pragma once

#include "grpl/curve/curve.h"
#include "grpl/util/constants.h"

#include <iostream>

namespace grpl {
namespace curve {

  class arc2d : public curve<2> {
   public:
    using vector_t = typename curve<2>::vector_t;

    arc2d() {}
    arc2d(vector_t start, vector_t mid, vector_t end) { from_three(start, mid, end); }

    void from_three(vector_t start, vector_t mid, vector_t end) {
      Eigen::Matrix<double, 2, 2> coeffmatrix;
      Eigen::Matrix<double, 2, 1> rvec;

      coeffmatrix << 2 * (start[0] - end[0]), 2 * (start[1] - end[1]),
          2 * (start[0] - mid[0]), 2 * (start[1] - mid[1]);

      if (coeffmatrix.determinant() == 0) {
        // start, mid and end are all colinear, therefore
        // this arc is, in fact, a straight line.
        _curvature = 0;
        _ref       = start;
        _delta     = end - start;
        _length    = _delta.norm();
      } else {
        rvec << start.squaredNorm() - end.squaredNorm(),
            start.squaredNorm() - mid.squaredNorm();

        _ref       = coeffmatrix.inverse() * rvec;
        _curvature = 1.0 / (start - _ref).norm();

        _angle_offset = atan2((start - _ref)[1], (start - _ref)[0]);
        double angle1 = atan2((end - _ref)[1], (end - _ref)[0]);

        _length = fabs((angle1 - _angle_offset)) / _curvature;
        _sign   = ((angle1 - _angle_offset) < 0 ? -1 : 1);  // Sign for curvature
      }
    }

    vector_t calculate(const double s) const override {
      if (_curvature != 0) {
        double angle = _angle_offset + (s * _curvature) * _sign;
        return _ref + vector_t{cos(angle) / _curvature, sin(angle) / _curvature};
      } else {
        return _ref + _delta * (s / _length);
      }
    }

    vector_t calculate_derivative(const double s) const override {
      if (_curvature != 0) {
        double angle = _angle_offset + (s * _curvature) * _sign;
        return vector_t{cos(angle + _sign * PI / 2), sin(angle + _sign * PI / 2)};
      } else {
        return _delta;
      }
    }

    double curvature(const double s) const override { return _curvature; }

    double length() const override { return _length; }

   private:
    // Line: Initial point, Arc: Center Point
    vector_t _ref;
    // Line uses delta (more efficient, no trig calcs),
    // whilst circle uses angle offset.
    vector_t _delta;
    double   _angle_offset;
    // Remaining values consistent.
    double _curvature;
    double _sign;
    double _length;
  };

}  // namespace curve
}  // namespace grpl