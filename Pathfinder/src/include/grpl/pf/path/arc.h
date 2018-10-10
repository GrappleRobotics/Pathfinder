#pragma once

#include "curve.h"
#include "grpl/pf/constants.h"

namespace grpl {
namespace pf {
  namespace path {

    class arc2d : public curve<2> {
     public:
      using vector_t = typename curve::vector_t;

      arc2d() {}
      arc2d(vector_t start, vector_t mid, vector_t end) { from_three(start, mid, end); }

      vector_t position(const double s) const override {
        double curv = _curvature;
        if (curv != 0) {
          double angle = _angle_offset + (s * curv);
          return _ref + vector_t{cos(angle) / fabs(curv), sin(angle) / fabs(curv)};
        } else {
          return _ref + _delta * (s / _length);
        }
      }

      vector_t velocity(const double s) const override {
        double curv = _curvature;
        if (curv != 0) {
          double sign  = curv > 0 ? 1 : -1;
          double angle = _angle_offset + (s * curv);
          double off = sign * constants::PI / 2;
          return vector_t{cos(angle + off), sin(angle + off)};
        } else {
          return _delta;
        }
      }

      double curvature(const double s) const override { return _curvature; }

      double curvature_prime(const double s) const override { return 0; }

      double length() const override { return _length; }

     private:
      void from_three(vector_t start, vector_t mid, vector_t end) {
        Eigen::Matrix<double, 2, 2> coeffmatrix;
        Eigen::Matrix<double, 2, 1> rvec;

        coeffmatrix << 2 * (start[0] - end[0]), 2 * (start[1] - end[1]), 2 * (start[0] - mid[0]),
            2 * (start[1] - mid[1]);

        if (coeffmatrix.determinant() == 0) {
          // start, mid and end are all colinear, therefore
          // this arc is, in fact, a straight line.
          _curvature    = 0;
          _ref          = start;
          _delta        = end - start;
          _length       = _delta.norm();
          _angle_offset = atan2(_delta[1], _delta[0]);
        } else {
          rvec << start.squaredNorm() - end.squaredNorm(), start.squaredNorm() - mid.squaredNorm();

          _ref = coeffmatrix.inverse() * rvec;

          _angle_offset = atan2((start - _ref)[1], (start - _ref)[0]);
          double angle1 = atan2((end - _ref)[1], (end - _ref)[0]);

          _curvature = 1.0 / (start - _ref).norm();
          _length    = fabs((angle1 - _angle_offset)) / _curvature;
          _curvature *= ((angle1 - _angle_offset) < 0 ? -1 : 1);
        }
      }

      // Line: Initial point, Arc: Centre Point
      vector_t _ref;
      // Line uses delta (more efficient, no trig calcs),
      // whilst circle uses angle offset.
      vector_t _delta;
      double   _angle_offset;
      double   _curvature;
      double   _length;
    };
  }  // namespace path
}  // namespace pf
}  // namespace grpl