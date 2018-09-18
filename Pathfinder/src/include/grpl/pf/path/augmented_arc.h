#pragma once

#include "grpl/pf/path/arc.h"

namespace grpl {
namespace pf {
  namespace path {
    // Augmented Arc is an arc with a non-constant curvature.
    // This slightly disobeys the geometry of the arc, since an arc is, by
    // definition, a circle segment, and a circles radius (therefore curvature)
    // is constant, however this functions as an approximation so that curvature
    // at knot points in the parameterization (spline sections) is continuous, via
    // linear interpolation with dk/ds. As such, if you run a test on curvature
    // vs radius of the circle, they will not match, instead being (slightly) out.
    // This is another case of pure math not _quite_ working out in real-world
    // applications, who would-a thunk it.
    class augmented_arc2d : public arc2d {
     public:
      augmented_arc2d() : arc2d(){};
      augmented_arc2d(vector_t start, vector_t mid, vector_t end) : arc2d(start, mid, end) {}
      augmented_arc2d(vector_t start, vector_t mid, vector_t end, double start_k, double end_k)
          : arc2d(start, mid, end) {
        set_curvature(start_k, end_k);
      }

      void set_curvature(double start_k, double end_k) {
        _curvature     = start_k;
        _dk_ds         = (end_k - start_k) / length();
        _curvature_set = true;
      }

      double curvature(double s) const override {
        if (_curvature_set)
          return _curvature + s * _dk_ds;
        else
          return arc2d::curvature(s);
      }

      double curvature_prime(double s) const override { return _dk_ds; }

     private:
      double _curvature, _dk_ds;
      bool   _curvature_set = false;
    };
  }  // namespace path
}  // namespace pf
}  // namespace grpl