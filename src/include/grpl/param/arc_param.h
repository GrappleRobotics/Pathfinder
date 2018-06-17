#pragma once

#include "grpl/curve/arc.h"
#include "grpl/param/param.h"

namespace grpl {
namespace param {

  // Augmented Arc is an arc with a non-constant curvature.
  // This slightly disobeys the geometry of the arc, since an arc is, by
  // definition, a circle segment, and a circles radius (therefore curvature)
  // is constant, however this functions as an approximation so that curvature
  // at knot points in the parameterization (spline sections) is continuous, via
  // linear interpolation with dk/ds. As such, if you run a test on curvature
  // vs radius of the circle, they will not match, instead being (slightly) out.
  // This is another case of pure math not _quite_ working out in real-world
  // applications, who would-a thunk it.
  class augmented_arc2d : public grpl::curve::arc2d {
   public:
    augmented_arc2d() : grpl::curve::arc2d() {}
    augmented_arc2d(vector_t start, vector_t mid, vector_t end)
        : grpl::curve::arc2d(start, mid, end) {}

    void set_curvature(double start_k, double end_k) {
      _curvature       = start_k;
      _curvature_dk_ds = (end_k - start_k) / length();
      _curvature_set   = true;
    }

    double curvature(const double s) const override {
      if (_curvature_set)
        return _curvature + s * _curvature_dk_ds;
      else
        return grpl::curve::arc2d::curvature(s);
    }

   private:
    double _curvature, _curvature_dk_ds;
    bool   _curvature_set = false;
  };

  class arc_parameterizer : public parameterizer<2, augmented_arc2d> {
   public:
    arc_parameterizer() {}
    arc_parameterizer(grpl::path::path<2> *path) : parameterizer(path) {}

    void configure(double max_arc_length, double max_delta_curvature) {
      _max_arc_length      = max_arc_length;
      _max_delta_curvature = max_delta_curvature;
    }

    // Recurse, assumes path exists
    // TODO: Error condition if we reach max curves, continue with a valid output, but
    // signal an error of some kind.
    size_t parameterize_step(augmented_arc2d *curves, size_t max_curves,
                             size_t current_curves, double tLo, double tHi) const {
      double tMid = (tHi + tLo) / 2.0;

      grpl::path::path<2> *path = get_path();
      augmented_arc2d      current_arc(path->calculate(tLo), path->calculate(tMid),
                                  path->calculate(tHi));

      double kLo = path->curvature(tLo);
      double kHi = path->curvature(tHi);

      bool subdivide = (fabs(kHi - kLo) > _max_delta_curvature) ||
                       (current_arc.length() > _max_arc_length);

      // TODO: Better job at detecting when we're near the end
      if (subdivide && current_curves < max_curves) {
        // Bisect at half-way point of t
        current_curves =
            parameterize_step(curves, max_curves, current_curves, tLo, tMid);
        current_curves =
            parameterize_step(curves, max_curves, current_curves, tMid, tHi);
        return current_curves;
      } else {
        // Either we shouldn't subdivide, or we're 1 item away from capacity
        // Insert this current curve
        current_arc.set_curvature(kLo, kHi);
        *(curves + current_curves) = current_arc;  // Copy arc
        return current_curves + 1;
      }
    }

    size_t parameterize(augmented_arc2d *curves, size_t max_curves) override {
      if (has_path())
        return parameterize_step(curves, max_curves, 0, 0, 1);
      else
        return 0;
    }

   private:
    double _max_arc_length;
    double _max_delta_curvature;
  };
}  // namespace param
}  // namespace grpl