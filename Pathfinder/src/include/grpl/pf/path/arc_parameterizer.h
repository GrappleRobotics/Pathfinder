#pragma once

#include "augmented_arc.h"
#include "spline.h"

namespace grpl {
namespace pf {
  namespace path {
    // TODO: This can be genericized.
    // TODO:  Replace curve_count and parameterize with a spliterator-like
    //        pattern if efficient. Need to benchmark before implementation.
    //        Another thought, we can actually genericize it into a bounds
    //        divider, with visit and subdivide being combined into one function.
    //        Default subclasses can handle the predicate and visit splitting.
    class arc_parameterizer {
     public:
      using curve_t  = augmented_arc2d;
      using vector_t = curve_t::vector_t;

      arc_parameterizer() {}

      void configure(double max_arc_length, double max_delta_curvature) {
        _max_arc_length      = max_arc_length;
        _max_delta_curvature = max_delta_curvature;
      }

      bool has_overrun() { return _has_overrun; }

      size_t curve_count(spline<2> &spline, double t_lo = 0, double t_hi = 1, size_t count = 0) const {
        double t_mid = (t_hi + t_lo) / 2.0;
        double k_lo  = spline.curvature(t_lo);
        double k_hi  = spline.curvature(t_hi);

        augmented_arc2d arc{spline.position(t_lo), spline.position(t_mid), spline.position(t_hi), k_lo, k_hi};

        bool subdivide = (fabs(k_hi - k_lo) > _max_delta_curvature) || (arc.length() > _max_arc_length);

        if (subdivide) {
          count = curve_count(spline, t_lo, t_mid, count);
          count = curve_count(spline, t_mid, t_hi, count);
          return count;
        } else {
          return count + 1;
        }
      }

      template <typename iterator_spline_t>
      size_t curve_count(iterator_spline_t spline_begin, iterator_spline_t spline_end,
                         size_t count = 0) const {
        size_t total_count = 0;
        for (iterator_spline_t it = spline_begin; it != spline_end; it++) {
          total_count += curve_count(*it);
        }
        return total_count;
      }

      template <typename output_iterator_t>
      size_t parameterize(spline<2> &spline, output_iterator_t &&curve_begin, const size_t max_curve_count,
                          double t_lo = 0, double t_hi = 1) {
        _has_overrun = false;
        if (max_curve_count <= 0) {
          _has_overrun = true;
          return 0;
        }

        double t_mid = (t_hi + t_lo) / 2.0;
        double k_lo  = spline.curvature(t_lo);
        double k_hi  = spline.curvature(t_hi);

        augmented_arc2d arc{spline.position(t_lo), spline.position(t_mid), spline.position(t_hi), k_lo, k_hi};

        bool subdivide = (fabs(k_hi - k_lo) > _max_delta_curvature) || (arc.length() > _max_arc_length);

        if (subdivide) {
          output_iterator_t head = curve_begin;

          size_t len = parameterize(spline, head, max_curve_count, t_lo, t_mid);
          len += parameterize(spline, head, max_curve_count - len, t_mid, t_hi);
          return len;
        } else {
          arc.set_curvature(k_lo, k_hi);
          *(curve_begin++) = arc;
          return 1;
        }
      }

      template <typename output_iterator_t, typename iterator_spline_t>
      size_t parameterize(const iterator_spline_t spline_begin, const iterator_spline_t spline_end,
                          output_iterator_t &&curve_begin, const size_t max_curve_count) {
        size_t len = 0;
        for (iterator_spline_t it = spline_begin; it != spline_end; it++) {
          len += parameterize(*it, curve_begin, max_curve_count - len);
        }
        return len;
      }

     private:
      double _max_arc_length;
      double _max_delta_curvature;
      bool   _has_overrun;
    };
  }  // namespace path
}  // namespace pf
}  // namespace grpl