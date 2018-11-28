#pragma once

#include "arc.h"

namespace grpl {
namespace pf {
  namespace path {
    /**
     * Implementation of @ref arc2d with non-constant curvature.
     * 
     * The Augmented Arc is an arc with a non-continuous curvature, slightly disobeying 
     * the geometry of an arc segment. This class is used as the output of an approximation
     * done by @ref arc_parameterizer designed to approximate splines with continous curvature.
     * 
     * The curvature is interpolated with respect to the arc length of the segment. This is
     * necessary for certain systems that require a parameterized spline.
     */
    class augmented_arc2d : public arc2d {
     public:
      augmented_arc2d() : arc2d(){};

      /**
       * Create a circular arc from a set of 3 points (start, any, and end).
       * 
       * @param start The start point of the curve, in x,y metres.
       * @param mid   Any point along the curve sitting between start and end, 
       *              in x,y metres.
       * @param end   The end point of the curve, in x,y metres.
       */
      augmented_arc2d(vector_t start, vector_t mid, vector_t end) : arc2d(start, mid, end) {}

      /**
       * Create a circular arc from a set of 3 points (start, any, and end), and the
       * start and end curvature.
       * 
       * @param start   The start point of the curve, in x,y metres.
       * @param mid     Any point along the curve sitting between start and end, 
       *                in x,y metres.
       * @param end     The end point of the curve, in x,y metres.
       * @param start_k The starting curvature value k in m^-1.
       * @param end_k   The ending curvature value k in m^-1.
       */
      augmented_arc2d(vector_t start, vector_t mid, vector_t end, double start_k, double end_k)
          : arc2d(start, mid, end) {
        set_curvature(start_k, end_k);
      }

      /**
       * Set the start and end curvature values for interpolation.
       * 
       * @param start_k The starting curvature value k in m^-1.
       * @param end_k   The ending curvature value k in m^-1.
       */
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

      double dcurvature(double s) const override { return _dk_ds; }

     private:
      double _curvature, _dk_ds;
      bool   _curvature_set = false;
    };
  }  // namespace path
}  // namespace pf
}  // namespace grpl