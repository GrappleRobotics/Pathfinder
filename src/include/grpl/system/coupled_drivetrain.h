#pragma once

#include "grpl/curve/curve.h"
#include "grpl/profile/profile.h"
#include "grpl/util/constants.h"
#include "grpl/util/vec.h"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

namespace grpl {
namespace system {

  template <typename curve_t, typename profile_t>
  class coupled_drivetrain {
   public:
    static_assert(curve_t::DIMENSIONS == 2,
                  "Curve must function in exactly 2 Dimensions for Coupled Drivetrain!");
    using kinematics_t    = Eigen::Matrix<double, curve_t::DIMENSIONS, profile_t::ORDER>;
    using kinematics_1d_t = typename profile_t::kinematics_1d_t;
    using vector_t        = typename curve_t::vector_t;

    void apply_limit(int derivative_idx, double maximum) { _limits[derivative_idx] = maximum; }

    kinematics_1d_t &get_limits() { return _limits; }
    void             set_limits(kinematics_1d_t &other) { _limits = other; }

    void   set_trackwidth(double tw) { _trackwidth = tw; }
    double get_trackwidth() const { return _trackwidth; }

    struct coupled_side_t {
      double       t;
      double       d;
      kinematics_t k;
    };

    struct state {
      coupled_side_t  l, c, r;
      kinematics_1d_t a;
      double          curvature;
      bool            done;
    };

    state zero_state() {
      state s;
      s.curvature = 0;
      s.done      = false;
      s.r.t       = 0;
      s.r.d       = 0;
      s.r.k.setZero();
      s.l = s.c = s.r;
      return s;
    }

    template <typename iterator_curve_t>
    inline bool find_curve(double targ_len, const iterator_curve_t curve_begin,
                           const iterator_curve_t curve_end, iterator_curve_t &curve_out,
                           double &curve_len_out, double &total_len_out) {
      curve_len_out = targ_len;
      total_len_out = 0;
      bool found    = false;

      for (iterator_curve_t it = curve_begin; it != curve_end; it++) {
        double len = it->length();
        // If we haven't found a curve, and the current length of the curve will put us ahead
        // of our distance target.
        if (!found && (len + total_len_out) >= targ_len) {
          found         = true;
          curve_len_out = targ_len - total_len_out;
          curve_out     = it;
        }
        total_len_out += len;
      }
      return found;
    }

    template <typename iterator_curve_t>
    state generate(const iterator_curve_t curve_begin, const iterator_curve_t curve_end, profile_t &profile,
                   state &last, double time) {
      state output;

      iterator_curve_t curve;
      double           total_length, curve_distance;
      double           distance = last.c.d;

      bool curve_found = find_curve(distance, curve_begin, curve_end, curve, curve_distance, total_length);

      if (!curve_found) {
        output      = last;
        output.done = true;
        return output;
      }

      double trackradius = _trackwidth / 2.0;
      double dt          = time - last.c.t;
      bool   isFirst     = (dt < 0.0001);

      vector_t center = curve->calculate(curve_distance);
      // TODO: We can't trust derivative to be continuous across curves.
      vector_t center_slope = curve->calculate_derivative(curve_distance);
      double curvature = curve->curvature(curve_distance);
      double angle     = curve->angle(curve_distance);
      // double   angle        = atan2(center_slope[1], center_slope[0]);
      vector_t unit_heading = vec_polar(1, angle);

      size_t spl = std::distance(curve_begin, curve);
      // Determine angle and derivatives
      output.a[0] = angle;  // TODO: use small change to get angular vel/acc
      double cdist = 0.0001;
      kinematics_1d_t cangles;
      cangles[0] = curve->angle(curve_distance+cdist);

      // If I can find the period of rotation for the outer track, 
      // I can also find the period of rotation for inner and center track.
      // From that, I can then gaint he differential velocity?

      for (size_t i = 1; i < profile_t::ORDER; i++) {
        double diff = output.a[i - 1] - last.a[i - 1];
        // if (i == 1)
        //   std::cout << spl << "," << time << "," << dt << "," << angle << " - " << last.a[0] << " = " <<
        //   diff << std::endl;
        output.a[i] = (isFirst) ? 0 : diff / dt;
      }

      // Set the new limits on the velocity profile
      // Half of difference between left and right side for each of { vel, acc, etc }
      kinematics_1d_t differentials = output.a * trackradius;
      kinematics_1d_t new_limits    = _limits - differentials.cwiseAbs();

      profile.set_goal(total_length);
      profile.set_limits(new_limits);  // TODO: new limits

      // Calculate velocity profile
      typename profile_t::segment_t segment;
      segment.time = last.c.t;
      for (size_t i = 0; i < profile_t::ORDER; i++) {
        segment.k[i] = last.c.k.col(i).dot(unit_heading);
      }
      segment.k[0] = distance;
      segment      = profile.calculate(segment, time);

      // Set output values
      output.curvature = curvature;
      output.c.t = output.l.t = output.r.t = time;

      output.c.d = segment.k[0];
      output.c.k = unit_heading * segment.k;

      output.c.k.col(0) = center;
      output.l.k.col(0) = output.c.k.col(0) - vec_polar(trackradius, output.a[0] - PI / 2.0);
      output.r.k.col(0) = output.c.k.col(0) + vec_polar(trackradius, output.a[0] - PI / 2.0);

      // a = v^2 / r
      // TODO: Accel for outer tracks
      // TODO: Generalize this for n order motion
      double acc_in = (output.c.k.col(1).squaredNorm()) * curvature;
      acc_in *= (output.a[1] > 0 ? 1 : -1);
      output.c.k.col(2) += vec_polar(acc_in, output.a[0] + PI / 2.0);

      vector_t vel_center = output.c.k.col(1);
      vector_t vel_outer  = vel_center * (1 + trackradius * curvature);
      vector_t vel_inner  = vel_center * (1 - trackradius * curvature);

      if (output.a[1] > 0) {
        // right = outer
        output.r.k.col(1) = vel_outer;
        output.l.k.col(1) = vel_inner;
      } else {
        // left = outer
        output.r.k.col(1) = vel_inner;
        output.l.k.col(1) = vel_outer;
      }

      output.l.d = last.l.d + output.l.k.col(1).dot(unit_heading) * dt;
      output.r.d = last.r.d + output.r.k.col(1).dot(unit_heading) * dt;

      output.done = false;
      return output;
    }

   protected:
    double          _trackwidth;
    kinematics_1d_t _limits;
  };

}  // namespace system
}  // namespace grpl