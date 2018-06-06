#pragma once

#include "grpl/path/path.h"
#include "grpl/profile/profile.h"
#include "grpl/util/constants.h"
#include "grpl/util/vec.h"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

namespace grpl {
namespace system {

  template <typename path_t, typename profile_t>
  class coupled_drivetrain {
   public:
    static_assert(path_t::DIMENSIONS == 2,
                  "Path must function in exactly 2 Dimensions for Coupled Drivetrain!");
    using kinematics_t = Eigen::Matrix<double, path_t::DIMENSIONS, profile_t::ORDER>;
    using kinematics_1d_t = typename profile_t::kinematics_1d_t;
    using vector_t = typename path_t::vector_t;

    void apply_limit(int derivative_idx, double maximum) {
      _limits[derivative_idx] = maximum;
    }

    kinematics_1d_t &get_limits() { return _limits; }
    void set_limits(kinematics_1d_t &other) { _limits = other; }

    void set_trackwidth(double tw) { _trackwidth = tw; }
    double get_trackwidth() const { return _trackwidth; }

    struct coupled_side_t {
      double t;
      double d;
      kinematics_t k;
    };

    struct state {
      coupled_side_t l, c, r;
      kinematics_1d_t a;
      double curvature;
      bool done;
    };

    state generate(path_t *path, profile_t *profile, state &last, double time) {
      state output;
      double path_len = path->get_arc_length();

      double cur_distance = last.c.d;
      if (cur_distance >= path_len) {
        output = last;
        output.done = true;
        return output;
      }

      double path_progress = (cur_distance / path_len);
      double trackradius = _trackwidth / 2.0;
      double dt = time - last.c.t;
      bool isFirst = (dt < 0.0001);

      vector_t center = path->calculate(path_progress);
      vector_t center_slope = path->calculate_slope(path_progress);
      double curvature = path->calculate_curvature(path_progress);
      double angle = atan2(center_slope[1], center_slope[0]);
      vector_t unit_heading = vec_polar(1, angle);

      // Determine angle and derivatives
      output.a[0] = angle;
      for (size_t i = 1; i < profile_t::ORDER; i++) {
        double diff = output.a[i - 1] - last.a[i - 1];
        output.a[i] = (isFirst) ? 0 : diff / dt;
      }

      // Set the new limits on the velocity profile
      // Half of difference between left and right side for each of { vel, acc, etc }
      kinematics_1d_t differentials = output.a * trackradius;
      kinematics_1d_t new_limits = _limits - differentials.cwiseAbs();

      profile->set_goal(path_len);
      profile->set_limits(_limits);

      // Calculate velocity profile
      typename profile_t::segment_t segment;
      segment.time = last.c.t;
      for (size_t i = 0; i < profile_t::ORDER; i++) {
        segment.k[i] = last.c.k.col(i).dot(unit_heading);   // I think the error is here (?)
      }
      segment.k[0] = cur_distance;
      segment = profile->calculate(segment, time);

      // Set output values
      output.curvature = curvature;
      output.c.t = output.l.t = output.r.t = time;

      output.c.k = unit_heading * segment.k;
      output.c.d = segment.k[0];

      // Progress: profile isn't respecting max acceleration as shown by delta of 
      // position. This means either the profile is wrong (probs not), or the spline
      // length calculation / reference isn't working properly (more likely).
      // How's our parameterization?
      // This is probably because of the non-equidistant points in the path (not parameterized to length)

      // Center kinematics + circular kinematics
      // output.l.k = output.c.k - (trackradius * unit_heading) * output.a;
      // output.r.k = output.c.k + (trackradius * unit_heading) * output.a;
      // output.l.k = output.c.k;
      // output.r.k = output.c.k;
      // output.l.k.col(1) = vec_polar((output.c.k.col(1).dot(unit_heading) -
      // output.a[1]*(_trackwidth/2.0)), angle); output.r.k.col(1) =
      // vec_polar((output.c.k.col(1).dot(unit_heading) +
      // output.a[1]*(_trackwidth/2.0)), angle); std::cout << time << "- " <<

      // ((output.c.k.col(1).dot(unit_heading))) << " " <<
      // (output.a[1]*(_trackwidth/2.0)) << std::endl;

      output.c.k.col(0) = center;
      output.l.k.col(0) =
          output.c.k.col(0) - vec_polar(trackradius, output.a[0] - PI / 2.0);
      output.r.k.col(0) =
          output.c.k.col(0) + vec_polar(trackradius, output.a[0] - PI / 2.0);

      // TODO: I suspect the issue has to do with acceleration being in the wrong direction and not accounting for
      // the 'pull in' effect

      double radius = 1.0 / curvature;
      // double acc_in = (output.c.k.col(1).squaredNorm())/radius;
      // acc_in *= (output.a[1] > 0 ? 1 : -1);
      // output.c.k.col(2) += vec_polar(acc_in, output.a[0] + PI / 2.0);

      // double acc_in = (output.c.k.col(1).squaredNorm())/trackradius;

      double acc_in = output.a[1]*output.a[1]*radius;
      output.c.k.col(2) += vec_polar(acc_in, output.a[0] + (output.a[1] > 0 ? 1 : -1) * PI / 2.0);

      // Test new curvature-related derivation
      // double curvature = path->calculate_curvature(path_progress);
      // double radius = 1.0 / curvature;

      // vector_t vel_center = output.c.k.col(1);
      // vector_t vel_outer = vel_center * (1 + trackradius / radius);
      // vector_t vel_inner = vel_center * (1 - trackradius / radius);

      // if (output.a[1] > 0) {
      //   // right = outer
      //   output.r.k.col(1) = vel_outer;
      //   output.l.k.col(1) = vel_inner;
      // } else {
      //   // left = outer
      //   output.r.k.col(1) = vel_inner;
      //   output.l.k.col(1) = vel_outer;
      // }

      // if (isFirst) {
      //   output.l.k.col(1) = vec_cart(0, 0);
      //   output.c.k.col(1) = vec_cart(0, 0);
      //   output.r.k.col(1) = vec_cart(0, 0);
      // } else {
      //   output.l.k.col(1) = (output.l.k.col(0) - last.l.k.col(0)) / dt;
      //   // output.c.k.col(1) = (output.c.k.col(0) - last.c.k.col(0)) / dt;
      //   output.r.k.col(1) = (output.r.k.col(0) - last.r.k.col(0)) / dt;
      // }

      output.l.d = last.l.d + output.l.k.col(1).dot(unit_heading) * dt;
      output.r.d = last.r.d + output.r.k.col(1).dot(unit_heading) * dt;

      output.done = false;
      return output;
    }

   protected:
    double _trackwidth;
    kinematics_1d_t _limits;
  };

}  // namespace system
}  // namespace grpl