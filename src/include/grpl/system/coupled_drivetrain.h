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
      bool done;
    };

    void calculate_angle_derivatives(double dt, double angle, kinematics_1d_t &angles,
                                     kinematics_1d_t &last_angles) {
      angles[0] = angle;
      for (size_t i = 1; i < profile_t::ORDER; i++) {
        double diff = angles[i - 1] - last_angles[i - 1];
        angles[i] = (dt < 0.0001) ? 0 : diff / dt;
      }
    }

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
      double angle = atan2(center_slope[1], center_slope[0]);
      vector_t unit_heading = vec_polar(1, angle);

      calculate_angle_derivatives(dt, angle, output.a, last.a);

      // Half of difference between left and right side for each of { vel, acc, etc }
      kinematics_1d_t differentials = output.a * trackradius;
      kinematics_1d_t new_limits = _limits - differentials.cwiseAbs();

      profile->set_goal(path_len);
      profile->set_limits(new_limits);

      typename profile_t::segment_t segment;
      segment.time = last.c.t;
      for (size_t i = 0; i < profile_t::ORDER; i++) {
        segment.k[i] = last.c.k.col(i).dot(unit_heading);
      }
      segment.k[0] = cur_distance;
      segment = profile->calculate(segment, time);

      output.c.t = output.l.t = output.r.t = time;

      output.c.k = unit_heading * segment.k;
      output.c.d = segment.k[0];

      // Center kinematics + circular kinematics
      // output.l.k = output.c.k - tw_angle * output.a;
      // output.r.k = output.c.k + tw_angle * output.a;
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

      if (isFirst) {
        output.l.k.col(1) = vec_cart(0, 0);
        output.c.k.col(1) = vec_cart(0, 0);
        output.r.k.col(1) = vec_cart(0, 0);
      } else {
        output.l.k.col(1) = (output.l.k.col(0) - last.l.k.col(0)) / dt;
        output.r.k.col(1) = (output.r.k.col(0) - last.r.k.col(0)) / dt;
      }

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