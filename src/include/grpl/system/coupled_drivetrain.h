#pragma once

#include "grpl/profile/profile.h"
#include "grpl/path/path.h"

#include <cmath>

namespace grpl {
namespace system {

  template <typename path_t, typename profile_t>
  class coupled_drivetrain1 {
  public:
    static_assert(path_t::DIMENSIONS == 2, "Path must function in exactly 2 Dimensions for Coupled Drivetrain!");
    using vec_t = typename profile_t::vec_t;
    using vector_t = typename path_t::vector_t;

    void apply_limit(int derivative_idx, double maximum) { _limits[derivative_idx] = maximum; }
    vec_t &get_limits() { return _limits; }
    void set_limits(vec_t &other) { _limits = other; }

    void set_trackwidth(double tw) { _trackwidth = tw; }
    double get_trackwidth() const { return _trackwidth; }

    struct coupled_side {
      double t;
      vec_t k;
      vector_t p;
    };

    struct state {
      coupled_side l, c, r;
      vec_t a;
      bool done;
    };

    state generate(path_t *path, profile_t *profile, state &last, double time) {
      state output;
      double path_len = path->get_arc_length();

      double cur_distance = last.c.k[0];
      if (cur_distance >= path_len) {
        output = last;
        output.done = true;
        return output;
      }
      
      double path_progress = (cur_distance / path_len);

      vector_t center = path->calculate(path_progress);
      vector_t center_prime = path->calculate_slope(path_progress);

      double dt = time - last.c.t;
      bool first = (dt < 0.00000001); // to avoid dt = 0 errors

      double angle = atan2(center_prime[1], center_prime[0]);

      output.a[0] = angle;
      for (size_t i = 1; i < profile_t::ORDER; i++) {
        double diff = output.a[i-1] - last.a[i-1];
        output.a[i] = first ? 0 : atan2(sin(diff), cos(diff)) / dt;
      }

      vec_t differentials = output.a * _trackwidth / 2.0;
      vec_t new_limits = _limits - abs(differentials);

      typename profile_t::segment_t segment;
      segment.time = last.c.t;
      for (size_t i = 0; i < profile_t::ORDER; i++) {
        segment.vect[i] = last.c.k[i];
      }

      profile->set_goal(path_len);
      profile->set_limits(new_limits);
      segment = profile->calculate(segment, time);

      output.c.t = output.l.t = output.r.t = time;

      output.c.k = segment.vect;
      output.c.p = center;

      vec_t lcoeff = output.a;
      for (size_t i = 0; i < profile_t::ORDER; i++) { 
        lcoeff[i] = lcoeff[i] <= 0 ? 1 : -1;
      }

      output.l.k = output.c.k + lcoeff * differentials;
      output.r.k = output.c.k - lcoeff * differentials;

      double sina = sin(angle), cosa = cos(angle);
      vector_t angle_modifiers { _trackwidth / 2.0 * sina, -_trackwidth / 2.0 * cosa };

      output.l.p = output.c.p - angle_modifiers;
      output.r.p = output.c.p + angle_modifiers;

      output.l.k[0] = last.l.k[0] + output.l.k[1] * dt;
      output.r.k[0] = last.r.k[0] + output.r.k[1] * dt;

      output.done = false;
      return output;
    }

  protected:
    double _trackwidth;
    vec_t _limits;
  };

}
}