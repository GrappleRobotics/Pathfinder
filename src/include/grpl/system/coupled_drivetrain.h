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

    void apply_limits(int derivative_idx, double maximum) { _limits[derivative_idx] = maximum; }
    vec_t &get_limits() { return _limits; }

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
      bool done = false;
      double path_len = path->get_arc_length();

      double cur_distance = last.c.k[0];
      if (cur_distance >= path_len)
        done = true;
      
      double path_progress = (cur_distance / path_len);
      vector_t center = path->calculate(path_progress);
      vector_t center_prime = path->calculate_slope(path_progress);

      double dt = time - last.c.t;
      bool first = (dt < 0.00000001); // to avoid dt = 0 errors

      double angle = atan2(center_prime[1], center_prime[0]);
      double d_angle = atan2(sin(angle - last.a[0]), cos(angle - last.a[0]));

      double angular_velocity = first ? 0 : d_angle / dt;
      double angular_acceleration = first ? 0 : (angular_velocity - last.a[1]) / dt;

      // TODO: Expand this to N order

      double differential_accel = angular_acceleration * _trackwidth / 2.0;
      double max_accel = _limits[2] - abs(differential_accel);

      double differential_vel = angular_velocity * _trackwidth / 2.0;
      double max_vel = _limits[1] - abs(differential_vel);

      profile_t::segment_t segment;
      segment.time = last.c.t;
      for (size_t i = 0; i < profile_t::ORDER; i++) {
        segment.vect[i] = last.c.k[i];
      }

      profile->set_goal(path_len);
      segment = profile->calculate(segment, time);

      state output;
      output.a[0] = angle;
      output.a[1] = angular_velocity;
      output.a[2] = angular_acceleration;
      output.c.t = output.l.t = output.r.t = time;

      output.c.k = segment.vect;
      output.c.p = center;

      // TODO: Expand this to N order

      double lcoeff = angular_velocity <= 0 ? 1 : -1;
      output.l.k[1] = output.c.k[1] + lcoeff * differential_vel;
      output.r.k[1] = output.c.k[1] - lcoeff * differential_vel;

      // TODO: Can we trust this, with gain scheduling on the profile?
      // we should have a test for the kinematics here
      lcoeff = angular_acceleration <= 0 ? 1 : -1;
      output.l.k[2] = output.c.k[2] + lcoeff * differential_accel;
      output.r.k[2] = output.c.k[2] - lcoeff * differential_accel;

      //     out->left.acc = (out->left.vel - last->left.vel) / dt;
      //     out->right.acc = (out->right.vel - last->right.vel) / dt;

      double sina = sin(angle), cosa = cos(angle);
      vector_t angle_modifiers { _trackwidth / 2.0 * sina, -_trackwidth / 2.0 * cosa };

      output.l.p = output.c.p - angle_modifiers;
      output.r.p = output.c.p + angle_modifiers;

      output.l.k[0] = last.l.k[0] + output.l.k[1] * dt;
      output.r.k[0] = last.r.k[0] + output.r.k[1] * dt;

      output.done = done;
      return output;
    }

  protected:
    double _trackwidth;
    vec_t _limits;
  };

  // class coupled_drivetrain {
  // public:
  //   using profile_t = profile::profile<units::Distance, units::Time, units::Velocity, units::Acceleration>;
  //   using vec_t = vec2D<units::Distance>;
  //   using path_t = path::path<vec_t, units::Distance>;

  //   struct coupled_side_point {
  //     vec_t pos;
  //     units::Distance dist;
  //     units::Velocity vel;
  //     units::Acceleration acc;
  //   };

  //   struct point {
  //     units::Angle angle;
  //     units::AngularVelocity angular_velocity;
  //     units::Time time;
  //     coupled_side_point left, center, right;
  //   };

  //   coupled_drivetrain(profile_t *profile, path_t *path, units::Distance trackwidth, units::Velocity vel_max, units::Acceleration acc_max)
  //     : _profile(profile), _path(path), _trackwidth(trackwidth), _vel_max(vel_max), _acc_max(acc_max) { }

  //   bool generate(units::Distance spline_len, point *out, point *last, units::Time time) {
  //     point cache;
  //     bool done = false;
  //     if (last != nullptr) {
  //       cache = { last->angle, last->angular_velocity, last->time, last->left, last->center, last->right };
  //     }

  //     last = &cache;

  //     units::Distance cur_distance = last->center.dist;
  //     if (cur_distance >= spline_len) {
  //       done = true;
  //     }

  //     double spline_progress = (cur_distance / spline_len).scalar();  // unitless
  //     vec_t center = _path->calculate(spline_progress);
  //     vec_t center_slope = _path->calculate_slope(spline_progress);

  //     units::Time dt = time - last->time;
  //     bool first = (time == last->time);

  //     units::Angle center_angle = center_slope.angle();
  //     units::AngularVelocity angular_velocity = first ? 0 : (center_angle - last->angle) / dt;
  //     units::AngularAcceleration angular_acceleration = first ? 0 : (angular_velocity - last->angular_velocity) / dt;

  //     units::Acceleration differential_acceleration = angular_acceleration * _trackwidth / 2.0;
  //     units::Acceleration center_max_accel = _acc_max - static_cast<units::Acceleration>(abs(differential_acceleration.scalar()));

  //     units::Velocity differential_velocity = angular_velocity * _trackwidth / 2.0;
  //     units::Velocity center_max_velocity = _vel_max - static_cast<units::Velocity>(abs(differential_velocity.scalar()));

  //     profile_t::segment tmp;
  //     tmp.time = last->time;
  //     tmp.dist = last->center.dist;
  //     tmp.vel = last->center.vel;
  //     tmp.acc = last->center.acc;

  //     _profile->max_velocity(respect_velocity_constraint ? center_max_velocity : _vel_max);
  //     _profile->max_acceleration(respect_acceleration_constraint ? center_max_accel : _acc_max);
  //     _profile->goal(spline_len);
  //     _profile->calculate(&tmp, &tmp, time);

  //     out->time = tmp.time;
  //     out->angle = center_angle;
  //     out->angular_velocity = angular_velocity;

  //     out->center.pos = center;
  //     out->center.dist = tmp.dist;
  //     out->center.vel = tmp.vel;
  //     out->center.acc = tmp.acc;

  //     double lcoeff = angular_velocity.scalar() <= 0 ? 1 : -1;
  //     out->left.vel = tmp.vel + lcoeff * differential_velocity;
  //     out->right.vel = tmp.vel - lcoeff * differential_velocity;

  //     out->left.acc = (out->left.vel - last->left.vel) / dt;
  //     out->right.acc = (out->right.vel - last->right.vel) / dt;

  //     auto sina = sin(out->angle.as(units::rad)), cosa = cos(out->angle.as(units::rad));

  //     out->left.pos = { out->center.pos.x - (_trackwidth / 2.0 * sina), out->center.pos.y + (_trackwidth / 2.0 * cosa) };
  //     out->right.pos = { out->center.pos.x + (_trackwidth / 2.0 * sina), out->center.pos.y - (_trackwidth / 2.0 * cosa) };

  //     out->left.dist = last->left.dist + out->left.vel * dt;
  //     out->right.dist = last->left.dist + out->right.vel * dt;

  //     return done;
  //   }

  //   bool respect_velocity_constraint = true, respect_acceleration_constraint = true;

  // protected:
  //   profile_t *_profile;
  //   path_t *_path;
  //   units::Distance _trackwidth;
  //   units::Velocity _vel_max;
  //   units::Acceleration _acc_max;
  // };

}
}