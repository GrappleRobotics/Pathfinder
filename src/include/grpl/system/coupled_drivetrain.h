#pragma once

#include "grpl/profile/profile.h"
#include "grpl/path/path.h"
#include "grpl/vec.h"

#include <cmath>

namespace grpl {
namespace system {

  class coupled_drivetrain {
  public:
    using profile_t = profile::profile<units::Distance, units::Time, units::Velocity, units::Acceleration>;
    using vec_t = vec2D<units::Distance>;
    using path_t = path::path<vec_t, units::Distance>;

    struct coupled_side_point {
      vec_t pos;
      units::Distance dist;
      units::Velocity vel;
      units::Acceleration acc;
    };

    struct point {
      units::Angle angle;
      units::AngularVelocity angular_velocity;
      units::Time time;
      coupled_side_point left, center, right;
    };

    coupled_drivetrain(profile_t *profile, path_t *path, units::Distance trackwidth, units::Velocity vel_max)
      : _profile(profile), _path(path), _trackwidth(trackwidth), _vel_max(vel_max) { }

    bool generate(units::Distance spline_len, point *out, point *last, units::Time time) {
      point cache;
      if (last != nullptr) {
        cache = { last->angle, last->angular_velocity, last->time, last->left, last->center, last->right };
      }

      last = &cache;

      units::Distance cur_distance = last->center.dist;
      if (cur_distance >= spline_len) {
        return true;
      }

      double spline_progress = (cur_distance / spline_len).scalar();  // unitless
      vec_t center = _path->calculate(spline_progress);
      vec_t center_slope = _path->calculate_slope(spline_progress);

      units::Time dt = time - last->time;
      units::Angle center_angle = center_slope.angle();
      units::AngularVelocity angular_velocity = (center_angle - last->angle) / dt;
      units::Velocity differential_velocity = angular_velocity * _trackwidth / 2.0;
      units::Velocity center_max_velocity = _vel_max - differential_velocity;

      profile_t::segment tmp;
      tmp.time = last->time;
      tmp.dist = last->center.dist;
      tmp.vel = last->center.vel;
      tmp.acc = last->center.acc;

      _profile->max_velocity(center_max_velocity);
      _profile->goal(spline_len);
      _profile->calculate(&tmp, &tmp, time);

      out->time = tmp.time;
      out->angle = center_angle;
      out->angular_velocity = angular_velocity;

      out->center.pos = center;
      out->center.dist = tmp.dist;
      out->center.vel = tmp.vel;
      out->center.acc = tmp.acc;

      double lcoeff = angular_velocity.scalar() <= 0 ? 1 : -1;
      out->left.vel = tmp.vel + lcoeff * differential_velocity;
      out->right.vel = tmp.vel - lcoeff * differential_velocity;

      out->left.acc = (out->left.vel - last->left.vel) / dt;
      out->right.acc = (out->right.vel - last->right.vel) / dt;

      auto sina = sin(out->angle.as(units::rad)), cosa = cos(out->angle.as(units::rad));

      out->left.pos = { out->center.pos.x - (_trackwidth / 2.0 * sina), out->center.pos.y + (_trackwidth / 2.0 * cosa) };
      out->right.pos = { out->center.pos.x + (_trackwidth / 2.0 * sina), out->center.pos.y - (_trackwidth / 2.0 * cosa) };

      out->left.dist = last->left.dist + out->left.vel * dt;
      out->right.dist = last->left.dist + out->right.vel * dt;

      return false;
    }

  protected:
    profile_t *_profile;
    path_t *_path;
    units::Distance _trackwidth;
    units::Velocity _vel_max;
  };

}
}