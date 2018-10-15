#pragma once

#include "chassis.h"
#include "grpl/pf/profile/profile.h"
#include "state.h"

namespace grpl {
namespace pf {
  namespace coupled {
    class drivetrain {
     public:
      using vector_t = Eigen::Vector2d;

      drivetrain(chassis &chassis_in) : _chassis(chassis_in) {}

      template <typename iterator_curve_t>
      state generate(const iterator_curve_t curve_begin, const iterator_curve_t curve_end,
                     profile::profile &profile, state &last, double time) {
        iterator_curve_t curve;
        state            output;
        double           total_length, curve_distance;
        double           distance = last.kinematics[0];

        bool curve_found = find_curve(distance, curve_begin, curve_end, curve, curve_distance, total_length);

        // TODO: The epsilon of the profile causes this to never advance, meaning the path
        // is never marked as 'finished' on some timesteps.
        if (!curve_found) {
          output          = last;
          output.finished = true;
          return output;
        }

        profile.set_goal(total_length);

        vector_t centre     = curve->position(curve_distance);
        vector_t centre_rot = curve->rotation(curve_distance);
        double   curvature  = curve->curvature(curve_distance);
        double   dcurvature = curve->curvature_prime(curve_distance);

        double        heading = atan2(centre_rot.y(), centre_rot.x());
        configuration config{centre.x(), centre.y(), heading};

        output.time          = time;
        output.configuration = config;

        // TODO: Allow multiple constraints (like current limits)
        // TODO: Enforce minimum acceleration constraints in profiles.
        // TODO: Does limiting jerk prevent oscillation
        double                    limit_vel = _chassis.linear_vel_limit(config, curvature);
        std::pair<double, double> limit_acc =
            _chassis.acceleration_limits(config, curvature, last.kinematics[1]);

        profile.apply_limit(1, -limit_vel, limit_vel);
        profile.apply_limit(2, limit_acc.first, limit_acc.second);

        profile::segment segment;
        segment.time       = last.time;
        segment.kinematics = last.kinematics;

        segment = profile.calculate(segment, time);

        output.kinematics = segment.kinematics;
        output.curvature  = curvature;
        output.dcurvature = dcurvature;

        output.finished = false;
        return output;
      }

      std::pair<wheel_state, wheel_state> split(state centre) {
        wheel_state left, right;

        left.time = right.time = centre.time;
        left.finished = right.finished = centre.finished;

        // Split positions
        vector_t position{centre.configuration.x(), centre.configuration.y()};
        double   heading  = centre.configuration[2];
        vector_t p_offset = vector_t{0, _chassis.track_radius()};

        Eigen::Matrix<double, 2, 2> rotation;
        rotation << cos(heading), -sin(heading), sin(heading), cos(heading);

        // Rotate the wheel offsets by the heading of the robot, adding it to the
        // centre position, this 'splits' the centre path into two paths constrained
        // by the configuration (heading + position) and track radius.
        left.position  = position + rotation * p_offset;
        right.position = position - rotation * p_offset;

        // Split velocities
        double v_linear       = centre.kinematics[1];
        double v_angular      = v_linear * centre.curvature;
        double v_differential = v_angular * _chassis.track_radius();
        left.kinematics[1]    = v_linear - v_differential;
        right.kinematics[1]   = v_linear + v_differential;

        // Split accelerations
        double a_linear = centre.kinematics[2];
        // This is a bit of a tricky one, so don't blink
        // a_angular = dw / dt (where w = v_angular)
        // a_angular = d/dt (v * k) (from v_angular above, w = vk)
        // Then, by product rule:
        //    a_angular = dv/dt * k + v * dk/dt     (note dv/dt is acceleration)
        //    a_angular = a * k + v * dk/dt         [1]
        // We don't have dk/dt, but we do have dk/ds. By chain rule:
        //    dk/dt = dk/ds * ds/dt                 (note ds/dt is velocity)
        //    dk/dt = dk/ds * v                     [2]
        // Therefore, by composing [1] and [2],
        //    a_angular = a * k + v^2 * dk/ds
        // Isn't that just a gorgeous piece of math?
        double a_angular      = a_linear * centre.curvature + v_linear * v_linear * centre.dcurvature;
        double a_differential = a_angular * _chassis.track_radius();
        left.kinematics[2]    = a_linear - a_differential;
        right.kinematics[2]   = a_linear + a_differential;

        _chassis.solve_electrical(left, right);

        return std::pair<wheel_state, wheel_state>{left, right};
      }

     private:
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
      
      chassis _chassis;
    };
  }  // namespace coupled
}  // namespace pf
}  // namespace grpl