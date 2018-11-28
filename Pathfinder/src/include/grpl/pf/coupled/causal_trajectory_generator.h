#pragma once

#include "chassis.h"
#include "grpl/pf/path/curve.h"
#include "grpl/pf/profile/profile.h"
#include "state.h"

namespace grpl {
namespace pf {
  namespace coupled {

    /**
     * Causal trajectory generator, given the chassis, path and motion profile.
     *
     * This class provides the necessary plumbing in order to generate trajectories from a given path,
     * applying motion and timing information based on the limits and constraints of a drivetrain. The
     * generator applies a motion profile to a path, while fitting within given constraints.
     *
     * This generator is causal and will generate a single state (output) with knowledge only of the current
     * state of the system. This generator can be used on-the-fly, and is primarily advantageous in speed and
     * flexibility. The cost is accuracy towards the end of the path, as a sudden deceleration may be required
     * with insufficient time steps, as the end velocity may not be zero.
     *
     * It is recommended to use this generator if on-the-fly generation is required, or memory and
     * computational resources are sparse. This generator also enables the ability to provide feedback
     * information on chassis kinematics to the next generation step, making it possible to embed a feedback
     * loop into the generation phase.
     */
    class causal_trajectory_generator {
     public:
      using vector_t = Eigen::Vector2d;

      // TODO: Rename this class to something more fitting (drivetrain is a synonym for chassis). This
      // class can also be split apart to migrate most calculation to chassis, while keeping only the
      // parts required to follow a causal path within this class.

      /**
       * Generate the next state of the trajectory given the current state.
       * 
       * The curves in this system define the path that will be followed.
       *
       * @param chassis       The coupled chassis, used to provide limits for the trajectory the trajectory
       *                      kinematics.
       * @param curve_begin   Iterator pointing to the start of the curve collection. See @ref std::iterator.
       * @param curve_end     Iterator pointing to the end of the curve collection. See @ref std::iterator.
       * @param profile       Reference to the profile to use. The profile must, at a minimum, provide
       *                      continous, bounded velocity and bounded acceleration.
       *                      @ref grpl::pf::profile::trapezoidal is recommended.
       * @param last          The current ("last") state of the trajectory. If this is the first call to
       *                      generate, this may be considered the "initial conditions".
       * @param time          The time of the next point (last.time + dt), in seconds.
       */
      template <typename iterator_curve_t>
      state generate(chassis &chassis, const iterator_curve_t curve_begin, const iterator_curve_t curve_end,
                     profile::profile &profile, state &last, double time) {
        path::curve<2> *curve;
        state           output;
        double          total_length, curve_distance;
        double          distance = last.kinematics[0];

        curve = find_curve(distance, curve_begin, curve_end, curve_distance, total_length);

        // TODO: The epsilon of the profile causes this to never advance, meaning the path
        // is never marked as 'finished' on some timesteps.
        if (curve == nullptr) {
          output          = last;
          output.finished = true;
          return output;
        }

        profile.set_goal(total_length);

        vector_t centre     = curve->position(curve_distance);
        vector_t centre_rot = curve->rotation(curve_distance);
        double   curvature  = curve->curvature(curve_distance);
        double   dcurvature = curve->dcurvature(curve_distance);

        double              heading = atan2(centre_rot.y(), centre_rot.x());
        configuration_state config{centre.x(), centre.y(), heading};

        output.time   = time;
        output.config = config;

        // TODO: Allow multiple constraints (like current limits)
        // TODO: Enforce minimum acceleration constraints in profiles.
        // TODO: Does limiting jerk prevent oscillation
        double                    limit_vel = chassis.linear_vel_limit(config, curvature);
        std::pair<double, double> limit_acc =
            chassis.acceleration_limits(config, curvature, last.kinematics[1]);

        profile.apply_limit(1, -limit_vel, limit_vel);
        profile.apply_limit(2, limit_acc.first, limit_acc.second);

        profile::state prof_state;
        prof_state.time       = last.time;
        prof_state.kinematics = last.kinematics;

        prof_state = profile.calculate(prof_state, time);

        output.kinematics = prof_state.kinematics;
        output.curvature  = curvature;
        output.dcurvature = dcurvature;

        output.finished = false;
        return output;
      }

     private:
      // TODO: We can store the last known curve to make lookup faster, but will cause random-access slowdown.
      // That's a fixable problem with a settable parameter to optimize for either sequential or random
      // access.
      template <typename iterator_curve_t>
      inline path::curve<2> *find_curve(double targ_len, const iterator_curve_t curve_begin,
                                        const iterator_curve_t curve_end, double &curve_len_out,
                                        double &total_len_out) {
        path::curve<2> *curve_out = nullptr;
        curve_len_out             = targ_len;
        total_len_out             = 0;

        for (iterator_curve_t it = curve_begin; it != curve_end; it++) {
          path::curve<2> &curr = *it;

          double len = curr.length();
          // If we haven't found a curve, and the current length of the curve will put us ahead
          // of our distance target.
          if (curve_out == nullptr && (len + total_len_out) >= targ_len) {
            curve_len_out = targ_len - total_len_out;
            curve_out     = &curr;
          }
          total_len_out += len;
        }
        return curve_out;
      }
    };
  }  // namespace coupled
}  // namespace pf
}  // namespace grpl