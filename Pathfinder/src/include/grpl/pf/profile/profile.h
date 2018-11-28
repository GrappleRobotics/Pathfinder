#pragma once

#include <Eigen/Dense>
#include "grpl/pf/constants.h"

namespace grpl {
namespace pf {
  /**
   * Motion profiles
   *
   * The grpl::pf::profile namespace contains all motion profile implementations.
   * Motion profiles are used to describe the shape of the position-time curve to the nth
   * degree.
   */
  namespace profile {

    using kinematic_state = Eigen::Matrix<double, 1, constants::profile_kinematics_order>;

    /**
     * A single state (sample point) of a motion profile. This contains the kinematics of
     * the profile sampled at a given time.
     *
     * The state kinematics goes up to a maximum order of @ref grpl::pf::constants::profile_kinematics_order,
     * although implementations of @ref grpl::pf::profile::profile may or may not fill the entire vector,
     * depending on their operating order (e.g. @ref grpl::pf::profile::trapezoidal will only fill up to
     * @ref grpl::pf::ACCELERATION, all higher orders will be an undefined value).
     */
    struct state {
      //! The time point of this state, in seconds
      double time = 0;
      //! The kinematic state of the system at the time of the state, filled to the maximum order of
      //! the profile. All higher orders will be of an undefined value.
      kinematic_state kinematics = kinematic_state::Zero();
    };

    /**
     * Abstract base class for all motion profile types.
     *
     * A motion profile describes some function that forms the shape of the position-time curve
     * and its derivatives, allowing velocity, acceleration and jerk to be shaped in ways that may
     * be desirable, such as optimizing for speed, safety, or smooth operation.
     *
     * Profiles are predictive in Pathfinder, meaning each calculation does not require a full history
     * of the profile. This allows it to adjust to changing system conditions and limits during a
     * profile, increasing flexibility and efficiency.
     *
     * Since the system is predictive, it may result in a small oscillation or sudden deceleration
     * if a sufficient timestep is not used. For this reason, a timeslice mechanism is included in the
     * profile.
     */
    class profile {
     public:
      using limits_t = Eigen::Matrix<double, 2, constants::profile_limits_order>;

      virtual ~profile() {}

      /**
       * Get the index of the limited term (the highest order, non-infinite term). See constants in
       * @ref grpl::pf
       */
      virtual const size_t limited_term() const = 0;

      /**
       * Set the goal (setpoint) of the profile.
       *
       * @param sp The goal (setpoint) of the profile, in metres.
       */
      void set_goal(double sp) { _goal = sp; }

      /**
       * Get the goal (setpoint) of the profile.
       *
       * @return The goal (setpoint) of the profile, in metres.
       */
      double get_goal() const { return _goal; }

      // TODO: Abstract timeslice?

      /**
       * Set the timeslice period.
       *
       * The timeslice is used in @ref calculate(state&, double), where a single
       * call will be translated into N smaller calls, where N is ceil(T_calc / T_slice), where T_calc is the
       * period at which @ref calculate(state&, double) is called, and T_slice is the timeslice period.
       *
       * Timeslice is used to counteract slow calls to @ref calculate(state&, double) in order to still
       * produce smooth curves.
       *
       * @param timeslice The timeslice period T_slice, in seconds.
       */
      void set_timeslice(double timeslice) { _timeslice = timeslice; }

      /**
       * Get the timeslice period.
       *
       * @return The timeslice period, T_slice, in seconds.
       */
      double get_timeslice() const { return _timeslice; }

      /**
       * Apply a constrained limit to the profile. This will limit the maximum and minimum value of
       * this term during the profile (e.g. maximum velocity / acceleration).
       *
       * These limits may be changed between calls to @ref calculate(state&, double), but if the maximum
       * or minimum is lower in magnitude, it may cause sudden decelerations.
       *
       * @param term  The term to apply the limit to. See constants in @ref grpl::pf
       * @param min   The minimum value of the term, in the units of the term
       * @param max   The maximum value of the term, in the units of the term
       */
      void apply_limit(int term, double min, double max) {
        _limits(0, term) = min;
        _limits(1, term) = max;
      }

      /**
       * Obtain the currently set limits of the profile
       *
       * @return The limits matrix. Row 0 is the minimum values, row 1 is the maximum. The column indices
       * match those of the terms (see constants in @ref grpl::pf)
       */
      limits_t get_limits() { return _limits; }

      /**
       * Calculate a single state of the motion profile, in a predictive manner.
       *
       * This method function uses the last (current) state of the system in order to generate the following
       * state at the time given. It does this in a predictive manner, meaning it predicts when it must
       * begin slowing down in order to not overshoot the setpoint. This means the profile calculation does
       * not require a full history of the profile, allowing it to adjust to changing system conditions and
       * limits.
       */
      virtual state calculate(state &last, double time) = 0;

     protected:
      double   _goal, _timeslice = 0.001;
      limits_t _limits = limits_t::Zero();
    };

  }  // namespace profile
}  // namespace pf
}  // namespace grpl