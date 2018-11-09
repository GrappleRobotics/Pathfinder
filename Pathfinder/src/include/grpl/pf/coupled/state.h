#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  /**
   * @brief
   * Coupled / tank / differential drivetrain models.
   *
   * The grpl::pf::coupled namespace contains all models and other structures for use
   * with coupled-style drivetrains.
   *
   * A coupled-style drivetrain defines a drivetrain with individually controlled, parallel
   * left and right sides, also known as a tank drive or differential drive. May also refer
   * to skid-steer systems.
   */
  namespace coupled {

    /**
     * @brief
     * Drivetrain configuration, describing the position of the chassis.
     * 
     * @param x         The x position of the centre of the drivetrain, in metres.
     * @param y         The y position of the centre of the drivetrain, in metres.
     * @param heading   The heading of the drivetrain, in radians.
     */
    using configuration = Eigen::Vector3d;

    /**
     * @brief
     * Drivetrain kinematics, describing the movement and motion of the chassis.
     * 
     * @param distance      The distance covered by the drivetrain, in metres.
     * @param velocity      The linear velocity of the drivetrain, in metres per second (ms^-1).
     * @param acceleration  The linear acceleration of the drivetrain, in metres per second
     *                      per second (ms^-2).
     */
    using kinematics = Eigen::Vector3d;

    /**
     * @brief
     * The state of a coupled drivetrain at any point in time, as a single state within a 
     * trajectory.
     */
    struct state {
      //! The time point of this state, in seconds.
      double        time      = 0;
      //! The instantaneous curvature of the state, in m^-1
      double        curvature = 0;
      //! The instantaneous change in curvature of the state, in (ms)^-1
      double        dcurvature = 0;
      //! The configuration of the chassis at the time of the state.
      configuration configuration = configuration::Zero();
      //! The kinematics of the chassis at the time of the state.
      kinematics    kinematics    = kinematics::Zero();
      bool          finished      = false;
    };

    /**
     * @brief
     * The state of a wheel (side) of the coupled drivetrain at any point in time, primarily
     * for use with encoders / other following regimes.
     */
    struct wheel_state {
      /**
       * @brief
       * Position vector of the wheel
       * 
       * @param x The x position of the wheel, in metres.
       * @param y The y position of the wheel, in metres.
       */
      using vector_t = Eigen::Vector2d;

      //! The time point of this state, in seconds.
      double      time       = 0;
      //! The position of the wheel at the time of the state.
      vector_t    position   = vector_t::Zero();
      //! The kinematics of the wheel at the time of the state. Note this is linear, not rotational.
      kinematics  kinematics = kinematics::Zero();
      //! The voltage applied to the transmission connected to this wheel, in Volts
      double      voltage = 0;
      //! The current drawn by the transmission connected to this wheel, in Amperes.
      double      current = 0;
      bool        finished = false;
    };
  }  // namespace coupled
}  // namespace pf
}  // namespace grpl