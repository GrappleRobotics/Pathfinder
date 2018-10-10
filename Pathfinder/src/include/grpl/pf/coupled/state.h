#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  namespace coupled {
    // Robot Configuration:
    //    x, y, heading
    using configuration = Eigen::Vector3d;

    // Motion Information:
    //    distance, velocity, acceleration
    using kinematics = Eigen::Vector3d;

    struct state {
      double        time      = 0;
      double        curvature = 0, dcurvature = 0;
      configuration configuration = configuration::Zero();
      kinematics    kinematics    = kinematics::Zero();
      bool          finished      = false;
    };

    struct wheel_state {
      using vector_t = Eigen::Vector2d;

      double     time       = 0;
      vector_t   position   = vector_t::Zero();
      kinematics kinematics = kinematics::Zero();
      double     voltage = 0, current = 0;
      bool       finished = false;
    };
  }  // namespace coupled
}  // namespace pf
}  // namespace grpl