#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  namespace coupled {
    // Robot Configuration:
    //    x, y, heading
    using configuration_t = Eigen::Vector3d;

    // Motion Information:
    //    distance, velocity, acceleration
    using kinematics_t = Eigen::Vector3d;

    struct state {
      double          time      = 0;
      double          curvature = 0, dcurvature = 0;
      configuration_t configuration{0, 0, 0};
      kinematics_t    kinematics{0, 0, 0};
      bool            finished = false;
    };

    struct wheel_state {
      using vector_t = Eigen::Vector2d;

      double       time = 0;
      vector_t     position{0, 0};
      kinematics_t kinematics{0, 0, 0};
      double       voltage = 0, current = 0;
      bool         finished = false;
    };
  }  // namespace coupled
}  // namespace pf
}  // namespace grpl