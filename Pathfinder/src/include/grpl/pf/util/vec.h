#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace pf {
  static Eigen::Vector2d polar(double mag, double angle) {
    return Eigen::Vector2d{mag * cos(angle), mag * sin(angle)};
  }

  static Eigen::Vector2d cartesian(double x, double y) { return Eigen::Vector2d{x, y}; }
}  // namespace pf
}  // namespace grpl