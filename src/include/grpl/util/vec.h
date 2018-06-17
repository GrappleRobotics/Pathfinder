#pragma once

#include <Eigen/Dense>

namespace grpl {

Eigen::Vector2d vec_polar(double mag, double angle) {
  return Eigen::Vector2d{mag * cos(angle), mag * sin(angle)};
}

Eigen::Vector2d vec_cart(double x, double y) {
  return Eigen::Vector2d{x, y};
}

}  // namespace grpl