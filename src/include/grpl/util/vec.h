#pragma once

#include <Eigen/Dense>

namespace grpl {
  // blaze::StaticVector<double, 2> vec_polar(double mag, double angle) {
  //   return blaze::StaticVector<double, 2>{ mag * cos(angle), mag * sin(angle) };
  // }

  // blaze::StaticVector<double, 2> vec_cart(double x, double y) {
  //   return blaze::StaticVector<double, 2>{ x, y };
  // }
  Eigen::Vector2d vec_polar(double mag, double angle) {
    return Eigen::Vector2d{ mag * cos(angle), mag * sin(angle) };
  }

  Eigen::Vector2d vec_cart(double x, double y) {
    return Eigen::Vector2d{ x, y };
  }
}