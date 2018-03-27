#pragma once

#include <blaze/Math.h>

namespace grpl {
  blaze::StaticVector<double, 2> vec_polar(double mag, double angle) {
    return blaze::StaticVector<double, 2>{ mag * cos(angle), mag * sin(angle) };
  }

  blaze::StaticVector<double, 2> vec_cart(double x, double y) {
    return blaze::StaticVector<double, 2>{ x, y };
  }
}