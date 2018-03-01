#pragma once

#include "grpl/units.h"
#include <cmath>

namespace grpl {

  template <typename UNIT>
  struct vec2D {
    UNIT x, y;

    UNIT mag() const {
      return static_cast<UNIT>(sqrt(static_cast<double>(x*x + y*y)));
    }

    units::Angle angle() const {
      return atan2(static_cast<double>(y), static_cast<double>(x)) * units::rad;
    }

    static vec2D &make_polar(UNIT magnitude, units::Angle angle) {
      return vec2D { magnitude * cos(angle.as(units::rad)), magnitude * sin(angle.as(units::rad)) };
    }

    vec2D &operator=(const vec2D &equal_to) {
      x = equal_to.x;
      y = equal_to.y;
      return *this;
    }

    vec2D &operator+=(const vec2D &other) {
      x += other.x;
      y += other.y;
      return *this;
    }

    vec2D &operator-=(const vec2D &other) {
      x -= other.x;
      y -= other.y;
      return *this;
    }

    vec2D operator+(const vec2D &other) const {
      return { x + other.x, y + other.y };
    }

    vec2D operator-(const vec2D &other) const {
      return { x - other.x, y - other.y };
    }
  };

  template<typename UNIT>
  const vec2D<UNIT> operator*(double scalar, const vec2D<UNIT> &v) {
    return { v.x * scalar, v.y * scalar };
  }

  template<typename UNIT>
  const vec2D<UNIT> operator/(double scalar, const vec2D<UNIT> &v) {
    return { scalar / v.x, scalar / v.y };
  }

  template<typename UNIT>
  const vec2D<UNIT> operator*(const vec2D<UNIT> &v, double scalar) {
    return { v.x * scalar, v.y * scalar };
  }

  template<typename UNIT>
  const vec2D<UNIT> operator/(const vec2D<UNIT> &v, double scalar) {
    return { v.x / scalar, v.y / scalar };
  }
}