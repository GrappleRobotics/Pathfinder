#pragma once

namespace grpl {

  template <typename UNIT>
  struct vec2D {
    UNIT x, y;

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