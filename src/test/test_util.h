#pragma once

#include <Eigen/Dense>

#include <iostream>

namespace testutil {
template <size_t DIM, size_t ORDER>
class pose_simulation {
 public:
  using v_t = Eigen::Matrix<double, DIM, 1>;
  void write(v_t value, size_t order, double dt) {
    for (size_t i = 0; i < order; i++) _oldPose[i] = _pose[i];

    _pose[order] = value;
    for (size_t divLow = 1; divLow < order; divLow++) {
      size_t o = order - divLow;
      _pose[o] += _pose[o + 1] * dt;
    }

    for (size_t divHi = 1; divHi < (ORDER - order); divHi++) {
      size_t o = order + divHi;
      _pose[o] = (_pose[o - 1] - _oldPose[o - 1]) / dt;
    }
  }

  void zero() {
    for (size_t i = 0; i < ORDER; i++) _oldPose[i] = _pose[i] = {0, 0};
  }

  v_t get(size_t order) { return _pose[order]; }

 private:
  v_t _pose[ORDER], _oldPose[ORDER];
};
}  // namespace testutil