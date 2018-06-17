#pragma once

#include "grpl/curve/curve.h"
#include "grpl/path/path.h"

namespace grpl {
namespace param {

  template <size_t DIM, typename curve_t>
  class parameterizer {
   public:
    using vector_t = typename Eigen::Matrix<double, DIM, 1>;

    static const size_t DIMENSIONS = DIM;

    parameterizer() { set_path(nullptr); }
    parameterizer(grpl::path::path<DIM> *path) { set_path(path); }

    inline void set_path(grpl::path::path<DIM> *path) { _path = path; }
    inline bool has_path() const { return _path != nullptr; }

    inline grpl::path::path<DIM> *get_path() const { return _path; }

    virtual size_t parameterize(curve_t *curves, size_t max_curves) = 0;

   private:
    grpl::path::path<DIM> *_path = nullptr;
  };

}  // namespace param
}  // namespace grpl