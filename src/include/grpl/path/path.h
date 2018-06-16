#pragma once

#include <Eigen/Dense>

namespace grpl {
namespace path {

  template <size_t DIM>
  class path {
   public:
    using vector_t = typename Eigen::Matrix<double, DIM, 1>;  // col vector

    static const size_t DIMENSIONS = DIM;

    virtual vector_t calculate(double t) = 0;
    virtual vector_t calculate_slope(double t) = 0;
    virtual double calculate_curvature(double t) = 0;
    virtual double get_arc_length() = 0;
    virtual void reset_arc_length() = 0;
  };

  template <size_t DIM>
  class path_set : public path<DIM> {
   public:
    using vector_t = typename path<DIM>::vector_t;

    struct path_index {
      size_t index;
      double t;
    };

    virtual path<DIM> *get_path(size_t idx) = 0;
    virtual size_t get_path_count() = 0;

    path_index get_path_idx(double t) {
      double target_arc = get_arc_length() * t;
      double arc_sum = 0;
      path_index idx;
      bool done = false;
      for (size_t i = 0; i < get_path_count() && !done; i++) {
        double al = get_path(i)->get_arc_length();
        if (target_arc <= arc_sum + al) {
          double p = (target_arc - arc_sum) / al;
          idx.index = i;
          idx.t = p;
          done = true;
        }
        arc_sum += al;
      }
      return idx;
    }

    vector_t calculate(double t) override {
      path_index idx = get_path_idx(t);
      return get_path(idx.index)->calculate(idx.t);
    }

    vector_t calculate_slope(double t) override {
      path_index idx = get_path_idx(t);
      return get_path(idx.index)->calculate_slope(idx.t);
    }

    double calculate_curvature(double t) override {
      path_index idx = get_path_idx(t);
      return get_path(idx.index)->calculate_curvature(idx.t);
    }

    double get_arc_length() {
      double total = 0;
      for (size_t i = 0; i < get_path_count(); i++)
        total += get_path(i)->get_arc_length();
      return total;
    }

    void reset_arc_length() {
      for (size_t i = 0; i < get_path_count(); i++) get_path(i)->reset_arc_length();
    }
  };

  template <typename PATHTYPE, size_t COUNT, size_t DIM>
  class static_path_set : public path_set<DIM> {
   public:
    path<DIM> *get_path(size_t idx) override { return _paths[idx]; }

    size_t get_path_count() override { return COUNT; }

    void set_path(size_t idx, PATHTYPE &path) { _paths[idx] = path; }

   protected:
    PATHTYPE _paths[COUNT];
  };

  template <typename PATHTYPE, size_t DIM>
  class wrapper_path_set : public path_set<DIM> {
   public:
    wrapper_path_set(PATHTYPE *paths, size_t size) : _paths(paths), _size(size) {}

    path<DIM> *get_path(size_t idx) override { return &_paths[idx]; }

    size_t get_path_count() override { return _size; }

   protected:
    PATHTYPE *_paths;
    size_t _size;
  };
}  // namespace path
}  // namespace grpl