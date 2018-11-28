#pragma once

#include "spline.h"

namespace grpl {
namespace pf {
  namespace path {

    /**
     * Hermite Spline Base Class
     *
     * Base implementation of a hermite spline, either quintic or cubic, as detemined
     * by the ORDER template parameter. This class should not be used directly, instead
     * see @ref hermite_cubic and @ref hermite_quintic.
     *
     * @param ORDER the order of the spline. 3 = Cubic, 5 = Quintic.
     */
    template <size_t ORDER = 3>
    class hermite : public spline<2> {
     public:
      using vector_t         = typename spline::vector_t;
      using basis_t          = typename Eigen::Matrix<double, ORDER + 1, 1>;
      using control_matrix_t = typename Eigen::Matrix<double, 2, ORDER + 1>;

      /**
       * Create a default hermite, with identical zero'd start and end waypoints
       */
      hermite() {}

      /**
       * Create a hermite spline with the given control matrix.
       *
       * The control matrix is structured with vector columns. The first half of
       * the columns are for the start point, and the second half for the end point.
       * The columns are ordered position, tangent for cubic, and position, tangent,
       * dtangent for quintic.
       *
       * @param M The control matrix.
       */
      hermite(control_matrix_t &M) { set_control_matrix(M); }

      /**
       * Set the control matrix. See @ref hermite(control_matrix_t &)
       */
      void set_control_matrix(control_matrix_t &M) { _M = M; }

      /**
       * Get the control matrix. See @ref hermite(control_matrix_t &)
       */
      control_matrix_t &get_control_matrix() { return _M; }

      vector_t position(double t) override { return _M * basis(t); }

      vector_t derivative(double t) override { return _M * basis_1st(t); }

      vector_t derivative2(double t) { return _M * basis_2nd(t); }

      double curvature(double t) override {
        vector_t h_p = derivative(t), h_pp = derivative2(t);

        return (h_p[0] * h_pp[1] - h_p[1] * h_pp[0]) / pow(h_p.norm(), 3);
      }

     protected:
      //! basis matrix
      virtual basis_t basis(double t) const = 0;
      //! derivatives of basis
      virtual basis_t basis_1st(double t) const = 0;
      //! 2nd derivatives of basis
      virtual basis_t basis_2nd(double t) const = 0;

      control_matrix_t _M;
    };

    /**
     * Implementation of a cubic (order 3) hermite spline.
     *
     * A Cubic Hermite Spline is a spline of order 3, as defined here
     * https://en.wikipedia.org/wiki/Cubic_Hermite_spline.
     *
     * The cubic spline is defined in regards to its waypoints, which are defined in terms of position and
     * tangent.
     */
    class hermite_cubic : public hermite<3> {
     public:
      /**
       * Waypoint for a cubic hermite spline.
       */
      struct waypoint {
        //! 2D position of the waypoint, in metres. Ordered x, y.
        vector_t position;
        //! 2D tangent to the waypoint, in metres. Ordered x, y.
        vector_t tangent;
      };

      hermite_cubic() = default;

      /**
       * Construct a cubic hermite spline, given a start and end point.
       *
       * @param start The waypoint of the start of the spline
       * @param end   The waypoint of the end of the spline.
       */
      hermite_cubic(waypoint &start, waypoint &end) { set_waypoints(start, end); }

      /**
       * Set the start and end waypoints of the spline.
       *
       * @param start The waypoint of the start of the spline
       * @param end   The waypoint of the end of the spline.
       */
      void set_waypoints(waypoint &start, waypoint &end) {
        _M.col(0) = start.position;
        _M.col(1) = start.tangent;
        _M.col(2) = end.position;
        _M.col(3) = end.tangent;
      }

     protected:
      inline basis_t basis(double t) const override {
        // 2t^3 - 3t^2 + 1
        // t^3 - 2t^2 + t
        // -2t^3 + 3t^2
        // t^3 - t^2
        return (basis_t() << (2 * t * t * t - 3 * t * t + 1), (t * t * t - 2 * t * t + t),
                (-2 * t * t * t + 3 * t * t), (t * t * t - t * t))
            .finished();
      }

      inline basis_t basis_1st(double t) const override {
        return (basis_t() << (6 * t * t - 6 * t), (3 * t * t - 4 * t + 1), (-6 * t * t + 6 * t),
                (3 * t * t - 2 * t))
            .finished();
      }

      inline basis_t basis_2nd(double t) const override {
        return (basis_t() << (12 * t - 6), (6 * t - 4), (-12 * t + 6), (6 * t - 2)).finished();
      }
    };

    /**
     * Implementation of a quintic (order 5) hermite spline.
     * 
     * The quintic spline is defined in regards to its waypoints, which are defined in terms of position,
     * tangent and the derivative of the tangent.
     */
    class hermite_quintic : public hermite<5> {
     public:
      /**
       * Waypoint for a quintic hermite spline.
       */
      struct waypoint {
        //! 2D position of the waypoint, in metres. Ordered x, y.
        vector_t position;
        //! 2D tangent to the waypoint, in metres. Ordered x, y.
        vector_t tangent;
        //! 2D derivative of the tangent to the waypoint, in metres. Ordered x, y.
        vector_t dtangent;
      };

      hermite_quintic() = default;

      /**
       * Construct a quintic hermite spline, given a start and end point.
       *
       * @param start The waypoint of the start of the spline
       * @param end   The waypoint of the end of the spline.
       */
      hermite_quintic(waypoint &start, waypoint &end) { set_waypoints(start, end); }

      /**
       * Set the start and end waypoints of the spline.
       *
       * @param start The waypoint of the start of the spline
       * @param end   The waypoint of the end of the spline.
       */
      void set_waypoints(waypoint &start, waypoint &end) {
        _M.col(0) = start.position;
        _M.col(1) = start.tangent;
        _M.col(2) = start.dtangent;
        _M.col(3) = end.position;
        _M.col(4) = end.tangent;
        _M.col(5) = end.dtangent;
      }

     protected:
      inline basis_t basis(double t) const override {
        // 1 - 10t^3 + 15t^4 - 6t^5
        // t - 6t^3 + 8t^4 - 3t^5
        // 0.5*t^2 - 1.5t^3 + 1.5t^4 - 0.5t^5
        // 10t^3 - 15t^4 + 6t^5
        // 7t^4 - 4t^3 - 3t^5
        // 0.5t^3 - t^4 + 0.5t^5
        return (basis_t() << (1 - 10 * t * t * t + 15 * t * t * t * t - 6 * t * t * t * t * t),
                (t - 6 * t * t * t + 8 * t * t * t * t - 3 * t * t * t * t * t),
                (0.5 * t * t - 1.5 * t * t * t + 1.5 * t * t * t * t - 0.5 * t * t * t * t * t),
                (10 * t * t * t - 15 * t * t * t * t + 6 * t * t * t * t * t),
                (7 * t * t * t * t - 4 * t * t * t - 3 * t * t * t * t * t),
                (0.5 * t * t * t - t * t * t * t + 0.5 * t * t * t * t * t))
            .finished();
      }

      inline basis_t basis_1st(double t) const override {
        return (basis_t() << (-30 * t * t + 60 * t * t * t - 30 * t * t * t * t),
                (1 - 18 * t * t + 32 * t * t * t - 15 * t * t * t * t),
                (t - 4.5 * t * t + 6 * t * t * t - 2.5 * t * t * t * t),
                (30 * t * t - 60 * t * t * t + 30 * t * t * t * t),
                (28 * t * t * t - 12 * t * t - 15 * t * t * t * t),
                (1.5 * t * t - 4 * t * t * t + 2.5 * t * t * t * t))
            .finished();
      }

      inline basis_t basis_2nd(double t) const override {
        return (basis_t() << (-60 * t + 180 * t * t - 120 * t * t * t),
                (-36 * t + 96 * t * t - 60 * t * t * t), (1 - 9 * t + 18 * t * t - 10 * t * t * t),
                (60 * t - 180 * t * t + 120 * t * t * t), (84 * t * t - 24 * t - 60 * t * t * t),
                (3 * t - 12 * t * t + 10 * t * t * t))
            .finished();
      }
    };

    // TODO: How to structure this better
    namespace hermite_factory {
      template <typename hermite_t, typename output_iterator_t, typename iterator_wp_t>
      size_t generate(const iterator_wp_t wp_begin, const iterator_wp_t wp_end, output_iterator_t &&out,
                      size_t max_size) {
        if (wp_begin == wp_end) return 0;
        if (max_size < std::distance(wp_begin, wp_end)) return 0;  // Not enough space!

        size_t        size    = 0;
        iterator_wp_t last_wp = wp_begin, start = wp_begin;
        for (iterator_wp_t it = ++start; it != wp_end; it++) {
          *(out++) = hermite_t{*last_wp, *it};
          last_wp  = it;
          size++;
        }
        return size;
      }
    }  // namespace hermite_factory
  }    // namespace path
}  // namespace pf
}  // namespace grpl