#include <grpl/spline/hermite.h>
#include <grpl/units.h>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>

using namespace grpl;
using namespace grpl::units;
using namespace grpl::spline;

TEST(Hermite, Cubic) {
  using hermite_t = hermite<2>;

  hermite_t::waypoint start{{2, 2}, {5, 0}}, end{{5, 5}, {0, 5}};

  hermite_t hermite(start, end);

  std::ofstream outfile("hermite_cubic.csv");
  outfile << "t,x,y,curvature\n";

  hermite_t::vector_t position = start.point;

  // Check start and end points
  auto t0 = hermite.calculate(0);
  auto t1 = hermite.calculate(1);
  ASSERT_LT((t0 - start.point).norm(), 0.01) << t0;
  ASSERT_LT((t1 - end.point).norm(), 0.01) << t1;
  // Check start and end tangents
  auto t0_d = hermite.calculate_derivative(0);
  auto t1_d = hermite.calculate_derivative(1);
  ASSERT_LT((t0_d - start.tangent).norm(), 0.01) << t0;
  ASSERT_LT((t1_d - end.tangent).norm(), 0.01) << t1;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt    = hermite.calculate(t);
    auto deriv = hermite.calculate_derivative(t);

    position += deriv * 0.001;

    // Assert simulations match actual readings
    ASSERT_LT((position - pt).norm(), 0.02);

    outfile << t << "," << pt[0] << "," << pt[1] << "," << hermite.curvature(t)
            << std::endl;
  }
}

TEST(Hermite, Quintic) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {0, 5}, {0, 0}};

  hermite_t hermite(start, end);

  std::ofstream outfile("hermite_quintic.csv");
  outfile << "t,x,y,curvature\n";

  hermite_t::vector_t position = start.point, position_second = start.point;
  hermite_t::vector_t derivative = start.tangent;

  // Check start and end points
  auto t0 = hermite.calculate(0);
  auto t1 = hermite.calculate(1);
  ASSERT_LT((t0 - start.point).norm(), 0.01) << t0;
  ASSERT_LT((t1 - end.point).norm(), 0.01) << t1;
  // Check start and end tangents
  auto t0_d = hermite.calculate_derivative(0);
  auto t1_d = hermite.calculate_derivative(1);
  ASSERT_LT((t0_d - start.tangent).norm(), 0.01) << t0;
  ASSERT_LT((t1_d - end.tangent).norm(), 0.01) << t1;
  // Check start and end second derivatives
  auto t0_sd = hermite.calculate_second_derivative(0);
  auto t1_sd = hermite.calculate_second_derivative(1);
  ASSERT_LT((t0_sd - start.tangent_slope).norm(), 0.01) << t0;
  ASSERT_LT((t1_sd - end.tangent_slope).norm(), 0.01) << t1;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt       = hermite.calculate(t);
    auto deriv    = hermite.calculate_derivative(t);
    auto deriv2nd = hermite.calculate_second_derivative(t);

    derivative += deriv2nd * 0.001;

    position += deriv * 0.001;
    position_second += derivative * 0.001;

    // Assert simulations match actual readings
    ASSERT_LT((position - pt).norm(), 0.02);
    ASSERT_LT((derivative - deriv).norm(), 0.02);
    ASSERT_LT((position_second - pt).norm(), 0.02);

    outfile << t << "," << pt[0] << "," << pt[1] << "," << hermite.curvature(t)
            << std::endl;
  }
}