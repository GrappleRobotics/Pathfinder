#include <gtest/gtest.h>
#include "grpl/pf/path/hermite.h"

#include <array>
#include <fstream>
#include <iostream>
#include <vector>

using namespace grpl::pf;
using namespace grpl::pf::path;

TEST(Hermite, Cubic) {
  using hermite_t = hermite_cubic;

  hermite_t::waypoint start{{2, 2}, {5, 0}}, end{{5, 5}, {0, 5}};

  hermite_t hermite(start, end);

  std::ofstream outfile("hermite_cubic.csv");
  outfile << "t,x,y,curvature\n";

  hermite_t::vector_t position = start.position;

  // Check start and end points
  auto t0 = hermite.position(0);
  auto t1 = hermite.position(1);
  ASSERT_LT((t0 - start.position).norm(), 0.01) << t0;
  ASSERT_LT((t1 - end.position).norm(), 0.01) << t1;
  // Check start and end tangents
  auto t0_d = hermite.derivative(0);
  auto t1_d = hermite.derivative(1);
  ASSERT_LT((t0_d - start.tangent).norm(), 0.01) << t0;
  ASSERT_LT((t1_d - end.tangent).norm(), 0.01) << t1;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt    = hermite.position(t);
    auto deriv = hermite.derivative(t);
    auto curv  = hermite.curvature(t);

    position += deriv * 0.001;

    // Assert simulations match actual readings
    ASSERT_LT((position - pt).norm(), 0.02);

    ASSERT_GT(curv, 0);

    outfile << t << "," << pt[0] << "," << pt[1] << "," << curv << std::endl;
  }
}

TEST(Hermite, Quintic) {
  using hermite_t = hermite_quintic;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {0, 5}, {0, 0}};

  hermite_t hermite(start, end);

  std::ofstream outfile("hermite_quintic.csv");
  outfile << "t,x,y,curvature\n";

  hermite_t::vector_t position = start.position, position_second = start.position;
  hermite_t::vector_t derivative = start.tangent;

  // Check start and end points
  auto t0 = hermite.position(0);
  auto t1 = hermite.position(1);
  ASSERT_LT((t0 - start.position).norm(), 0.01) << t0;
  ASSERT_LT((t1 - end.position).norm(), 0.01) << t1;
  // Check start and end tangents
  auto t0_d = hermite.derivative(0);
  auto t1_d = hermite.derivative(1);
  ASSERT_LT((t0_d - start.tangent).norm(), 0.01) << t0;
  ASSERT_LT((t1_d - end.tangent).norm(), 0.01) << t1;
  // Check start and end second derivatives
  auto t0_sd = hermite.derivative2(0);
  auto t1_sd = hermite.derivative2(1);
  ASSERT_LT((t0_sd - start.dtangent).norm(), 0.01) << t0;
  ASSERT_LT((t1_sd - end.dtangent).norm(), 0.01) << t1;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt       = hermite.position(t);
    auto deriv    = hermite.derivative(t);
    auto deriv2nd = hermite.derivative2(t);
    auto curv     = hermite.curvature(t);

    derivative += deriv2nd * 0.001;

    position += deriv * 0.001;
    position_second += derivative * 0.001;

    // Assert simulations match actual readings
    ASSERT_LT((position - pt).norm(), 0.02);
    ASSERT_LT((derivative - deriv).norm(), 0.02);
    ASSERT_LT((position_second - pt).norm(), 0.02);

    ASSERT_GE(curv, 0);

    outfile << t << "," << pt[0] << "," << pt[1] << "," << curv << std::endl;
  }
}

TEST(Hermite, NegativeCurvature) {
  using hermite_t = hermite_cubic;

  hermite_t::waypoint start{{2, 2}, {5, 0}}, end{{5, -1}, {0, -5}};

  hermite_t hermite(start, end);

  std::ofstream outfile("hermite_cubic_negcurv.csv");
  outfile << "t,x,y,curvature\n";

  hermite_t::vector_t position = start.position;

  // Check start and end points
  auto t0 = hermite.position(0);
  auto t1 = hermite.position(1);
  ASSERT_LT((t0 - start.position).norm(), 0.01) << t0;
  ASSERT_LT((t1 - end.position).norm(), 0.01) << t1;
  // Check start and end tangents
  auto t0_d = hermite.derivative(0);
  auto t1_d = hermite.derivative(1);
  ASSERT_LT((t0_d - start.tangent).norm(), 0.01) << t0;
  ASSERT_LT((t1_d - end.tangent).norm(), 0.01) << t1;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt    = hermite.position(t);
    auto deriv = hermite.derivative(t);
    auto curv  = hermite.curvature(t);

    position += deriv * 0.001;

    // Assert simulations match actual readings
    ASSERT_LT((position - pt).norm(), 0.02);

    ASSERT_LT(curv, 0);

    outfile << t << "," << pt[0] << "," << pt[1] << "," << curv << std::endl;
  }
}

template <typename hermite_t>
void multitest(std::string name) {
  using waypoint_t = typename hermite_t::waypoint;

  std::vector<waypoint_t> wps;
  wps.push_back(waypoint_t{{2, 2}, {5, 0}});
  wps.push_back(waypoint_t{{3, 5}, {0, 5}});
  wps.push_back(waypoint_t{{5, 7}, {2, 2}});
  wps.push_back(waypoint_t{{7, 9}, {5, -5}});

  std::vector<hermite_t> hermites;

  size_t num_hermites = hermite_factory::generate<hermite_t>(
      wps.begin(), wps.end(), std::back_inserter(hermites), hermites.max_size());

  ASSERT_EQ(num_hermites, wps.size() - 1);

  std::ofstream outfile("hermite_" + name + ".csv");
  outfile << "t,x,y,curvature\n";

  for (size_t i = 0; i < num_hermites; i++) {
    for (double t = 0; t <= 1; t += 0.001) {
      auto pt = hermites[i].position(t);
      outfile << (t + i) << "," << pt[0] << "," << pt[1] << "," << hermites[i].curvature(t) << std::endl;
    }
  }
}

TEST(Hermite, MultiCubic) {
  multitest<hermite_cubic>("multicubic");
}

TEST(Hermite, MultiQuintic) {
  multitest<hermite_quintic>("multiquintic");
}

TEST(Hermite, MultiZeroWP) {
  using hermite_t = hermite_quintic;
  std::vector<hermite_t::waypoint> wps;
  wps.push_back(hermite_t::waypoint{});

  std::array<hermite_t, 10> hermites;

  size_t num_hermites =
      hermite_factory::generate<hermite_t>(wps.begin(), wps.end(), hermites.begin(), hermites.max_size());

  ASSERT_EQ(num_hermites, 0);
}