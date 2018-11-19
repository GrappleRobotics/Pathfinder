#include <gtest/gtest.h>
#include "grpl/pf/path/arc.h"

#include <fstream>
#include <iostream>

using namespace grpl::pf;
using namespace grpl::pf::path;

TEST(Arc, ArcTest) {
  arc2d::vector_t start = {0, 0}, mid = {4, 2}, end = {5, 5};

  arc2d arc(start, mid, end);

  std::ofstream outfile("arc.csv");
  outfile << "t,x,y,vx,vy,curvature\n";

  arc2d::vector_t position = start;

  // Check start and end points
  auto s0 = arc.position(0);
  auto s1 = arc.position(arc.length());
  ASSERT_LT((s0 - start).norm(), 0.02) << s0;
  ASSERT_LT((s1 - end).norm(), 0.02) << s1;

  double curvature = arc.curvature(0);

  for (double s = 0; s < arc.length(); s += 0.01) {
    auto pos   = arc.position(s);
    auto deriv = arc.derivative(s);

    position += deriv * 0.01;

    // Simulated position (from derivative) should
    // match the actual position calculated. Confirms
    // position and derivative are correct.
    ASSERT_LT((position - pos).norm(), 0.02);

    // Curvature of an arc (1/radius) is constant.
    ASSERT_DOUBLE_EQ(curvature, arc.curvature(s));

    outfile << s << "," << pos[0] << "," << pos[1] << "," << deriv[0] << "," << deriv[1] << ","
            << arc.curvature(s) << std::endl;
  }
}