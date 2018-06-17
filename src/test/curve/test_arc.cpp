#include <grpl/curve/arc.h>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>

using namespace grpl;
using namespace grpl::curve;

TEST(Arc, ArcTest) {
  arc2d::vector_t start = {0, 0}, mid = {4, 2}, end = {5, 5};
  arc2d arc(start, mid, end);

  std::ofstream outfile("arc.csv");
  outfile << "t,x,y,vx,vy,curvature\n";

  arc2d::vector_t position = start;

  // Check start and end points
  auto s0 = arc.calculate(0);
  auto s1 = arc.calculate(arc.length());
  ASSERT_LT((s0 - start).norm(), 0.02) << s0;
  ASSERT_LT((s1 - end).norm(), 0.02) << s1;

  for (double s = 0; s < arc.length(); s += 0.01) {
    auto pos = arc.calculate(s);
    auto deriv = arc.calculate_derivative(s);

    position += deriv * 0.01;

    // Check that infinitescimal derivative integration matches
    // actual position.
    ASSERT_LT((position - pos).norm(), 0.02);

    outfile << s << "," << pos[0] << "," << pos[1] << "," << deriv[0] << "," << deriv[1]
            << "," << arc.curvature(s) << std::endl;
  }
}