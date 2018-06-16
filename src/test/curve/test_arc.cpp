#include <grpl/curve/arc.h>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>

using namespace grpl;
using namespace grpl::curve;

TEST(Arc, ArcTest) {
  arc2d::vector_t start = {-6, 6}, mid = {-3.5, 5.5}, end = {-1, 3};
  arc2d arc(start, mid, end);

  std::ofstream outfile("arc.csv");
  outfile << "t,x,y,vx,vy,curvature\n";

  arc2d::vector_t position = start;

  // Check start and end points
  ASSERT_TRUE(arc.calculate(0).isApprox(start));
  ASSERT_TRUE(arc.calculate(arc.length()).isApprox(end));

  for (double s = 0; s < arc.length(); s += 0.01) {
    auto pos = arc.calculate(s);
    auto deriv = arc.calculate_derivative(s);

    position += deriv * 0.01;

    // Check that infinitescimal derivative integration matches
    // actual position.
    ASSERT_TRUE(position.isApprox(pos, 0.02));

    outfile << s << "," << pos[0] << "," << pos[1] << "," << deriv[0] << "," << deriv[1]
            << "," << arc.curvature(s) << std::endl;
  }
}