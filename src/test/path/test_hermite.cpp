#include <gtest/gtest.h>
#include <grpl/path/hermite.h>

#include <fstream>

using namespace grpl;
using namespace grpl::units;
using namespace grpl::path;

TEST(Path, Hermite) {
  using hermite_t = hermite < vec2D<Distance> >;
  hermite_t::waypoint wp0 = { { 2, 2 }, { 1, 0 } },
                      wp1 = { { 2, 5 }, { 0, 0 } };

  hermite_t hermite(wp0, wp1);
  std::ofstream outfile("hermite.csv");
  outfile << "t,x,y\n";

  for (double t = 0; t <= 1; t += 0.001) {
    vec2D<Distance> pt = hermite.calculate(t);
    outfile << t << "," << pt.x.as(m) << "," << pt.y.as(m) << std::endl;
  }
}