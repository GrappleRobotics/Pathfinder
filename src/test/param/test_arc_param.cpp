#include <gtest/gtest.h>
#include "grpl/param/arc_param.h"
#include "grpl/path/hermite.h"

#include <fstream>
#include <iostream>

using namespace grpl;
using namespace grpl::path;
using namespace grpl::param;

TEST(ArcParam, Hermite) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {5, 5}, {0, 0}};
  hermite_t           hermite(start, end);

  augmented_arc2d curves[128];

  arc_parameterizer param(&hermite);
  param.configure(0.1, 0.1);

  size_t numcurves = param.parameterize(curves, 100);

  std::cout << numcurves << std::endl;

  std::ofstream outfile("arcparam.csv");
  outfile << "curveid,s,x,y,curvature\n";

  for (double t = 0; t < 1; t += 0.001) {
    auto pos = hermite.calculate(t);
    outfile << -100.0 << ",0," << pos[0] << "," << pos[1] << ",0" << std::endl;
  }

  size_t curve = 0;
  double s = 0, si = 0;
  while (curve < numcurves) {
    auto c    = curves[curve];
    auto pos  = c.calculate(si);
    auto curv = c.curvature(si);

    outfile << curve << "," << s << "," << pos[0] << "," << pos[1] << "," << curv
            << std::endl;

    si += 0.01;
    s += 0.01;
    if (si > c.length()) {
      curve++;
      si = 0;
    }
  }
}