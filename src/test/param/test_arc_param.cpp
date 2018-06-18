#include <gtest/gtest.h>
#include "grpl/param/arc_param.h"
#include "grpl/spline/hermite.h"

#include <fstream>
#include <iostream>

using namespace grpl;
using namespace grpl::spline;
using namespace grpl::param;

TEST(ArcParam, Hermite) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {5, 5}, {0, 0}};
  hermite_t           hermite(start, end);

  std::vector<arc_parameterizer::curve_t> curves;

  arc_parameterizer param;
  param.configure(0.1, 0.1);

  size_t numcurves_required = param.curve_count(&hermite);
  curves.resize(numcurves_required);
  auto   curves_end = param.parameterize(&hermite, curves.begin(), curves.end());
  size_t numcurves  = std::distance(curves.begin(), curves_end);

  ASSERT_EQ(numcurves, numcurves_required);
  ASSERT_FALSE(param.has_overrun());
}

TEST(ArcParam, Overrun) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {5, 5}, {0, 0}};
  hermite_t           hermite(start, end);

  std::array<arc_parameterizer::curve_t, 1> curves;

  arc_parameterizer param;
  param.configure(0.1, 0.1);

  size_t numcurves_required = param.curve_count(&hermite);
  auto   curves_end         = param.parameterize(&hermite, curves.begin(), curves.end());
  size_t numcurves          = std::distance(curves.begin(), curves_end);

  ASSERT_GT(numcurves_required, numcurves);
  ASSERT_TRUE(param.has_overrun());
}

TEST(ArcParam, Multispline) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint      wp0{{2, 2}, {5, 0}, {0, 0}};
  hermite_t::waypoint      wp1{{5, 5}, {5, 5}, {0, 0}};
  hermite_t::waypoint      wp2{{8, 8}, {2, 0}, {0, 0}};
  std::array<hermite_t, 2> hermites{hermite_t{wp0, wp1}, hermite_t{wp1, wp2}};

  std::vector<arc_parameterizer::curve_t> curves;

  arc_parameterizer param;
  param.configure(0.5, 0.5);

  size_t numcurves_required = param.curve_count(hermites.begin(), hermites.end());
  curves.resize(numcurves_required);
  auto curves_end =
      param.parameterize(hermites.begin(), hermites.end(), curves.begin(), curves.end());
  size_t numcurves = std::distance(curves.begin(), curves_end);

  ASSERT_EQ(numcurves, numcurves_required);
  ASSERT_FALSE(param.has_overrun());

  std::ofstream outfile("arcparam.csv");
  outfile << "curveid,s,x,y,curvature\n";

  for (auto it = hermites.begin(); it != hermites.end(); it++) {
    for (double t = 0; t < 1; t += 0.001) {
      auto pos = it->calculate(t);
      outfile << -100.0 << ",0," << pos[0] << "," << pos[1] << ",0" << std::endl;
    }
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