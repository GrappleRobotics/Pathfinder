#include <gtest/gtest.h>
#include "grpl/param/arc_param.h"
#include "grpl/spline/hermite.h"

#include <fstream>
#include <iostream>

#include <array>
#include <list>
#include <vector>

using namespace grpl;
using namespace grpl::spline;
using namespace grpl::param;

TEST(ArcParam, Hermite) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {5, 5}, {0, 0}};
  hermite_t           hermite(start, end);

  std::list<arc_parameterizer::curve_t> curves;

  arc_parameterizer param;
  param.configure(0.1, 0.1);

  size_t numcurves_required = param.curve_count(&hermite);
  size_t numcurves =
      param.parameterize(&hermite, std::back_inserter(curves), curves.max_size());

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
  size_t numcurves = param.parameterize(&hermite, curves.begin(), curves.max_size());

  ASSERT_GT(numcurves_required, numcurves);
  ASSERT_TRUE(param.has_overrun());
}

TEST(ArcParam, Multispline) {
  using hermite_t = hermite<2, 5>;

  std::array<hermite_t::waypoint, 4> wps{hermite_t::waypoint{{2, 2}, {5, 0}, {0, 0}},
                                         hermite_t::waypoint{{3, 5}, {0, 5}, {0, 0}},
                                         hermite_t::waypoint{{5, 7}, {2, 2}, {0, 0}},
                                         hermite_t::waypoint{{7, 9}, {5, -5}, {0, 0}}};

  std::vector<hermite_t> hermites;
  hermite_factory::generate<hermite_t>(wps.begin(), wps.end(),
                                       std::back_inserter(hermites), hermites.max_size());

  std::vector<arc_parameterizer::curve_t> curves;

  arc_parameterizer param;
  param.configure(0.5, 0.5);

  size_t numcurves_required = param.curve_count(hermites.begin(), hermites.end());
  size_t numcurves          = param.parameterize(hermites.begin(), hermites.end(),
                                        std::back_inserter(curves), curves.max_size());

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

    ASSERT_FALSE(std::isnan(c.length()));

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