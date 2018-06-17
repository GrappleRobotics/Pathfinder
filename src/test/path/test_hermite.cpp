#include <grpl/path/hermite.h>
#include <grpl/units.h>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>

using namespace grpl;
using namespace grpl::units;
using namespace grpl::path;

TEST(Hermite, Cubic) {
  using hermite_t = hermite<2>;

  hermite_t::waypoint wp0{hermite_t::vector_t{2, 2}, hermite_t::vector_t{5, 0}},
      wp1{hermite_t::vector_t{5, 5}, hermite_t::vector_t{0, 5}};

  hermite_t     hermite(wp0, wp1);
  std::ofstream outfile("hermite_cubic.csv");
  outfile << "t,x,y,curvature\n";

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt = hermite.calculate(t);
    outfile << t << "," << pt[0] << "," << pt[1] << "," << hermite.calculate_curvature(t)
            << std::endl;
  }
}

TEST(Hermite, Quintic) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint wp0{hermite_t::vector_t{2, 2}, hermite_t::vector_t{5, 0},
                          hermite_t::vector_t{0, 0}},
      wp1{hermite_t::vector_t{5, 5}, hermite_t::vector_t{0, 5},
          hermite_t::vector_t{0, 0}};

  hermite_t     hermite(wp0, wp1);
  std::ofstream outfile("hermite_quintic.csv");
  outfile << "t,x,y,curvature\n";

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt = hermite.calculate(t);
    outfile << t << "," << pt[0] << "," << pt[1] << "," << hermite.calculate_curvature(t)
            << std::endl;
  }
}