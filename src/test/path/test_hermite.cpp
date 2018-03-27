#include <gtest/gtest.h>
#include <grpl/path/hermite.h>
#include <grpl/units.h>

#include <fstream>
#include <iostream>

using namespace grpl;
using namespace grpl::units;
using namespace grpl::path;

TEST(Hermite, Cubic) {
  using hermite_t = hermite<2>;

  hermite_t::waypoint   wp0 { hermite_t::vector_t{ 2, 2 }, hermite_t::vector_t{ 5, 0 } },
                        wp1 { hermite_t::vector_t{ 5, 5 }, hermite_t::vector_t{ 0, 5 } };

  hermite_t hermite(wp0, wp1, 100000);
  std::ofstream outfile("hermite_cubic.csv");
  outfile << "t,x,y,curvature\n";
  std::cout << static_cast<double>(hermite.get_arc_length()) << std::endl;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt = hermite.calculate(t);
    outfile << t << "," << pt[0] << "," << pt[1] << "," << hermite.calculate_curvature(t) << std::endl;
  }
}

TEST(Hermite, Quintic) {
  using hermite_t = hermite<2, 5>;

  hermite_t::waypoint   wp0 { hermite_t::vector_t{ 2, 2 }, hermite_t::vector_t{ 5, 0 }, hermite_t::vector_t{ 0, 0 } },
                        wp1 { hermite_t::vector_t{ 5, 5 }, hermite_t::vector_t{ 0, 5 }, hermite_t::vector_t{ 0, 0 } };

  hermite_t hermite(wp0, wp1, 100000);
  std::ofstream outfile("hermite_quintic.csv");
  outfile << "t,x,y,curvature\n";
  std::cout << static_cast<double>(hermite.get_arc_length()) << std::endl;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt = hermite.calculate(t);
    outfile << t << "," << pt[0] << "," << pt[1] << "," << hermite.calculate_curvature(t) << std::endl;
  }
}

TEST(Hermite, MultipleCubic) {
  using hermite_t = hermite<2>;

  hermite_t hermites[3];

  hermite_t::waypoint wps[4] {
    { hermite_t::vector_t{ 2, 2 }, hermite_t::vector_t{ 5, 0 } },
    { hermite_t::vector_t{ 3, 5 }, hermite_t::vector_t{ 0, 5 } },
    { hermite_t::vector_t{ 5, 7 }, hermite_t::vector_t{ 2, 2 } },
    { hermite_t::vector_t{ 7, 9 }, hermite_t::vector_t{ 5, -5 } }
  };
  hermite_factory::generate(&wps[0], 4, &hermites[0], 3);
  wrapper_path_set<hermite_t, 2> p(&hermites[0], 3);

  std::ofstream outfile("hermite_multicubic.csv");
  outfile << "t,x,y,curvature\n";
  std::cout << static_cast<double>(p.get_arc_length()) << std::endl;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt = p.calculate(t);
    outfile << t << "," << pt[0] << "," << pt[1] << "," << p.calculate_curvature(t) << std::endl;
  }
}

TEST(Hermite, MultipleQuintic) {
  using hermite_t = hermite<2, 5>;

  hermite_t hermites[3];

  hermite_t::waypoint wps[4] {
    { hermite_t::vector_t{ 2, 2 }, hermite_t::vector_t{ 5, 0 } },
    { hermite_t::vector_t{ 3, 5 }, hermite_t::vector_t{ 0, 5 } },
    { hermite_t::vector_t{ 5, 7 }, hermite_t::vector_t{ 2, 2 } },
    { hermite_t::vector_t{ 7, 9 }, hermite_t::vector_t{ 5, -5 } }
  };
  hermite_factory::generate(&wps[0], 4, &hermites[0], 3);
  wrapper_path_set<hermite_t, 2> p(&hermites[0], 3);

  std::ofstream outfile("hermite_multiquintic.csv");
  outfile << "t,x,y,curvature\n";
  std::cout << static_cast<double>(p.get_arc_length()) << std::endl;

  for (double t = 0; t <= 1; t += 0.001) {
    auto pt = p.calculate(t);
    outfile << t << "," << pt[0] << "," << pt[1] << "," << p.calculate_curvature(t) << std::endl;
  }
}