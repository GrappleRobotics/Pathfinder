#include "grpl/model/coupled.h"
#include "grpl/path/arc_parameterizer.h"
#include "grpl/path/hermite.h"
#include "grpl/profile/trapezoidal.h"

#include <gtest/gtest.h>

#include <fstream>

using namespace grpl;

TEST(CDT, basic) {
  using hermite_t = path::hermite_quintic;

  transmission::dc_motor               mot{12.0, 5330 * 2.0 * PI / 60.0, 2.7, 131, 2.41};
  model::coupled<profile::trapezoidal> model{mot, mot, 0.0762, 1.0, 25.0, 10.0};
  profile::trapezoidal                 profile;
  std::vector<path::augmented_arc2d>   curves;

  profile.set_timeslice(0.0001);

  std::array<hermite_t::waypoint, 2> wps{hermite_t::waypoint{{2, 2}, {5, 0}, {0, 0}},
                                         hermite_t::waypoint{{3, 5}, {0, 5}, {0, 0}}};

  std::vector<hermite_t> hermites;
  path::hermite_factory::generate<hermite_t>(wps.begin(), wps.end(), std::back_inserter(hermites),
                                             hermites.max_size());

  path::arc_parameterizer param;
  param.configure(0.1, 0.1);
  param.parameterize(hermites.begin(), hermites.end(), std::back_inserter(curves), curves.max_size());

  typename model::coupled<profile::trapezoidal>::state state = model.initial_state(6);

  std::ofstream pathfile("cdt.csv");
  pathfile << "t,x,y,heading,curvature,v,a\n";

  state.kinematics[2] = 10;

  for (double t = 0; !state.finished && t < 5; t += 0.01) {
    state = model.generate(curves.begin(), curves.end(), profile, state, t);
    pathfile << t << "," << state.configuration.x() << "," << state.configuration.y() << ","
             << state.configuration[2] << "," << state.kinematics[0] << "," << state.kinematics[1] << ","
             << state.kinematics[2] << std::endl;
  }
}