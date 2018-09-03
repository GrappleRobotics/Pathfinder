#include "grpl/model/coupled.h"
#include "grpl/path/arc_parameterizer.h"
#include "grpl/path/hermite.h"
#include "grpl/profile/trapezoidal.h"

#include <gtest/gtest.h>

#include <fstream>

using namespace grpl;

template <typename ST>
void echo(std::ofstream &out, ST state, std::string id) {
  out << state.time << "," << state.configuration.x() << "," << state.configuration.y() << ","
      << state.configuration[2] << "," << state.kinematics[0] << "," << state.kinematics[1] << ","
      << state.kinematics[2] << "," << state.curvature << "," << id << std::endl;
}

template <typename ST>
void echoWheel(std::ofstream &out, ST state, std::string id) {
  out << state.time << "," << state.position.x() << "," << state.position.y() << ","
      << "" << "," << "" << "," << state.velocity << ","
      << state.acceleration << "," << "" << "," << id << std::endl;
}

TEST(CDT, basic) {
  using hermite_t = path::hermite_quintic;
  using profile_t = profile::trapezoidal;
  using state_t   = typename model::coupled<profile_t>::state;
  using wheel_state_t = typename model::coupled<profile_t>::wheel_state;

  std::vector<path::augmented_arc2d> curves;
  std::array<hermite_t::waypoint, 2> wps{hermite_t::waypoint{{0, 0}, {5, 0}, {0, 0}},
                                         hermite_t::waypoint{{3, 5}, {0, 5}, {0, 0}}};

  std::vector<hermite_t> hermites;
  path::hermite_factory::generate<hermite_t>(wps.begin(), wps.end(), std::back_inserter(hermites),
                                             hermites.max_size());

  path::arc_parameterizer param;
  param.configure(0.1, 0.1);
  param.parameterize(hermites.begin(), hermites.end(), std::back_inserter(curves), curves.max_size());

  profile_t profile;
  profile.set_timeslice(0.0001);
  double G = 12.75;

  transmission::dc_motor    dualCIM{12.0, 5330 * 2.0 * PI / 60.0 / G, 2 * 2.7, 2 * 131.0, 2 * 2.41 * G};
  model::coupled<profile_t> model{dualCIM, dualCIM, 0.0762, 0.5, 25.0, 10.0};

  state_t state = model.initial_state();

  std::ofstream pathfile("cdt.csv");
  pathfile << "t,x,y,heading,distance,velocity,acceleration,curvature,path\n";

  for (double t = 0; !state.finished && t < 3; t += 0.01) {
    state = model.generate(curves.begin(), curves.end(), profile, state, t);
    echo(pathfile, state, "centre");

    std::pair<wheel_state_t, wheel_state_t> split = model.split(state);
    echoWheel(pathfile, split.first, "left");
    echoWheel(pathfile, split.second, "right");
  }
}