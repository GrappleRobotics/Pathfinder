#include "grpl/model/coupled.h"
#include "grpl/path/arc_parameterizer.h"
#include "grpl/path/hermite.h"
#include "grpl/profile/trapezoidal.h"

#include <gtest/gtest.h>

#include <fstream>

using namespace grpl;

template <typename ST>
void echo(std::ofstream &out, ST state, int id) {
  out << state.time << "," << state.configuration.x() << "," << state.configuration.y() << ","
      << state.configuration[2] << "," << state.kinematics[0] << "," << state.kinematics[1] << ","
      << state.kinematics[2] << "," << state.curvature << "," << id << "," << "" << "," << std::endl;
}

void echo_limits(std::ofstream &out, double time, profile::trapezoidal::limits_t lim) {
  auto vel = lim.col(1);
  auto acc = lim.col(2);

  out << time << ",,,,," << vel[0] << "," << acc[0] << ",,-1,," << std::endl;
  out << time << ",,,,," << vel[1] << "," << acc[1] << ",,-1,," << std::endl;
}

void echo_simulation(std::ofstream &out, double time, Eigen::Vector3d config) {
  out << time << "," << config.x() << "," << config.y() << "," << config[2] << ",,,,,3,," << std::endl;
}

template <typename ST>
void echo_wheel(std::ofstream &out, ST state, int id) {
  out << state.time << "," << state.position.x() << "," << state.position.y() << ","
      << "" << "," << "" << "," << state.velocity << "," << state.acceleration << "," 
      << "" << "," << id << "," << state.voltage << "," << state.current << std::endl;
}

TEST(CDT, basic) {
  using hermite_t = path::hermite_quintic;
  using profile_t = profile::trapezoidal;
  using coupled_t = model::coupled<profile_t>;
  using state_t   = typename coupled_t::state;
  using wheel_state_t = typename coupled_t::wheel_state;

  std::vector<path::augmented_arc2d> curves;
  std::array<hermite_t::waypoint, 2> wps{hermite_t::waypoint{{0, 0}, {5, 0}, {0, 0}},
                                         hermite_t::waypoint{{4, 4}, {0, 5}, {0, 0}}};

  std::vector<hermite_t> hermites;
  path::hermite_factory::generate<hermite_t>(wps.begin(), wps.end(), std::back_inserter(hermites),
                                             hermites.max_size());

  path::arc_parameterizer param;
  param.configure(0.01, 0.01);
  param.parameterize(hermites.begin(), hermites.end(), std::back_inserter(curves), curves.max_size());

  profile_t profile;
  // profile.set_timeslice(0.00001);
  double G = 12.75;

  transmission::dc_motor  dualCIM{12.0, 5330 * 2.0 * PI / 60.0 / G, 2 * 2.7, 2 * 131.0, 2 * 2.41 * G};
  coupled_t               model{dualCIM, dualCIM, 0.0762, 0.5, 25.0, 10.0};

  state_t state = model.initial_state();

  std::ofstream pathfile("cdt.csv");
  pathfile << "t,x,y,heading,distance,velocity,acceleration,curvature,path,voltage,current\n";

  typename coupled_t::configuration_t centre{0, 0, 0};

  for (double t = 0; !state.finished && t < 5; t += 0.01) {
    state = model.generate(curves.begin(), curves.end(), profile, state, t);
    echo(pathfile, state, 0);

    echo_limits(pathfile, t, profile.get_limits());

    std::pair<wheel_state_t, wheel_state_t> split = model.split(state);
    echo_wheel(pathfile, split.first, 1);
    echo_wheel(pathfile, split.second, 2);

    double w = (split.second.velocity - split.first.velocity);    // track radius = 0.5, track diam = 1
    double v = (split.second.velocity + split.first.velocity) / 2.0;

    // TODO: Checks

    centre[2] += w * 0.01;
    centre[0] += v * cos(centre[2]) * 0.01;
    centre[1] += v * sin(centre[2]) * 0.01;

    echo_simulation(pathfile, t, centre);
  }
}