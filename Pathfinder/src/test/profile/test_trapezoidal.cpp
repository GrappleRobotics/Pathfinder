#include <gtest/gtest.h>
#include "grpl/pf/profile/trapezoidal.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace grpl::pf;
using namespace grpl::pf::profile;
using namespace std;

TEST(Profile, Trapezoidal) {
  double sim_position = 0, sim_velocity = 0;
  double dt = 0.001;

  trapezoidal pr;
  pr.apply_limit(VELOCITY, -3, 3);      // Velocity Limit = -3 to 3m/s
  pr.apply_limit(ACCELERATION, -3, 4);  // Acceleration limit = -3 to 4m/s
  pr.set_goal(5);                       // Goal = 5m
  pr.set_timeslice(0);                  // No Timeslice

  state           st;
  kinematic_state kin;

  ofstream outfile("profile_trap.csv");
  ofstream outfile_sim("profile_trap_simulated.csv");
  outfile << "time,dist,vel,acc\n";
  outfile_sim << "time,dist,vel\n";

  for (double t = 0; t < 7; t += dt) {
    st = pr.calculate(st, t);
    kin = st.kinematics;
    sim_velocity += kin[ACCELERATION] * dt;
    sim_position += sim_velocity * dt;

    // Assert Limits
    ASSERT_LE(abs(kin[1]), 3) << "Time: " << t;
    ASSERT_LE(abs(kin[2]), 4) << "Time: " << t;

    // Check simulation matches theoretical
    // TODO: the velocity error builds up here (residual from end of path)
    // ASSERT_LE(abs(kin[1] - sim_velocity), 0.01) << "Time: " << t;
    // ASSERT_LE(abs(kin[0] - sim_position), 0.01) << "Time: " << t;

    outfile_sim << st.time << "," << sim_position << "," << sim_velocity << "\n";
    outfile << st.time << "," << kin[POSITION] << "," << kin[VELOCITY] << "," << kin[ACCELERATION] << "\n";
  }

  // Check setpoint has been reached at end of profile
  ASSERT_NEAR(kin[0], 5, 0.001);
}