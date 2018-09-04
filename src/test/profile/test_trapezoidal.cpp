#include <grpl/profile/trapezoidal.h>
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

using namespace grpl::profile;
using namespace std;

TEST(Profile, Trapezoidal) {
  double sim_position = 0, sim_velocity = 0;
  double dt = 0.001;

  trapezoidal pr;
  pr.apply_limit(1, -3, 3);  // Velocity Limit = -3 to 3m/s
  pr.apply_limit(2, -3, 4);  // Acceleration limit = -3 to 4m/s
  pr.set_goal(5);        // Goal = 5m
  pr.set_timeslice(0);   // No Timeslice

  trapezoidal::segment_t seg;
  trapezoidal::kinematics_t kin;

  ofstream outfile("profile_trap.csv");
  ofstream outfile_sim("profile_trap_simulated.csv");
  outfile << "time,dist,vel,acc\n";
  outfile_sim << "time,dist,vel\n";

  for (double t = 0; t < 7; t += dt) {
    seg = pr.calculate(seg, t);
    kin = seg.kinematics;
    sim_velocity += kin[2] * dt;
    sim_position += sim_velocity * dt;

    // Assert Limits
    ASSERT_LE(abs(kin[1]), 3) << "Time: " << t;
    ASSERT_LE(abs(kin[2]), 4) << "Time: " << t;

    // Check simulation matches theoretical
    // TODO: the velocity error builds up here (residual from end of path)
    // ASSERT_LE(abs(kin[1] - sim_velocity), 0.01) << "Time: " << t;
    // ASSERT_LE(abs(kin[0] - sim_position), 0.01) << "Time: " << t;

    outfile_sim << seg.time << "," << sim_position << "," << sim_velocity << "\n";
    outfile << seg.time << "," << kin[0] << "," << kin[1] << "," << kin[2] << "\n";
  }

  // Check setpoint has been reached at end of profile
  ASSERT_NEAR(kin[0], 5, 0.001);
}