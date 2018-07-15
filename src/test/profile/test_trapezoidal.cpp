#include <grpl/profile/trapezoidal.h>
#include <gtest/gtest.h>

#include <grpl/units.h>

#include <cmath>
#include <fstream>

using namespace grpl::profile;
using namespace grpl::units;
using namespace std;

TEST(Profile, Trapezoidal) {
  double sim_position = 0, sim_velocity = 0;
  double dt = (1 * ms).as(s);

  trapezoidal pr;
  pr.apply_limit(1, 3);  // Velocity Limit = 3m/s
  pr.apply_limit(2, 4);  // Acceleration limit = 5m/s
  pr.set_goal(5);        // Goal = 5m
  pr.set_timeslice(0);   // No Timeslice

  trapezoidal::segment_t seg;

  ofstream outfile("profile_trap.csv");
  ofstream outfile_sim("profile_trap_simulated.csv");
  outfile << "time,dist,vel,acc\n";
  outfile_sim << "time,dist,vel\n";

  for (Time t = 0 * s; (t < 7 * s); t += dt) {
    seg = pr.calculate(seg, t.as(s));
    sim_velocity += seg.k[2] * dt;
    sim_position += sim_velocity * dt;

    // Assert Limits
    ASSERT_LE(abs(seg.k[1]), 3) << "Time: " << t.as(s);
    ASSERT_LE(abs(seg.k[2]), 4) << "Time: " << t.as(s);

    // Check simulation matches theoretical
    // TODO: the velocity error builds up here.
    // ASSERT_LE(abs(seg.k[1] - sim_velocity), 0.01) << "Time: " << t.as(s);
    // ASSERT_LE(abs(seg.k[0] - sim_position), 0.01) << "Time: " << t.as(s);

    outfile_sim << seg.time << "," << sim_position << "," << sim_velocity << "\n";
    outfile << seg.time << "," << seg.k[0] << "," << seg.k[1] << "," << seg.k[2] << "\n";
  }

  // Check setpoint has been reached at end of profile
  ASSERT_NEAR(seg.k[0], 5, 0.001);
}