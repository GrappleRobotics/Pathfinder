#include <gtest/gtest.h>
#include "grpl/pf/profile/trapezoidal.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>

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

  trapezoidal::segment_t    seg;
  trapezoidal::kinematics_t kin;

  ofstream outfile("profile_trap.csv");
  ofstream outfile_sim("profile_trap_simulated.csv");
  outfile << "time,dist,vel,acc\n";
  outfile_sim << "time,dist,vel\n";

  for (double t = 0; t < 7; t += dt) {
    seg = pr.calculate(seg, t);
    kin = seg.kinematics;
    sim_velocity += kin[ACCELERATION] * dt;
    sim_position += sim_velocity * dt;

    // Assert Limits
    ASSERT_LE(abs(kin[1]), 3) << "Time: " << t;
    ASSERT_LE(abs(kin[2]), 4) << "Time: " << t;

    // Check simulation matches theoretical
    // TODO: the velocity error builds up here (residual from end of path)
    // ASSERT_LE(abs(kin[1] - sim_velocity), 0.01) << "Time: " << t;
    // ASSERT_LE(abs(kin[0] - sim_position), 0.01) << "Time: " << t;

    outfile_sim << seg.time << "," << sim_position << "," << sim_velocity << "\n";
    outfile << seg.time << "," << kin[POSITION] << "," << kin[VELOCITY] << "," << kin[ACCELERATION] << "\n";
  }

  // Check setpoint has been reached at end of profile
  ASSERT_NEAR(kin[0], 5, 0.001);
}

TEST(Profile, SegmentConversion) {
  trapezoidal pr;
  pr.apply_limit(VELOCITY, -3, 3);      // Velocity Limit = -3 to 3m/s
  pr.apply_limit(ACCELERATION, -3, 4);  // Acceleration limit = -3 to 4m/s
  pr.set_goal(5);                       // Goal = 5m
  pr.set_timeslice(0);                  // No Timeslice

  trapezoidal::segment_t    seg, seg_last;

  segment<5> seg_large, seg_last_large;

  std::cout << std::setprecision(20);
  for (double t = 0; t < 7; t += 0.01) {
    seg = pr.calculate(seg_last, t);
    pr.calculate_into(seg_large, seg_last_large, t);

    // std::cout << seg.kinematics[1] << std::endl;
    // std::cout << seg_large.kinematics[1] << std::endl;

    size_t min_size = std::min(seg_large.ORDER, seg.ORDER);
    // Make sure common terms match
    for (size_t i = 0; i < min_size; i++) {
      ASSERT_DOUBLE_EQ(seg.kinematics[i], seg_large.kinematics[i]) << "Time: " << t << " Order: " << i;
    }
    // Make sure other terms are defaulted to zero.
    for (size_t i = min_size; i < seg_large.ORDER; i++) {
      ASSERT_DOUBLE_EQ(0, seg_large.kinematics[i]) << "Time: " << t << " Order: " << i;
    }
    seg_last_large = seg_large;
    seg_last = seg;
  }
}