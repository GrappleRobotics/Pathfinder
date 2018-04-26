#include <gtest/gtest.h>
#include <grpl/profile/trapezoidal.h>

#include <cmath>
#include <fstream>

using namespace grpl::profile;
using namespace grpl::units;

TEST(Profile, Trapezoidal) {
    double sim_position = 0, sim_velocity = 0;
    double dt = (1*ms).as(s);

    trapezoidal pr;
    pr.apply_limit(1, 3);
    pr.apply_limit(2, 4);
    pr.set_goal(5);
    pr.set_timeslice(dt);

    trapezoidal::segment_t seg;
    std::ofstream outfile("profile_trap.csv");
    std::ofstream outfile_sim("profile_trap_simulated.csv");
    outfile << "time,dist,vel,acc\n";
    outfile_sim << "time,dist\n";

    for (Time t = 0*s; t < 3*s; t+=dt) {
        seg = pr.calculate(seg, t.as(s));
        sim_velocity += seg.vect[2] * dt;
        sim_position += sim_velocity * dt;

        // TODO: Check sim matches theoretical

        outfile_sim << seg.time << "," << sim_position << "\n";
        outfile << seg.time << "," << seg.vect[0] << "," << seg.vect[1] << "," << seg.vect[2] << "\n";
    }
}

// void test_trapezoidal(Velocity max_v, Acceleration max_a, Distance goal, Time timeslice, std::string fname) {
//   using profile_t = trapezoidal<Distance, Time, Velocity, Acceleration>;
//   profile_t pr;
//   pr.max_velocity(max_v);
//   pr.max_acceleration(max_a);
//   pr.goal(goal);
//   pr.timeslice(timeslice);

//   profile_t::segment seg;
//   std::ofstream outfile("profile_trapezoidal_" + fname + ".csv");
//   outfile << "time,dist,vel,acc\n";

//   for (Time t = 0 * s; t < 5 * s; t += 10 * ms) {
//     pr.calculate(&seg, &seg, t);
//     // Time value doesn't fluctuate past a half period (timeslice)
//     ASSERT_NEAR(seg.time.as(s), t.as(s), (timeslice / 2).as(s));
//     // Don't exceed max velocity
//     ASSERT_LE(abs(seg.vel.as(m / s)), max_v.as(m / s));
//     // Don't exceed max acceleration
//     ASSERT_LE(abs(seg.acc.as(m / s / s)), max_a.as(m / s / s));

//     outfile << seg.time.as(s) << "," << seg.dist.as(m) << "," << seg.vel.as(m/s) << "," << seg.acc.as(m/s/s) << "\n";
//   }
// }

// TEST(Trapezoidal, FwdLimits) {
//   test_trapezoidal(3*m/s, 4*m/s/s, 5*m, 0.5*ms, "forward");
// }

// TEST(Trapezoidal, RvsLimits) {
//   test_trapezoidal(3*m/s, 4 *m/s/s, -5*m, 0.5*ms, "reverse");
// }

// TEST(Trapezoidal, SmallTimesliceLimits) {
//   test_trapezoidal(3*m/s, 4*m/s/s, 5*m, 0.05*ms, "small_timeslice");
// }

// TEST(Trapezoidal, LargeTimesliceLimits) {
//   // Note: Timeslice larger than iteration time in test_trapezoidal, so 
//   // it does not slice at all.
//   test_trapezoidal(3*m/s, 4*m/s/s, 5*m, 1*s, "large_timescale");
// }

// TEST(Trapezoidal, LargeAcceleration) {
//   test_trapezoidal(3*m/s, 4000*km/s/s, 5*m, 0.05*ms, "large_acceleration");
// }