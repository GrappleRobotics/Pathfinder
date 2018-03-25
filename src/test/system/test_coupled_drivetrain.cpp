#include <gtest/gtest.h>
#include <grpl/profile/trapezoidal.h>
#include <grpl/path/hermite.h>
#include <grpl/system/coupled_drivetrain.h>

#include <cmath>
#include <fstream>

using namespace grpl::profile;
using namespace grpl::path;
using namespace grpl::system;
using namespace grpl::units;

TEST(System, Coupled) {
    //coupled_drivetrain1<hermite<Distance, 2>, trapezoidal1> dt;
//   using hermite_t = hermite < coupled_drivetrain::vec_t, Distance >;
//   using profile_t = trapezoidal<Distance, Time, Velocity, Acceleration>;
//   hermite_t::waypoint wp0 = { { 2 * m, 2 * m },{ 5 * m, 0 * m } },
//                       wp1 = { { 4 * m, 5 * m },{ 0    , 5 * m } };

//   hermite_t hermite(wp0, wp1, 10000);
//   profile_t profile;
//   auto len = hermite.calculate_arc_length();

//   std::ofstream outfile("coupled.csv");
//   outfile << "path,t,x,y,d,v,a,angle,anglev\n";

//   coupled_drivetrain dt(&profile, &hermite, 0.5*m, 3*m/s, 4 * m/s/s);

//   coupled_drivetrain::point point;
//   bool done = false;
//   for (Time t = 0; !done; t += 1 * ms) {
//     done = dt.generate(len, &point, &point, t);

//     const char * titles[3] = { "left", "center", "right" };
//     int i = 0;
//     for (auto &it : { point.left, point.center, point.right }) {
//       outfile << titles[(i++) % 3] << "," << point.time.as(s) << "," << it.pos.x.as(m) << ","
//         << it.pos.y.as(m) << "," << it.dist.as(m) << "," << it.vel.as(m / s) << ","
//         << it.acc.as(m / s / s) << "," << point.angle.as(deg) << "," << point.angular_velocity.as(deg / s) << std::endl;
//     }
//   }
}