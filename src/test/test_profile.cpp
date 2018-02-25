#include <gtest/gtest.h>
#include <grpl/profile/trapezoidal.h>

#include <fstream>

using namespace grpl::profile;
using namespace grpl::units;

TEST(Profile, Root) {
  using profile_t = trapezoidal<Distance, Time, Velocity, Acceleration>;
  profile_t pr;
  pr.configure(3 * m / s, 4 * m / s / s);
  pr.goal(5 * m);

  profile_t::segment seg;
  std::ofstream out("out.csv");
  out << "time,dist,vel,acc\n";

  for (Time t = 0 * s; t < 10 * s; t += 10 * ms) {
    pr.calculate(&seg, &seg, t);
    out << seg.time.as(s) << "," << seg.dist.as(m) << "," << seg.vel.as(m / s) << "," << seg.acc.as(m / s / s) << std::endl;
  }
}