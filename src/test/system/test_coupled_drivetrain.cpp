#include <gtest/gtest.h>
#include "grpl/param/arc_param.h"
#include "grpl/profile/trapezoidal.h"
#include "grpl/spline/hermite.h"
#include "grpl/system/coupled_drivetrain.h"
#include "grpl/units.h"

#include "test_util.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include <array>
#include <list>
#include <vector>

using namespace grpl;
using namespace grpl::spline;
using namespace grpl::param;
using namespace grpl::profile;
using namespace grpl::curve;
using namespace grpl::system;
using namespace grpl::units;
using namespace testutil;

template <typename profile_t, typename curve_t>
class CoupledDrivetrainTest : public ::testing::Test {
 protected:
  using cdt_t = coupled_drivetrain<curve_t, profile_t>;

  template <typename vec_t>
  std::string csv(vec_t vec) {
    std::stringstream strs;
    for (size_t i = 0; i < vec.rows(); i++) {
      strs << vec[i];
      if (i != vec.rows() - 1) strs << ",";
    }
    return strs.str();
  }

  void run() {
    typename cdt_t::state state = cdt.zero_state();
    state.done                  = false;

    for (size_t p = 0; p < 3; p++)
      for (size_t o = 0; o < profile_t::ORDER; o++) sims[p][o].zero();

    std::ofstream pathfile(filename + ".csv");
    std::ofstream simfile(filename + "_simulated.csv");
    std::ofstream vecfile(filename + "_vecsim.csv");

    pathfile << "path,t,x,y,d,v,a,angle,anglev,anglea,curvature,curveid\n";
    simfile << "path,control,t,x,y,v\n";
    vecfile << "path,control,t,x,y,vx,vy,ax,ay\n";

    for (double t = 0; !state.done && t < timeout; t += dt) {
      state                              = cdt.generate(curves.begin(), curves.end(), profile, state, t);
      typename curve_t::vector_t heading = vec_polar(1, state.a[0]);

      // Curve finding, for graphing.
      typename std::vector<curve_t>::iterator selected_curve;
      double                                  curve_len, total_len;
      bool                                    has_curve =
          cdt.find_curve(state.c.d, curves.begin(), curves.end(), selected_curve, curve_len, total_len);
      size_t curveid = has_curve ? std::distance(curves.begin(), selected_curve) : 0;

      for (size_t pid = 0; pid < 3; pid++) {
        typename cdt_t::coupled_side_t side = (pid == 0 ? state.c : pid == 1 ? state.l : state.r);
        std::string                    path = (pid == 0 ? "center" : pid == 1 ? "left" : "right");

        vecfile << path << ",ref" << t << "," << csv(side.k.col(0)) << "," << csv(side.k.col(1)) << ","
                << csv(side.k.col(2)) << std::endl;

        for (size_t o = 0; o < profile_t::ORDER; o++) {
          auto &sim = sims[pid][o];
          sim.write(side.k.col(o), o, dt);
          vecfile << path << ",o" << o << "," << t << "," << csv(sim.get(0)) << "," << csv(sim.get(1)) << ","
                  << csv(sim.get(2)) << std::endl;
          // TODO: Sim should function purely off of 1d velocities/accels, no vectors.
          simfile << path << ",o" << o << "," << t << "," << sim.get(0).dot(heading) << ","
                  << sim.get(1).dot(heading) << "," << sim.get(2).dot(heading) << std::endl;
        }

        pathfile << path << "," << t << "," << csv(side.k.col(0)) << "," << side.d << ","
                 << side.k.col(1).dot(heading) << "," << side.k.col(2).dot(heading) << "," << state.a[0]
                 << "," << state.a[1] << "," << state.a[2] << "," << state.curvature << "," << curveid
                 << std::endl;
      }
    }
  }

  double      dt       = (10 * ms).as(s);
  double      timeout  = (10 * s).as(s);
  std::string filename = "coupled";

  std::vector<curve_t> curves;
  cdt_t                cdt;
  profile_t            profile;
  // 3 Paths (left, center, right), 3 Control Orders (pos, vel, acc)
  pose_simulation<curve_t::DIMENSIONS, profile_t::ORDER> sims[3][profile_t::ORDER];
};

using CDTest = CoupledDrivetrainTest<trapezoidal, arc_parameterizer::curve_t>;
TEST_F(CDTest, Basic) {
  using hermite_t = hermite<2, 5>;
  cdt.apply_limit(1, 3);  // 1st derivative (velocity) limit
  cdt.apply_limit(2, 4);  // 2nd derivative (acceleration) limit

  cdt.set_trackwidth(0.5);

  profile.set_timeslice(0.0001);

  std::array<hermite_t::waypoint, 2> wps{hermite_t::waypoint{{2, 2}, {5, 0}, {0, 0}},
                                         hermite_t::waypoint{{3, 5}, {0, 5}, {0, 0}}};
  //  hermite_t::waypoint{{5, 7}, {2, 2}, {0, 0}},
  //  hermite_t::waypoint{{7, 9}, {5, -5}, {0, 0}}};

  std::vector<hermite_t> hermites;
  hermite_factory::generate<hermite_t>(wps.begin(), wps.end(), std::back_inserter(hermites),
                                       hermites.max_size());

  arc_parameterizer param;
  param.configure(0.1, 0.1);
  param.parameterize(hermites.begin(), hermites.end(), std::back_inserter(curves), curves.max_size());

  //   arc2d::vector_t start = {0, 0}, mid = {4, 2}, end = {5, 5}, end2 = { 6, 8 }, end3 =
  //   {10, 10}; curves.emplace_back(start, mid, end); curves.emplace_back(end, end2,
  //   end3);
  run();
}

// void write_vecinfo(std::ofstream &out, std::string path, double t, arc2d::vector_t p,
// arc2d::vector_t v, arc2d::vector_t a, double angle, double anglev, double
// curvature_radius, arc2d::vector_t curvcenter) {
//     out << path << ", " << t << "," << p[0] << "," << p[1] << "," << v[0] << "," <<
//     v[1] << "," << a[0] << "," << a[1] << "," << angle << "," << anglev << "," <<
//     curvature_radius << "," << curvcenter[0] << "," << curvcenter[1] << "\n";
// }

// template<typename profile_t, typename curve_t>
// void run_kinematics_test(std::string filename, coupled_drivetrain<curve_t, profile_t>
// &cdt, profile_t &profile, curve_t &curve) {
//     typename curve_t::vector_t pos_c_v, pos_c_a, vel_c_a, pos_l, pos_c, pos_r, vel_l,
//     vel_c, vel_r, vel_c_p, acc_c_p;

//     typename coupled_drivetrain<curve_t, profile_t>::state state, lastState;
//     state.done = false;

//     std::ofstream outfile(filename + ".csv");
//     std::ofstream outfile_sim(filename + "_simulated.csv");
//     std::ofstream outfile_vecinfo(filename + "_vecinfo.csv");
//     outfile << "path,t,x,y,d,v,a,angle,anglev\n";
//     outfile_sim << "path,t,x,y,v,a\n";
//     outfile_vecinfo <<
//     "path,t,x,y,vx,vy,ax,ay,angle,anglev,curvature_radius,curvature_x,curvature_y\n";

//     int i = 0;
//     const char *titles[3] = { "left", "center", "right" };
//     double dt = (1*ms).as(s);
//     for (Time t = 0; !state.done && t < 10*s; t+=dt) {
//         state = cdt.generate(&curve, &profile, state, t.as(s));
//         if (t == 0*s) {
//             pos_c_v = pos_c_a = state.c.k.col(0);
//             pos_c = state.c.k.col(0);
//             pos_l = state.l.k.col(0);
//             pos_r = state.r.k.col(0);
//             vel_c_a = { 0, 0 };
//             vel_c_p = { 0, 0 };
//         } else {
//             typename curve_t::vector_t last_vel = vel_c_p;
//             vel_c_p = (state.c.k.col(0) - lastState.c.k.col(0)) / dt;
//             acc_c_p = (vel_c_p - last_vel) / dt;
//         }

//         vel_l = state.l.k.col(1);
//         vel_r = state.r.k.col(1);
//         vel_c = state.c.k.col(1);

//         pos_l += vel_l * dt;
//         pos_r += vel_r * dt;
//         pos_c += vel_c * dt;

//         // TODO: I have a feeling this is failing because the velocity and acceleration
//         angles are not necessarily the
//         // same as the path angle.
//         // Yup, just verified by having the path be a straight line.

//         typename curve_t::vector_t unit_angle = vec_polar(1, state.a[0]);

//         outfile_sim << "left,"   << t.as(s) << "," << pos_l[0] << "," << pos_l[1] <<
//         "," << vel_l.dot(unit_angle) << ",0" << std::endl; outfile_sim << "right,"  <<
//         t.as(s) << "," << pos_r[0] << "," << pos_r[1] << "," << vel_r.dot(unit_angle)
//         << ",0" << std::endl; outfile_sim << "center,"  << t.as(s) << "," << pos_c[0]
//         << "," << pos_c[1] << "," << vel_c.dot(unit_angle) << ",0" << std::endl;

//         for (auto &it : { state.l, state.c, state.r }) {
//             // outfile << titles[(i++)%3] << ","
//             //     << it.t << "," << it.p[0] << "," << it.p[1] << "," << it.k[0] << ","
//             << it.k[1] << "," << it.k[2] << ","
//             //     << state.a[0] << "," << state.a[1] << "\n";
//             outfile << titles[(i++)%3] << "," << it.t << "," << it.k(0, 0) << "," <<
//             it.k(1, 0) << ","
//                     << it.d << "," << it.k.col(1).dot(unit_angle) << "," <<
//                     it.k.col(2).dot(unit_angle) << ","
//                     << state.a[0] << "," << state.a[1] << std::endl;

//             // outfile_vecinfo << titles[(i++)%3] << "," << it.t << "," << it.k(0, 0)
//             << "," << it.k(1, 0) << ","
//             //         << it.d << "," << it.k.col(1)[0] << "," << it.k.col(1)[1] << ","
//             << it.k.col(2)[0] << "," << it.k.col(2)[1] << ","
//             //         << state.a[0] << "," << state.a[1] << std::endl;
//         }

//         pos_c_v += state.c.k.col(1) * dt;
//         vel_c_a += state.c.k.col(2)*dt;
//         pos_c_a += vel_c*dt;

//         double radius = 1.0 / state.curvature;

//         typename curve_t::vector_t curv_center = state.c.k.col(0) + vec_polar(radius,
//         state.a[0] + (state.a[1] > 0 ? 1 : -1)*PI/2.0); write_vecinfo(outfile_vecinfo,
//         "posrel", t.as(s), state.c.k.col(0), vel_c_p, acc_c_p, state.a[0], state.a[1],
//         radius, curv_center);

//         curv_center = pos_c_v + vec_polar(radius, state.a[0] + (state.a[1] > 0 ? 1 :
//         -1)*PI/2.0); write_vecinfo(outfile_vecinfo, "velrel", t.as(s), pos_c_v,
//         state.c.k.col(1), state.c.k.col(2), state.a[0], state.a[1], radius,
//         curv_center);

//         curv_center = pos_c_a + vec_polar(radius, state.a[0] + (state.a[1] > 0 ? 1 :
//         -1)*PI/2.0); write_vecinfo(outfile_vecinfo, "accrel", t.as(s), pos_c_a,
//         vel_c_a, state.c.k.col(2), state.a[0], state.a[1], radius, curv_center);

//         lastState = state;
//     }
// }

// TEST(System, Coupled) {
//     using curve_t = arc2d;
//     using profile_t = trapezoidal;

//     coupled_drivetrain<curve_t, profile_t> cdt;
//     cdt.apply_limit(1, 3); // 1st derivative (velocity) limit
//     cdt.apply_limit(2, 4); // 2nd derivative (acceleration) limit

//     cdt.set_trackwidth(0.5);

//     curve_t::vector_t start = {0, 0}, mid = {4, 2}, end = {5, 5};
//     curve_t arc(start, mid, end);

//     profile_t profile;
//     profile.set_timeslice(0.0001);

//     run_kinematics_test("coupled", cdt, profile, arc);
// }

// -- BREAK -- //

// TEST(System, CoupledMulti) {
//     using hermite_t = hermite<2>;
//     using path_t = wrapper_path_set<hermite_t, 2>;
//     using profile_t = trapezoidal1;

//     coupled_drivetrain1<path_t, profile_t> cdt;
//     cdt.apply_limit(1, 3); // 1st derivative (velocity) limit
//     cdt.apply_limit(2, 4); // 2nd derivative (acceleration) limit

//     cdt.set_trackwidth(0.5);
//     hermite_t hermites[3];

//     hermite_t::waypoint wps[4] {
//         { hermite_t::vector_t{ 2, 2 }, hermite_t::vector_t{ 5, 0 } },
//         { hermite_t::vector_t{ 3, 5 }, hermite_t::vector_t{ 0, 5 } },
//         { hermite_t::vector_t{ 5, 7 }, hermite_t::vector_t{ 2, 2 } },
//         { hermite_t::vector_t{ 7, 9 }, hermite_t::vector_t{ 5, -5 } }
//     };
//     hermite_factory::generate(&wps[0], 4, &hermites[0], 3);
//     wrapper_path_set<hermite_t, 2> p(&hermites[0], 3);

//     profile_t profile;
//     profile.set_timeslice(0.001);

//     coupled_drivetrain1<path_t, profile_t>::state state;
//     state.done = false;

//     std::ofstream outfile("coupled_multi.csv");
//     outfile << "path,t,x,y,d,v,a,angle,anglev\n";

//     int i = 0;
//     const char *titles[3] = { "left", "center", "right" };
//     for (Time t = 0; !state.done; t+=1*ms) {
//         state = cdt.generate(&p, &profile, state, t.as(s));
//         for (auto &it : { state.c, state.l, state.r }) {
//             ASSERT_LE(it.k[1], cdt.get_limits()[1]);    // by being under the set
//             derivative limit, we can imply the function is continuous
//             ASSERT_LE(it.k[2], cdt.get_limits()[2]);
//         }
//         for (auto &it : { state.l, state.c, state.r }) {
//             outfile << titles[(i++)%3] << "," << it.t << "," << it.p[0] << "," <<
//             it.p[1] << "," << it.k[0] << "," << it.k[1] << "," << it.k[2] << "," <<
//             state.a[0] << "," << state.a[1] << "\n";
//         }
//     }
// }