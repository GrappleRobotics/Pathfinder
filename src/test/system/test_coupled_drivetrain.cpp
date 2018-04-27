#include <gtest/gtest.h>
#include <grpl/profile/trapezoidal.h>
#include <grpl/path/hermite.h>
#include <grpl/system/coupled_drivetrain.h>
#include <grpl/util/vec.h>

#include <cmath>
#include <fstream>
#include <iostream>

using namespace grpl::profile;
using namespace grpl::path;
using namespace grpl::system;
using namespace grpl::units;
using namespace grpl;

template<typename profile_t, typename path_t>
void run_kinematics_test(std::string filename, coupled_drivetrain<path_t, profile_t> &cdt, profile_t &profile, path_t &path) {
    typename path_t::vector_t pos_l, pos_r, pos_c, vel_c;

    typename coupled_drivetrain<path_t, profile_t>::state state;
    state.done = false;

    std::ofstream outfile(filename + ".csv");
    std::ofstream outfile_sim(filename + "_simulated.csv");
    outfile << "path,t,x,y,d,v,a,angle,anglev\n";
    outfile_sim << "path,t,x,y,v,a\n";

    int i = 0;
    const char *titles[3] = { "left", "center", "right" };
    double dt = (1*ms).as(s);
    for (Time t = 0; !state.done; t+=dt) {
        state = cdt.generate(&path, &profile, state, t.as(s));
        if (t == 0*s) {
            // pos_l = column(state.l.k, 0);
            // pos_r = column(state.r.k, 0);
            // pos_c = column(state.c.k, 0);
            pos_l = state.l.k.col(0);
            pos_r = state.r.k.col(0);
            pos_c = state.c.k.col(0);
        }

        // Check conformity to coupled system constraints TODO: angles
        // EXPECT_NEAR((state.l.k[1] + state.r.k[1]) / 2, state.c.k[1], 0.0001);
        // EXPECT_NEAR((state.l.k[2] + state.r.k[2]) / 2, state.c.k[2], 0.0001);

        // TODO: check wheelbase position vectors are trackwidth apart
        // TODO: when moving to k = matrix, check they are parallel at any given time

        typename path_t::vector_t rv_a;
        // Check acceleration kinematics match with velocity kinematics
        // rv_a = robot_vel + vec_polar((state.l.k[2] + state.r.k[2]) / 2, state.a[0]) * (1*ms).as(s);
        // robot_vel = vec_polar((state.l.k[1] + state.r.k[2]) / 2, state.a[0]);
        // bool taok  = length(rv_a - robot_vel) < 0.0001; // TODO:
        // Check velocity kinematics match

        // acc_l = vec_polar(state.l.k[2], state.a[0]);
        // acc_r = vec_polar(state.r.k[2], state.a[0]);

        // // vel_l = vec_polar(state.l.k[1], state.a[0]);
        // // vel_r = vec_polar(state.r.k[1], state.a[0]);
        // vel_l += acc_l*dt;
        // vel_r += acc_r*dt;

        // vel_c += column(state.c.k, 2)*dt;
        vel_c = state.c.k.col(2) * dt;

        // TODO: Check this in the profile too, it might be a problem there.
        // pos_l += column(state.l.k, 1)*dt;
        // pos_r += column(state.r.k, 1)*dt;
        pos_l += state.l.k.col(1) * dt;
        pos_r += state.r.k.col(1) * dt;
        pos_c += vel_c*dt;

        // TODO: I have a feeling this is failing because the velocity and acceleration angles are not necessarily the
        // same as the path angle.
        // Yup, just verified by having the path be a straight line.

        outfile_sim << "left,"   << t.as(s) << "," << pos_l[0] << "," << pos_l[1] << "," << "0,0" << std::endl;
        outfile_sim << "right,"  << t.as(s) << "," << pos_r[0] << "," << pos_r[1] << "," << "0,0" << std::endl;
        outfile_sim << "center,"  << t.as(s) << "," << pos_c[0] << "," << pos_c[1] << "," << "0,0" << std::endl;

        // for (auto &it : { state.c, state.l, state.r }) {
        //     EXPECT_LE(it.k[1], cdt.get_limits()[1]);    // by being under the set derivative limit, we can imply the function is continuous
        //     EXPECT_LE(it.k[2], cdt.get_limits()[2]);
        //     EXPECT_GE(it.k[1], -cdt.get_limits()[1]);
        //     EXPECT_GE(it.k[2], -cdt.get_limits()[2]);
        // }

        for (auto &it : { state.l, state.c, state.r }) {
            // outfile << titles[(i++)%3] << "," 
            //     << it.t << "," << it.p[0] << "," << it.p[1] << "," << it.k[0] << "," << it.k[1] << "," << it.k[2] << "," 
            //     << state.a[0] << "," << state.a[1] << "\n";
            outfile << titles[(i++)%3] << "," << it.t << "," << it.k(0, 0) << "," << it.k(1, 0) << ","
                    << it.d << "," << it.k.col(1).norm() << "," << it.k.col(2).norm() << ","
                    << state.a[0] << "," << state.a[1] << std::endl;
        }
    }
}

TEST(System, Coupled) {
    using hermite_t = hermite<2>;
    using profile_t = trapezoidal;

    coupled_drivetrain<hermite_t, profile_t> cdt;
    cdt.apply_limit(1, 3); // 1st derivative (velocity) limit
    cdt.apply_limit(2, 4); // 2nd derivative (acceleration) limit

    cdt.set_trackwidth(0.5);
    // TODO: If the tangent of the first point tries to 'overrun' the second point when in a straight line this fails.
    // Need to find the critical values.
    hermite_t::waypoint     wp0 { vec_cart(0, 0), vec_polar(15, 0) },
                            wp1 { vec_cart(4, 4), vec_polar(15, 3.14159265/2) },
                            wp2 { vec_cart(0, 2), vec_cart(-5, 5) },
                            wp3 { vec_cart(6, 0), vec_polar(0, 0) };

    hermite_t hermite(wp0, wp2, 100000);

    profile_t profile;
    profile.set_timeslice(0.001);

    run_kinematics_test("coupled", cdt, profile, hermite);
}

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
//             ASSERT_LE(it.k[1], cdt.get_limits()[1]);    // by being under the set derivative limit, we can imply the function is continuous
//             ASSERT_LE(it.k[2], cdt.get_limits()[2]);
//         }
//         for (auto &it : { state.l, state.c, state.r }) {
//             outfile << titles[(i++)%3] << "," << it.t << "," << it.p[0] << "," << it.p[1] << "," << it.k[0] << "," << it.k[1] << "," << it.k[2] << "," << state.a[0] << "," << state.a[1] << "\n";
//         }
//     }
// }