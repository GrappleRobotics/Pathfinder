#include <gtest/gtest.h>
#include <grpl/profile/trapezoidal.h>
#include <grpl/path/hermite.h>
#include <grpl/system/coupled_drivetrain.h>
#include <grpl/util/vec.h>

#include <grpl/units.h>

#include <cmath>
#include <fstream>
#include <iostream>

using namespace grpl::profile;
using namespace grpl::path;
using namespace grpl::system;
using namespace grpl::units;
using namespace grpl;

void write_vecinfo(std::ofstream &out, std::string path, double t, hermite<2>::vector_t p, hermite<2>::vector_t v, hermite<2>::vector_t a, double angle, double anglev, double curvature_radius, hermite<2>::vector_t curvcenter) {
    out << path << ", " << t << "," << p[0] << "," << p[1] << "," << v[0] << "," << v[1] << "," << a[0] << "," << a[1] << "," << angle << "," << anglev << "," << curvature_radius << "," << curvcenter[0] << "," << curvcenter[1] << "\n";
}

template<typename profile_t, typename path_t>
void run_kinematics_test(std::string filename, coupled_drivetrain<path_t, profile_t> &cdt, profile_t &profile, path_t &path) {
    typename path_t::vector_t pos_c_v, pos_c_a, vel_c_a, pos_l, pos_c, pos_r, vel_l, vel_c, vel_r, vel_c_p, acc_c_p;

    typename coupled_drivetrain<path_t, profile_t>::state state, lastState;
    state.done = false;

    std::ofstream outfile(filename + ".csv");
    std::ofstream outfile_sim(filename + "_simulated.csv");
    std::ofstream outfile_vecinfo(filename + "_vecinfo.csv");
    outfile << "path,t,x,y,d,v,a,angle,anglev\n";
    outfile_sim << "path,t,x,y,v,a\n";
    outfile_vecinfo << "path,t,x,y,vx,vy,ax,ay,angle,anglev,curvature_radius,curvature_x,curvature_y\n";

    int i = 0;
    const char *titles[3] = { "left", "center", "right" };
    double dt = (1*ms).as(s);
    for (Time t = 0; !state.done && t < 10*s; t+=dt) {
        state = cdt.generate(&path, &profile, state, t.as(s));
        if (t == 0*s) {
            pos_c_v = pos_c_a = state.c.k.col(0);
            pos_c = state.c.k.col(0);
            pos_l = state.l.k.col(0);
            pos_r = state.r.k.col(0);
            vel_c_a = { 0, 0 };
            vel_c_p = { 0, 0 };
        } else {
            typename path_t::vector_t last_vel = vel_c_p;
            vel_c_p = (state.c.k.col(0) - lastState.c.k.col(0)) / dt;
            acc_c_p = (vel_c_p - last_vel) / dt;
        }

        vel_l = state.l.k.col(1);
        vel_r = state.r.k.col(1);
        vel_c = state.c.k.col(1);

        pos_l += vel_l * dt;
        pos_r += vel_r * dt;
        pos_c += vel_c * dt;

        // TODO: I have a feeling this is failing because the velocity and acceleration angles are not necessarily the
        // same as the path angle.
        // Yup, just verified by having the path be a straight line.

        typename path_t::vector_t unit_angle = vec_polar(1, state.a[0]);

        outfile_sim << "left,"   << t.as(s) << "," << pos_l[0] << "," << pos_l[1] << "," << vel_l.dot(unit_angle) << ",0" << std::endl;
        outfile_sim << "right,"  << t.as(s) << "," << pos_r[0] << "," << pos_r[1] << "," << vel_r.dot(unit_angle) << ",0" << std::endl;
        outfile_sim << "center,"  << t.as(s) << "," << pos_c[0] << "," << pos_c[1] << "," << vel_c.dot(unit_angle) << ",0" << std::endl;

        for (auto &it : { state.l, state.c, state.r }) {
            // outfile << titles[(i++)%3] << "," 
            //     << it.t << "," << it.p[0] << "," << it.p[1] << "," << it.k[0] << "," << it.k[1] << "," << it.k[2] << "," 
            //     << state.a[0] << "," << state.a[1] << "\n";
            outfile << titles[(i++)%3] << "," << it.t << "," << it.k(0, 0) << "," << it.k(1, 0) << ","
                    << it.d << "," << it.k.col(1).dot(unit_angle) << "," << it.k.col(2).dot(unit_angle) << ","
                    << state.a[0] << "," << state.a[1] << std::endl;

            // outfile_vecinfo << titles[(i++)%3] << "," << it.t << "," << it.k(0, 0) << "," << it.k(1, 0) << ","
            //         << it.d << "," << it.k.col(1)[0] << "," << it.k.col(1)[1] << "," << it.k.col(2)[0] << "," << it.k.col(2)[1] << ","
            //         << state.a[0] << "," << state.a[1] << std::endl;
        }

        pos_c_v += state.c.k.col(1) * dt;
        vel_c_a += state.c.k.col(2)*dt;
        pos_c_a += vel_c*dt;

        double radius = 1.0 / state.curvature;

        std::cout << state.c.k.col(2).dot(vec_polar(1, state.a[0])) << "," << acc_c_p.dot(vec_polar(1, state.a[0])) << std::endl;

        hermite<2>::vector_t curv_center = state.c.k.col(0) + vec_polar(radius, state.a[0] + (state.a[1] > 0 ? 1 : -1)*PI/2.0);
        write_vecinfo(outfile_vecinfo, "posrel", t.as(s), state.c.k.col(0), vel_c_p, acc_c_p, state.a[0], state.a[1], radius, curv_center);

        curv_center = pos_c_v + vec_polar(radius, state.a[0] + (state.a[1] > 0 ? 1 : -1)*PI/2.0);
        write_vecinfo(outfile_vecinfo, "velrel", t.as(s), pos_c_v, state.c.k.col(1), state.c.k.col(2), state.a[0], state.a[1], radius, curv_center);

        curv_center = pos_c_a + vec_polar(radius, state.a[0] + (state.a[1] > 0 ? 1 : -1)*PI/2.0);
        write_vecinfo(outfile_vecinfo, "accrel", t.as(s), pos_c_a, vel_c_a, state.c.k.col(2), state.a[0], state.a[1], radius, curv_center);

        lastState = state;
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
    profile.set_timeslice(0);

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