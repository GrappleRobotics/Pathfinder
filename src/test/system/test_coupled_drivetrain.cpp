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

TEST(System, Coupled) {
    using hermite_t = hermite<2>;
    using profile_t = trapezoidal1;

    coupled_drivetrain1<hermite_t, profile_t> cdt;
    cdt.apply_limit(1, 3); // 1st derivative (velocity) limit
    cdt.apply_limit(2, 4); // 2nd derivative (acceleration) limit

    cdt.set_trackwidth(0.5);
    hermite_t::waypoint     wp0 { vec_cart(0, 0), vec_polar(5, 0) },
                            wp1 { vec_cart(4, 4), vec_polar(5, 3.14159265/2) };

    hermite_t hermite(wp0, wp1, 100000);

    profile_t profile;
    profile.set_timeslice(0.001);

    coupled_drivetrain1<hermite_t, profile_t>::state state;
    state.done = false;

    std::ofstream outfile("coupled.csv");   
    outfile << "path,t,x,y,d,v,a,angle,anglev\n";

    int i = 0;
    const char *titles[3] = { "left", "center", "right" };
    for (Time t = 0; !state.done; t+=1*ms) {
        state = cdt.generate(&hermite, &profile, state, t.as(s));
        for (auto &it : { state.c, state.l, state.r }) {
            ASSERT_LE(it.k[1], cdt.get_limits()[1]);    // by being under the set derivative limit, we can imply the function is continuous
            ASSERT_LE(it.k[2], cdt.get_limits()[2]);
        }
        for (auto &it : { state.l, state.c, state.r }) {
            outfile << titles[(i++)%3] << "," << it.t << "," << it.p[0] << "," << it.p[1] << "," << it.k[0] << "," << it.k[1] << "," << it.k[2] << "," << state.a[0] << "," << state.a[1] << "\n";
        }
    }
}

TEST(System, CoupledMulti) {
    using hermite_t = hermite<2>;
    using path_t = wrapper_path_set<hermite_t, 2>;
    using profile_t = trapezoidal1;

    coupled_drivetrain1<path_t, profile_t> cdt;
    cdt.apply_limit(1, 3); // 1st derivative (velocity) limit
    cdt.apply_limit(2, 4); // 2nd derivative (acceleration) limit

    cdt.set_trackwidth(0.5);
    hermite_t hermites[3];

    hermite_t::waypoint wps[4] {
        { hermite_t::vector_t{ 2, 2 }, hermite_t::vector_t{ 5, 0 } },
        { hermite_t::vector_t{ 3, 5 }, hermite_t::vector_t{ 0, 5 } },
        { hermite_t::vector_t{ 5, 7 }, hermite_t::vector_t{ 2, 2 } },
        { hermite_t::vector_t{ 7, 9 }, hermite_t::vector_t{ 5, -5 } }
    };
    hermite_factory::generate(&wps[0], 4, &hermites[0], 3);
    wrapper_path_set<hermite_t, 2> p(&hermites[0], 3);

    profile_t profile;
    profile.set_timeslice(0.001);

    coupled_drivetrain1<path_t, profile_t>::state state;
    state.done = false;

    std::ofstream outfile("coupled_multi.csv");
    outfile << "path,t,x,y,d,v,a,angle,anglev\n";

    int i = 0;
    const char *titles[3] = { "left", "center", "right" };
    for (Time t = 0; !state.done; t+=1*ms) {
        state = cdt.generate(&p, &profile, state, t.as(s));
        for (auto &it : { state.c, state.l, state.r }) {
            ASSERT_LE(it.k[1], cdt.get_limits()[1]);    // by being under the set derivative limit, we can imply the function is continuous
            ASSERT_LE(it.k[2], cdt.get_limits()[2]);
        }
        for (auto &it : { state.l, state.c, state.r }) {
            outfile << titles[(i++)%3] << "," << it.t << "," << it.p[0] << "," << it.p[1] << "," << it.k[0] << "," << it.k[1] << "," << it.k[2] << "," << state.a[0] << "," << state.a[1] << "\n";
        }
    }
}