#include <gtest/gtest.h>

#include "grpl/model/cdt.h"

#include <fstream>

using namespace grpl;
using namespace grpl::model;

TEST(CDT, Graph) {
    Eigen::Matrix<double, 2, 1> _position{0, 0}, vel{0, 0};
    double _heading = 0, _omega = 0;

    // CIM, 13:1
    unsigned int num_motor = 1;
    motor_model motor(12.0, 5330*(2*PI/60.0) / 13.0, 2.7*num_motor, 131*num_motor, 2.41*num_motor*13.0);
    cdt_model model(motor, 12.0, 0.1524, 54.0, 1.0);
    model.set_max_current(40);

    std::ofstream out("cdt_model.csv");
    out << "t,al,vl,il,voltl,ar,vr,ir,voltr,px,py,omega,wl,wr\n";

    double dt = 0.01;
    for (double t = 0; t < 5; t += dt) {
        model.set_vel(vel);
        auto limits = model.calculate_limits_2(0.5, dt);

        out << t << "," 
            << limits.accel_limits(0, 0) << "," << vel(0,0) << "," 
            << limits.current_limits(0, 0) << "," << limits.voltages(0, 0) << ","
            << limits.accel_limits(1, 0) << "," << vel(1,0) << "," 
            << limits.current_limits(1, 0) << "," << limits.voltages(1, 0) << ","
            << _position(0, 0) << "," << _position(1, 0) << "," << _omega << ","
            << limits.wheel_vels(0, 0) << "," << limits.wheel_vels(1, 0) << std::endl;

        vel(0, 0) += limits.accel_limits(0, 0)*dt;
        vel(1, 0) += limits.accel_limits(1, 0)*dt;

        double sc = (vel(1, 0) + vel(0, 0)) / 2;
        _omega = (vel(1, 0) - vel(0, 0)) / (model.get_trackwidth());
        _heading += _omega*dt;

        Eigen::Matrix<double, 2, 1> vc { cos(_heading), sin(_heading) };
        _position += sc * vc * dt;
    }
}