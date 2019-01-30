#include "grpl/pf.h"

#include <gtest/gtest.h>

#include <fstream>

using namespace grpl::pf;

TEST(DCMotor, Model) {
  transmission::dc_motor motor{12.0, transmission::rpm_to_rad(5330), 2.7, 131, 2.41};

  std::cout << "TM: " << motor.torque(0, 1.0) << std::endl;

  std::ofstream outfile("motor.csv");
  outfile << "rpm,torque,current" << std::endl;

  double angular_speed = 0; // rad/s

  for (double angular_speed = 0; angular_speed < motor.free_speed(1.0); angular_speed += motor.free_speed(1.0) / 100) {
    double rpm = transmission::rad_to_rpm(angular_speed);
    double torque = motor.torque(angular_speed, 1.0);
    double current = motor.torque_to_current(torque);

    outfile << rpm << "," << torque << "," << current << std::endl;
  }

  outfile.close();
}