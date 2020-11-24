#pragma once

#include <iostream>

namespace quori_holonomic_drive_controller
{
  struct HolonomicParams
  {
    // This distance in meters between the center of the two wheels
    double wheel_distance;
    double wheel_radius;

    double max_motor_turret_vel;
    double max_motor_left_vel;
    double max_motor_right_vel;
  };

  struct HolonomicCommand
  {
    double lin_x_vel;
    double lin_y_vel;
    double ang_z_vel;

  };

  HolonomicCommand limit_acceleration(const HolonomicCommand &prev, const HolonomicCommand &current, const double limit);


  struct DiffDriveCommand
  {
    // Left motor velocity in rad/s
    double motor_left_vel;

    // Right motor velocity in rad/s
    double motor_right_vel;

    // Turret motor velocity in rad/s
    double motor_turret_vel;
  };

  DiffDriveCommand compute_ramsis_jacobian(const HolonomicCommand &command, double turret_pos, const HolonomicParams &params);

}


std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::HolonomicParams &v);
std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::HolonomicCommand &v);
std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::DiffDriveCommand &v);