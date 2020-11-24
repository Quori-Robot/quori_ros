#include "quori_holonomic_drive_controller/holonomic.hpp"

using namespace quori_holonomic_drive_controller;

#include <math.h>
#include <algorithm>
#include <iostream>

HolonomicCommand quori_holonomic_drive_controller::limit_acceleration(const HolonomicCommand &prev, const HolonomicCommand &current, const double limit)
{
  const double curr_vel = sqrt(pow(current.lin_x_vel, 2) + pow(current.lin_y_vel, 2));
  const double prev_vel = sqrt(pow(prev.lin_x_vel, 2) + pow(prev.lin_y_vel, 2));
  
  if (curr_vel - prev_vel > limit)
  {
    const double scale = (limit - prev_vel) / curr_vel;
    return {
      .lin_x_vel = current.lin_x_vel * scale,
      .lin_y_vel = current.lin_y_vel * scale,
      .ang_z_vel = current.ang_z_vel * scale,
    };
  }
  else if (curr_vel - prev_vel < -limit)
  {
    const double scale = (-limit + prev_vel) / prev_vel;
    return {
      .lin_x_vel = prev.lin_x_vel * scale,
      .lin_y_vel = prev.lin_y_vel * scale,
      .ang_z_vel = prev.ang_z_vel * scale
    };
  }

  return current;
}


DiffDriveCommand quori_holonomic_drive_controller::compute_ramsis_jacobian(const HolonomicCommand &command, double turret_pos, const HolonomicParams &params)
{
  const double sint = sin(turret_pos);
  const double cost = cos(turret_pos);
  
  const double b_sin = params.wheel_distance * sint;
  const double b_cos = params.wheel_distance * cost;
  const double a_sin = params.wheel_distance * sint;
  const double a_cos = params.wheel_distance * cost;
  
  const double wheel_distance_inv = 1.0 / params.wheel_distance;
  const double output_m1 = (command.lin_x_vel * (-b_sin - a_cos) + command.lin_y_vel * (b_cos - a_sin)) * wheel_distance_inv;
  const double output_m2 = (command.lin_x_vel * (-b_sin + a_cos) + command.lin_y_vel * (b_cos + a_sin)) * wheel_distance_inv;

  // Notice this is the negative of the listed jacobian. this is because the motor velocity is the negative of the turret velocity.
  const double output_mt = (-command.lin_x_vel * cost - command.lin_y_vel * sint) * wheel_distance_inv - command.ang_z_vel; 

  // Scale the velocities linearly if a maximuim was reached
  double scale = 1;
  
  if (abs(output_mt) > params.max_motor_turret_vel)
  {
    scale = std::min(params.max_motor_turret_vel / abs(output_mt), scale);
  }

  if (abs(output_m1) > params.max_motor_left_vel)
  {
    scale = std::min(params.max_motor_left_vel / abs(output_m1), scale);
  }

  if (abs(output_m2) > params.max_motor_right_vel)
  {
    scale = std::min(params.max_motor_right_vel / abs(output_m2), scale);
  }

  std::cout << "scale: " << scale << std::endl;

  return {
    .motor_left_vel = scale * output_m1 / params.wheel_radius,
    .motor_right_vel = scale * output_m2 / params.wheel_radius,
    .motor_turret_vel = scale * output_mt,
  };
}

std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::HolonomicParams &v)
{
  return o << "HolonomicParams {" << std::endl
    << "  wheel_distance: " << v.wheel_distance << " meters" << std::endl 
    << "  wheel_radius: " << v.wheel_distance << " meters" << std::endl 
    << "  max_motor_turret_vel: " << v.max_motor_turret_vel << " rad/s" << std::endl 
    << "  max_motor_left_vel: " << v.max_motor_left_vel << " m/s" << std::endl 
    << "  max_motor_right_vel: " << v.max_motor_right_vel << " m/s" << std::endl
    << "}"; 
}

std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::HolonomicCommand &v)
{
  return o << "HolonomicCommand {" << std::endl
    << "  Linear X: " << v.lin_x_vel << " m/s" << std::endl 
    << "  Linear Y: " << v.lin_y_vel << " m/s" << std::endl 
    << "  Angular Z: " << v.ang_z_vel << " rad/s" << std::endl 
    << "}";
}

std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::DiffDriveCommand &v)
{
  return o << "DiffDriveCommand {" << std::endl
    << "  Left Motor: " << v.motor_left_vel << " rad/s" << std::endl 
    << "  Right Motor: " << v.motor_right_vel << " rad/s" << std::endl 
    << "  Turret Motor: " << v.motor_turret_vel << " rad/s" << std::endl 
    << "}";
}