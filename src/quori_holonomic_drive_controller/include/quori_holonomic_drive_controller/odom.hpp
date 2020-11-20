#pragma once

#include <ros/time.h>
#include <iostream>

namespace quori_holonomic_drive_controller
{
  class Odom
  {
  public:
  Odom(const ros::Time &stamp, const double wheel_separation, const double wheel_radius);
    Odom update(const ros::Time &stamp, const double left_vel, const double right_vel, const double turret_pos) const;

    double getX() const;
    double getY() const;
    double getHeading() const;

    double getVelX() const;
    double getVelY() const;
    double getVelHeading() const;

    static Odom zero(const ros::Time &stamp, const double wheel_separation, const double wheel_radius);

  private:
    

    ros::Time stamp_;

    double wheel_separation_;
    double wheel_radius_;

    double x_;
    double y_;
    double heading_;
    double heading_offset_;

    double vel_x_;
    double vel_y_;
    double vel_heading_;
  };
}

std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::Odom &odom);