#include <quori_holonomic_drive_controller/odom.hpp>

#include <cmath>

#define TAU (2.0 * M_PI)

using namespace quori_holonomic_drive_controller;

Odom Odom::update(const ros::Time &stamp, const double left_vel, const double right_vel, const double turret_pos) const
{
  const double v_left = left_vel * wheel_radius_; // rad/s -> m/s
  const double v_right = right_vel * wheel_radius_; // rad/s -> m/s

  const double delta = (stamp - stamp_).toSec(); // s
  // std::cout << v_left << ", " << v_right << ", " << turret_pos << ", " << delta << std::endl;

  Odom ret(stamp, wheel_separation_, wheel_radius_);

  const double v_avg = (v_left + v_right) / 2;

  if (v_right == v_left)
  {
    ret.x_ = x_ + v_avg * cos(heading_) * delta;
    ret.y_ = y_ + v_avg * sin(heading_) * delta;
    ret.heading_ = heading_;
    ret.heading_offset_ = turret_pos;
    ret.vel_x_ = v_avg;
    ret.vel_y_ = v_avg;
    ret.vel_heading_ = 0.;

    return ret;
  }


  const double R = wheel_separation_ / 2 * (v_left + v_right) / (v_right - v_left);
  const double w = (v_right - v_left) / wheel_separation_;

  const double icc_x = x_ - R * sin(heading_);
  const double icc_y = y_ - R * cos(heading_);

  
  ret.vel_heading_ = w;

  ret.x_ = cos(w * delta) * (x_ - icc_x) - sin(w * delta) * (y_ - icc_y) + icc_x;
  ret.y_ = sin(w * delta) * (x_ - icc_x) + cos(w * delta) * (y_ - icc_y) + icc_y;
  ret.heading_ = heading_ + ret.vel_heading_ * delta;
  ret.heading_offset_ = turret_pos;

  ret.vel_x_ = (ret.x_ - x_) / delta;
  ret.vel_y_ = (ret.y_ - y_) / delta;

  return ret;
}

double Odom::getX() const
{
  return x_;
}

double Odom::getY() const
{
  return y_;
}

double Odom::getHeading() const
{
  return heading_;
}

double Odom::getVelX() const
{
  return vel_x_;
}

double Odom::getVelY() const
{
  return vel_y_;
}

double Odom::getVelHeading() const
{
  return vel_heading_;
}

Odom Odom::zero(const ros::Time &stamp, const double wheel_separation, const double wheel_radius)
{
  return Odom(stamp, wheel_separation, wheel_radius);
}

Odom::Odom(const ros::Time &stamp, const double wheel_separation, const double wheel_radius)
  : stamp_(stamp)
  , wheel_separation_(wheel_separation)
  , wheel_radius_(wheel_radius)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , heading_offset_(0.0)
  , vel_x_(0.0)
  , vel_y_(0.0)
  , vel_heading_(0.0)
{
}

std::ostream &operator <<(std::ostream &o, const quori_holonomic_drive_controller::Odom &odom)
{
  return o << "Odom {" << std::endl
           << "  vel_x (m/s): " << odom.getVelX() << std::endl
           << "  vel_y (m/s): " << odom.getVelY() << std::endl
           << "  vel_heading (rad/s): " << odom.getVelHeading() << std::endl
           << "  x (m): " << odom.getX() << std::endl
           << "  y (m): " << odom.getY() << std::endl
           << "  heading (rad): " << odom.getHeading() << std::endl
           << "}";
}
