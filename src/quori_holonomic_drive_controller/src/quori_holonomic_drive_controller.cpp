#include "quori_holonomic_drive_controller/quori_holonomic_drive_controller.hpp"
#include "quori_holonomic_drive_controller/holonomic.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Geometry>

using namespace quori_holonomic_drive_controller;

QuoriHolonomicDriveController::QuoriHolonomicDriveController()
  : name_("quori_holonomic_drive_controller")
  , odom_frame_id_("odom")
  , base_frame_id_("ramsis/wheel_axle")
  , enable_odom_tf_(true)
  , odom_(Odom::zero(ros::Time::now(), 0.0, 0.0))
  , cmd_vel_timeout_(0.5)
{
}

bool QuoriHolonomicDriveController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);

  controller_nh.param("open_loop", open_loop_, open_loop_);
  controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);

  controller_nh.param("max_motor_left_vel", holonomic_params_.max_motor_left_vel, 0.6);
  controller_nh.param("max_motor_right_vel", holonomic_params_.max_motor_right_vel, 0.6);
  controller_nh.param("max_motor_turret_vel", holonomic_params_.max_motor_turret_vel, 200.0 / 180.0 * M_PI);

  controller_nh.param("wheel_separation", holonomic_params_.wheel_distance, 0.12284075);
  controller_nh.param("wheel_radius", holonomic_params_.wheel_radius, 0.1524 / 2.0);

  setOdomPubFields(root_nh, controller_nh);

  left_joint_ = hw->getHandle("base_left");
  right_joint_ = hw->getHandle("base_right");
  turret_joint_ = hw->getHandle("base_turret");
  x_joint_ = hw->getHandle("base_x");
  y_joint_ = hw->getHandle("base_y");
  angle_joint_ = hw->getHandle("base_angle");
  base_mode_joint_ = hw->getHandle("base_mode");

  sub_command_ = controller_nh.subscribe("cmd_vel", 1, &QuoriHolonomicDriveController::cmdVelCallback, this);

  return true;
}

typedef Eigen::Transform<double, 3, Eigen::Affine> Transform3d;

void QuoriHolonomicDriveController::update(const ros::Time &time, const ros::Duration &period)
{
  // odometry_.setWheelParams(wheel_separation_, wheel_radius_, wheel_radius_);
  // odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);

  odom_ = odom_.update(time, left_joint_.getVelocity(), right_joint_.getVelocity(), turret_joint_.getPosition());

  if (!angle_offset_)
  {
    angle_offset_ = turret_joint_.getPosition();
  }
  const double heading = -odom_.getHeading() + turret_joint_.getPosition();
  
  const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(-odom_.getHeading()));

  if (last_state_publish_time_ + publish_period_ < time)
  {
    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info
    
    const double mapped_x = odom_.getX();
    const double mapped_y = odom_.getY();
    const double x = mapped_x * cos(heading) - mapped_y * sin(heading);
    const double y = mapped_x * sin(heading) + mapped_y * cos(heading);
    
    const Transform3d odom_frame = Eigen::Translation3d(-odom_.getX(), -odom_.getY(), 0.0)
      * Eigen::AngleAxisd(-odom_.getHeading(), Eigen::Vector3d::UnitZ());
    const Transform3d center_frame = odom_frame * Transform3d(Eigen::Translation3d(0.12, 0, 0));

    // Populate odom message and publish
    if (odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = -odom_.getX();
      odom_pub_->msg_.pose.pose.position.y = -odom_.getY();
      odom_pub_->msg_.pose.pose.orientation = orientation;
      odom_pub_->msg_.twist.twist.linear.x  = -odom_.getVelX();
      odom_pub_->msg_.twist.twist.linear.y  = -odom_.getVelY();
      odom_pub_->msg_.twist.twist.angular.z = -odom_.getVelHeading();
      odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_pub_->trylock())
    {
      geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      odom_frame.transform.translation.x = -odom_.getX();
      odom_frame.transform.translation.y = -odom_.getY();
      odom_frame.transform.rotation = orientation;
      // std::cout << "rotation " << tf::getYaw(orientation) << std::endl;
      tf_odom_pub_->unlockAndPublish();
    }
  }

  // MOVE ROBOT
  // Retreive current velocity command and time step:
  Commands curr_cmd = *(command_.readFromRT());
  const double dt = (time - curr_cmd.stamp).toSec();

  // Brake if cmd_vel has timeout:
  if (dt > cmd_vel_timeout_)
  {
    curr_cmd.lin_x = 0.0;
    curr_cmd.lin_y = 0.0;
    curr_cmd.ang = 0.0;
  }

  // Limit velocities and accelerations:
  const double cmd_dt(period.toSec());


  // limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
  // limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

  // last1_cmd_ = last0_cmd_;
  // last0_cmd_ = curr_cmd;

  // Publish limited velocity:
  /* if (publish_cmd_ && cmd_vel_pub_ && cmd_vel_pub_->trylock())
  {
    cmd_vel_pub_->msg_.header.stamp = time;
    cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin;
    cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
    cmd_vel_pub_->unlockAndPublish();
  }*/

  // The X and Y axes represented by the x_joint_ and y_joint_ do not rotate
  // with the base_link. We map the linear X vel and linear Y vel to these axes.

  x_joint_.setCommand(curr_cmd.lin_x);
  y_joint_.setCommand(curr_cmd.lin_y);
  angle_joint_.setCommand(curr_cmd.ang);
  base_mode_joint_.setCommand(1.0);

  // publishWheelData(time, period, curr_cmd, ws, lwr, rwr);
  time_previous_ = time;
}

void QuoriHolonomicDriveController::starting(const ros::Time &time)
{
  brake();

  // Register starting time used to keep fixed rate
  last_state_publish_time_ = time;
  time_previous_ = time;

  odom_ = Odom::zero(time, holonomic_params_.wheel_distance, holonomic_params_.wheel_radius);
}

void QuoriHolonomicDriveController::stopping(const ros::Time &time)
{
  brake();
}

void QuoriHolonomicDriveController::brake()
{
  x_joint_.setCommand(0.0);
  y_joint_.setCommand(0.0);
  angle_joint_.setCommand(0.0);
}

void QuoriHolonomicDriveController::setOdomPubFields(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = odom_frame_id_;
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0.;
  odom_pub_->msg_.pose.covariance = {
    static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
    0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
    0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
    0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
    0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
    0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])
  };

  odom_pub_->msg_.twist.twist.linear.y  = 0.;
  odom_pub_->msg_.twist.twist.linear.z  = 0.;
  odom_pub_->msg_.twist.twist.angular.x = 0.;
  odom_pub_->msg_.twist.twist.angular.y = 0.;
  odom_pub_->msg_.twist.covariance = {
    static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
    0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
    0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
    0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
    0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
    0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])
  };
    
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.075;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
}

void QuoriHolonomicDriveController::cmdVelCallback(const geometry_msgs::Twist &command)
{

  std::cout << "CMD VEL" << std::endl;
  if (!isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    return;
  }

  // check that we don't have multiple publishers on the command topic
  if (sub_command_.getNumPublishers() > 1)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
        << " publishers. Only 1 publisher is allowed. Going to brake.");
    brake();
    return;
  }

  if(!std::isfinite(command.angular.z) || !std::isfinite(command.angular.x))
  {
    ROS_WARN_THROTTLE(1.0, "Received NaN in velocity command. Ignoring.");
    return;
  }


  

  command_struct_.ang   = command.angular.z;
  command_struct_.lin_x = command.linear.x;
  command_struct_.lin_y = command.linear.y;
  command_struct_.stamp = ros::Time::now();

  // std::cout << "asdasd " << command_struct_.lin_x << std::endl;

  command_.writeFromNonRT (command_struct_);
  ROS_DEBUG_STREAM_NAMED(name_, "Added values to command. "
    << "Ang: "   << command_struct_.ang   << ", "
    << "Lin X: " << command_struct_.lin_x << ", "
    << "Lin Y: " << command_struct_.lin_y << ", "
    << "Stamp: " << command_struct_.stamp);
}

void QuoriHolonomicDriveController::publishWheelData(
  const ros::Time &time,
  const ros::Duration &period,
  Commands &curr_cmd,
  double wheel_separation,
  double left_wheel_radius,
  double right_wheel_radius
)
{
}

PLUGINLIB_EXPORT_CLASS(quori_holonomic_drive_controller::QuoriHolonomicDriveController, controller_interface::ControllerBase);