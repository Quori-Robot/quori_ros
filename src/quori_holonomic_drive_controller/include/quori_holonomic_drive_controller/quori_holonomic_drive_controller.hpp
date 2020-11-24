#pragma once


#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/controller.h>
#include <quori_holonomic_drive_controller/odometry.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>
#include "odom.hpp"

#include <boost/optional.hpp>

namespace quori_holonomic_drive_controller
{
  class QuoriHolonomicDriveController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    QuoriHolonomicDriveController();

    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    void update(const ros::Time &time, const ros::Duration &period);
    void starting(const ros::Time &time);
    void stopping(const ros::Time &time);

  private:
    std::string name_;

    // Odometry
    ros::Duration publish_period_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    ros::Time last_state_publish_time_;
    bool open_loop_;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    ros::Publisher marker_pub_;
    Odometry odometry_;
    Odom odom_;

    boost::optional<double> angle_offset_;


    // Command
    struct Commands
    {
      double lin_x;
      double lin_y;
      double ang;

      ros::Time stamp;

      Commands() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
    };

    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    // Joints
    hardware_interface::JointHandle left_joint_;
    hardware_interface::JointHandle right_joint_;
    hardware_interface::JointHandle turret_joint_;
    hardware_interface::JointHandle x_joint_;
    hardware_interface::JointHandle y_joint_;
    hardware_interface::JointHandle angle_joint_;
    hardware_interface::JointHandle mode_joint_;

    // Controller state requirements
    ros::Time time_previous_;


    /// Previous velocities from the encoders:
    double vel_x_previous_;
    double vel_y_previous_;
    double vel_angle_previous_;

    /// Previous velocities from the encoders:
    double vel_x_desired_previous_;
    double vel_y_desired_previous_;
    double vel_angle_desired_previous_;

    /// Wheel separation, wrt the midpoint of the wheel width:
    double wheel_separation_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

  private:
    void brake();
    void setOdomPubFields(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    void cmdVelCallback(const geometry_msgs::Twist &command);
    void publishWheelData(const ros::Time &time, const ros::Duration &period, Commands &curr_cmd, double wheel_separation, double left_wheel_radius, double right_wheel_radius);
  };
}