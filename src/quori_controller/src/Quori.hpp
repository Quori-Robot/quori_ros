#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

#include "SerialDevice.hpp"
#include "Joint.hpp"

#include "message.hpp"

#include <map>
#include <unordered_map>

#include <vector>
#include <boost/optional.hpp>

#include <ros/time.h>

namespace quori_controller
{
  class Quori : public hardware_interface::RobotHW
  {
  public:
    Quori(ros::NodeHandle &nh, const std::vector<SerialDevice::Ptr> &devices);
    ~Quori();

    virtual void read(const ros::Time &time, const ros::Duration &period);
    void write();
    virtual void write(const ros::Time &time, const ros::Duration &period);

  private:
    void on_base_vel_status_(const geometry_msgs::Vector3::ConstPtr &msg);
    void on_base_turret_pos_(const std_msgs::Float32::ConstPtr &msg);

    

    ros::NodeHandle &nh_;
    ros::Publisher base_vel_pub_;
    ros::Publisher base_holo_vel_pub_;
    ros::Publisher base_offset_pub_;
    ros::Subscriber base_vel_status_;
    ros::Subscriber base_turret_pos_;

    std::vector<SerialDevice::Ptr> devices_;
    std::map<SerialDevice::Ptr, quori::message::States> device_states_;

    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::PositionJointInterface position_interface_;
    hardware_interface::VelocityJointInterface velocity_interface_;

    std::map<SerialDevice::Ptr, std::vector<std::size_t>> device_joints_;
    std::size_t max_device_joints_;
    double *device_joint_buffer_;

    float base_offset_;


    std::unordered_map<std::string, std::size_t> joint_indices_;
    std::vector<Joint::Ptr> joints_;

    Joint::Ptr base_turret_;
    Joint::Ptr base_left_;
    Joint::Ptr base_right_;

    Joint::Ptr base_x_;
    Joint::Ptr base_y_;
    Joint::Ptr base_angle_;
    Joint::Ptr base_mode_;

    boost::optional<geometry_msgs::Vector3::ConstPtr> base_vel_;

    ros::Time last_read_;
  };
}

