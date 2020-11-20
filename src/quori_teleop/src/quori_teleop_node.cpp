#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <cstdint>
#include <boost/optional.hpp>

#include <unordered_map>

enum class Joint : std::uint8_t {
  LeftArm1,
  LeftArm2,
  RightArm1,
  RightArm2,
  WaistHinge,
  BaseLeft,
  BaseRight,
  BaseTurret
};

namespace std
{
  template<>
  struct hash<Joint>
  {
    size_t operator ()(const Joint &joint) const noexcept
    {
      return hash<std::uint8_t>()(static_cast<std::uint8_t>(joint));
    }
  };
}

const static std::unordered_map<Joint, std::string> JOINT_NAMES {
  { Joint::BaseLeft, "base_left" },
  { Joint::BaseRight, "base_right" },
  { Joint::BaseTurret, "base_turret" },
  { Joint::LeftArm1, "left_arm_r1" },
  { Joint::LeftArm2, "left_arm_r2" },
  { Joint::RightArm1, "right_arm_r1" },
  { Joint::RightArm2, "right_arm_r2" },
  { Joint::WaistHinge, "waist_hinge" }
};

struct JointCommand
{
  JointCommand()
  {
    for (const auto &joint_name : JOINT_NAMES)
    {
      joints.insert({ joint_name.first, 0.0 });
    }
  }

  std::unordered_map<Joint, double> joints; 
} command;


geometry_msgs::Twist vel;
trajectory_msgs::JointTrajectory traj;

struct XboxControllerState
{
  // L, R, U, D
  bool dpad[4];
  double left_stick[2];
  double right_stick[2];

  bool left_trigger_1;
  bool right_trigger_1;

  double left_trigger_2;
  double right_trigger_2;

  bool a_button;

  static XboxControllerState fromJoy(const sensor_msgs::Joy::ConstPtr &msg)
  {
    XboxControllerState ret;
    ret.left_stick[0] = msg->axes[0];
    ret.left_stick[1] = msg->axes[1];
    ret.left_trigger_2 = msg->axes[2];
    ret.right_stick[0] = msg->axes[3];
    ret.right_stick[1] = msg->axes[4];
    ret.right_trigger_2 = msg->axes[5];


    ret.a_button = msg->buttons[0];
    ret.left_trigger_1 = msg->buttons[4];
    ret.right_trigger_1 = msg->buttons[5];

    return ret;
  }
};

sensor_msgs::JointState::ConstPtr joint_states;

void on_joint_states(const sensor_msgs::JointState::ConstPtr &msg)
{
  joint_states = msg;
}

double lookup_latest_position(const std::string &name)
{
  if (!joint_states) return 0;

  for (std::size_t i = 0; i < joint_states->name.size(); ++i)
  {
    if (joint_states->name[i] != name) continue;
    std::cout << name << " " << joint_states->position[i] << std::endl;
    return joint_states->position[i];
  }

  return 0;
}

void on_joy(const sensor_msgs::Joy::ConstPtr &msg)
{
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.angular.z = 0;
  

  traj.points.clear();

  XboxControllerState state = XboxControllerState::fromJoy(msg);

  boost::optional<double> left_arm_r1;
  boost::optional<double> left_arm_r2;
  boost::optional<double> right_arm_r1;
  boost::optional<double> right_arm_r2;
  boost::optional<double> waist_hinge;

  // Left Arm Control
  if (state.left_trigger_2 < 0.5)
  {
    // std::cout << "Left arm control " << state.left_stick[0] << std::endl;
    left_arm_r1 = state.left_stick[1] * 2 * (state.a_button ? 2 : 1);
    left_arm_r2 = state.left_stick[0] * 2 * (state.a_button ? 2 : 1);
  }
  // Right Arm Control
  else if (state.right_trigger_2 < 0.5)
  {
    // std::cout << "Right arm control " << state.left_stick[0] << std::endl;
    right_arm_r1 = state.left_stick[1] * 2 * (state.a_button ? 2 : 1);
    right_arm_r2 = state.left_stick[0] * 2 * (state.a_button ? 2 : 1);
  }
  // Base Control
  else if (state.left_trigger_1)
  {
    vel.linear.x = state.left_stick[1] / 8 * (state.a_button ? 2 : 1);
    vel.linear.y = -state.left_stick[0] / 8 * (state.a_button ? 2 : 1);
    vel.angular.z = -state.right_stick[0] / 2 * (state.a_button ? 2 : 1);
  }
  // Hinge Control
  else if (state.right_trigger_1)
  {
    waist_hinge = state.left_stick[1] / 3.5;
  }

  trajectory_msgs::JointTrajectoryPoint point;
  // std::cout << "cmd " << (lookup_latest_position("right_arm_r1") + (right_arm_r1 ? *right_arm_r1 : 0.0)) << std::endl;
  point.positions.push_back(lookup_latest_position("right_arm_r1") + (right_arm_r1 ? *right_arm_r1 : 0.0));
  point.positions.push_back(lookup_latest_position("right_arm_r2") + boost::get_optional_value_or(right_arm_r2, 0.0));
  point.positions.push_back(lookup_latest_position("left_arm_r1") + boost::get_optional_value_or(left_arm_r1, 0.0));
  point.positions.push_back(lookup_latest_position("left_arm_r2") + boost::get_optional_value_or(left_arm_r2, 0.0));
  point.positions.push_back(waist_hinge ? *waist_hinge : 0.0);
  point.time_from_start = ros::Duration(2);
  traj.points.push_back(point);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "quori_teleop");

  ros::NodeHandle nh;

  traj.joint_names.push_back("right_arm_r1");
  traj.joint_names.push_back("right_arm_r2");
  traj.joint_names.push_back("left_arm_r1");
  traj.joint_names.push_back("left_arm_r2");
  traj.joint_names.push_back("waist_hinge");

  const auto joy = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &on_joy);
  const auto joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &on_joint_states);
  const auto joint_traj_command = nh.advertise<trajectory_msgs::JointTrajectory>("/quori/joint_trajectory_controller/command", 1);
  // const auto joint_traj_command = nh.subscribe<control_msgs::JointTrajectoryControllerState>("/quori/joint_trajectory_controller/command", 1, on_j);
  const auto cmd_vel = nh.advertise<geometry_msgs::Twist>("/quori/base_controller/cmd_vel", 1);
  // const auto command_pub = nh.advertise<tajectory_msgs::JointTrajectory>("/joy", 1, &on_joy);

  ros::Rate rate(25.0);
  while (ros::ok())
  {
    ros::spinOnce();

    cmd_vel.publish(vel);
    
    traj.header.seq = traj.header.seq + 1;
    traj.header.stamp = ros::Time::now();
    joint_traj_command.publish(traj);

    rate.sleep();
  }

  return EXIT_SUCCESS;
}