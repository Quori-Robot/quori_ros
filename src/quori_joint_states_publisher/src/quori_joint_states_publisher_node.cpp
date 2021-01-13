#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>
#include <string>
#include <unordered_map>
#include <boost/optional.hpp>
#include <boost/none.hpp>

enum class Joint {
  WaistHinge,
  TurretJoint,
  BaseX,
  BaseY,
  LeftArm1,
  LeftArm2,
  LeftArmLower,
  RightArmLower,
  RightArm1,
  RightArm2,
};

namespace std
{
  template <>
  struct hash<Joint>
  {
    size_t operator() (const Joint &t) const noexcept { return size_t(t); }
  };
}

const std::unordered_map<Joint, std::string> JOINT_NAMES {
  { Joint::WaistHinge, "waist_hinge" },
  { Joint::TurretJoint, "base_turret" },
  { Joint::LeftArm1, "left_arm_r1" },
  { Joint::LeftArm2, "left_arm_r2" },
  { Joint::RightArm1, "right_arm_r1" },
  { Joint::RightArm2, "right_arm_r2" },
  { Joint::LeftArmLower, "left_arm_lower_r1" },
  { Joint::RightArmLower, "right_arm_lower_r1" },
};

struct JointState {
  boost::optional<double> position;
  boost::optional<double> velocity;
  boost::optional<double> effort;

  static JointState from_position(const double position)
  {
    JointState ret;
    ret.position = position;
    return ret;
  }

  static JointState from_velocity(const double velocity)
  {
    JointState ret;
    ret.velocity = velocity;
    return ret;
  }
};

JointState merge(const JointState &lhs, const JointState &rhs)
{
  JointState ret = lhs;
  if (rhs.position) ret.position = rhs.position;
  if (rhs.velocity) ret.velocity = rhs.velocity;
  if (rhs.effort) ret.effort = rhs.effort;
  return ret;
}

struct JointStates {
  std::unordered_map<Joint, JointState> map;
  mutable std::mutex mut;

  JointStates()
  {
    // map.insert({ Joint::WaistHinge, JointState::from_position(0) });
    // map.insert({ Joint::TurretJoint, JointState::from_position(0) });
    // map.insert({ Joint::BaseX, JointState::from_position(0) });
    // map.insert({ Joint::BaseY, JointState::from_position(0) });
    // map.insert({ Joint::LeftArm1, JointState::from_position(0) });
    // map.insert({ Joint::LeftArm2, JointState::from_position(0) });
    map.insert({ Joint::LeftArmLower, JointState::from_position(3.14159 / 2) });
    // map.insert({ Joint::RightArm1, JointState::from_position(0) });
    // map.insert({ Joint::RightArm2, JointState::from_position(0) });
    map.insert({ Joint::RightArmLower, JointState::from_position(3.14159 / 2) });
  }

  sensor_msgs::JointState to_ros() const
  {
    sensor_msgs::JointState ret = base_;
    std::lock_guard<std::mutex> guard(mut);
    for (auto it = map.cbegin(); it != map.cend(); ++it)
    {
      ret.name.push_back(JOINT_NAMES.find(it->first)->second);
      ret.position.push_back(it->second.position.value_or(0.0));
      ret.velocity.push_back(it->second.velocity.value_or(0.0));
      ret.effort.push_back(it->second.effort.value_or(0.0));
    }
    return ret;
  }

  void set_base(const sensor_msgs::JointState &base)
  {
    std::lock_guard<std::mutex> guard(mut);
    base_ = base;
  }

  sensor_msgs::JointState base_;
} joint_states;

uint32_t seq;
ros::Publisher joint_states_pub;

void update_left_arm(const geometry_msgs::Vector3::ConstPtr &pos)
{
  std::lock_guard<std::mutex> guard(joint_states.mut);

  {
    const auto it = joint_states.map.find(Joint::LeftArm1);
    const JointState next = JointState::from_position(pos->x);
    joint_states.map[Joint::LeftArm1] = it != joint_states.map.cend() ? merge(it->second, next) : next;
  }

  {
    const auto it = joint_states.map.find(Joint::LeftArm2);
    const JointState next = JointState::from_position(pos->y);
    joint_states.map[Joint::LeftArm2] = it != joint_states.map.cend() ? merge(it->second, next) : next;
  }
}

void update_right_arm(const geometry_msgs::Vector3::ConstPtr &pos)
{
  std::lock_guard<std::mutex> guard(joint_states.mut);

  {
    const auto it = joint_states.map.find(Joint::RightArm1);
    const JointState next = JointState::from_position(pos->x);
    joint_states.map[Joint::RightArm1] = it != joint_states.map.cend() ? merge(it->second, next) : next;
  }

  {
    const auto it = joint_states.map.find(Joint::RightArm2);
    const JointState next = JointState::from_position(pos->y);
    joint_states.map[Joint::RightArm2] = it != joint_states.map.cend() ? merge(it->second, next) : next;
  }
}

void update_turret_joint(const std_msgs::Float32::ConstPtr &angle)
{
  std::lock_guard<std::mutex> guard(joint_states.mut);

  const auto it = joint_states.map.find(Joint::TurretJoint);
  const JointState next = JointState::from_position(angle->data);
  joint_states.map[Joint::TurretJoint] = it != joint_states.map.cend() ? merge(it->second, next) : next;
}

void update_base_vel(const geometry_msgs::Vector3::ConstPtr &vel)
{
  std::lock_guard<std::mutex> guard(joint_states.mut);

  {
    const auto it = joint_states.map.find(Joint::TurretJoint);
    const JointState next = JointState::from_velocity(vel->z);
    joint_states.map[Joint::TurretJoint] = it != joint_states.map.cend() ? merge(it->second, next) : next;
  }
}

void update_waist_pos(const geometry_msgs::Vector3::ConstPtr &pos)
{
  std::lock_guard<std::mutex> guard(joint_states.mut);

  std::cout << "Waist pos: " << pos->x << std::endl;

  {
    const auto it = joint_states.map.find(Joint::WaistHinge);
    const JointState next = JointState::from_position(pos->x);
    joint_states.map[Joint::WaistHinge] = it != joint_states.map.cend() ? merge(it->second, next) : next;
  }
}

void update_joint_states(const sensor_msgs::JointState::ConstPtr &s)
{
  std::lock_guard<std::mutex> guard(joint_states.mut);
  joint_states.base_ = *s;
}

void send_joint_states(sensor_msgs::JointState joint_states)
{
  joint_states.header.seq = seq++;
  joint_states.header.stamp = ros::Time::now();
  joint_states_pub.publish(joint_states);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "quori_joint_states_publisher_node");
  
  ros::NodeHandle nh;
  joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
  const auto joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/quori/joint_states", 1, &update_joint_states);

  const auto base_pos = nh.subscribe<std_msgs::Float32>("/quori/base/pos_status", 1, &update_turret_joint);
  const auto base_vel = nh.subscribe<geometry_msgs::Vector3>("/quori/base/vel_status", 1, &update_base_vel);
  const auto waist_pos = nh.subscribe<geometry_msgs::Vector3>("/quori/waist/pos_status", 1, &update_waist_pos);
  const auto left_arm_pos = nh.subscribe<geometry_msgs::Vector3>("/quori/arm_left/pos_status", 1, &update_left_arm);
  const auto right_arm_pos = nh.subscribe<geometry_msgs::Vector3>("/quori/arm_right/pos_status", 1, &update_right_arm);

  // TODO: Waist hinge

  ros::Rate rate(50.0);
  while(ros::ok())
  {
    ros::spinOnce();
    send_joint_states(joint_states.to_ros());
    rate.sleep();
  }
}
