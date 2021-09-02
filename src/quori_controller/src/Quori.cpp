#include "Quori.hpp"

#include <iostream>
#include <chrono>


using namespace quori_controller;

#define PI 3.1415927
#define TAU 6.2831853
#define WHEEL_RADIUS 0.075

const double wheel_circumference = TAU * WHEEL_RADIUS;

Quori::Quori(ros::NodeHandle &nh, const std::vector<SerialDevice::Ptr> &devices)
  : nh_(nh)
  , base_vel_pub_(nh_.advertise<geometry_msgs::Vector3>("/quori/base/cmd_diff", 1))
  , base_holo_vel_pub_(nh_.advertise<geometry_msgs::Vector3>("/quori/base/cmd_holo", 1))
  , base_vel_status_(nh_.subscribe("/quori/base/vel_status", 1, &Quori::on_base_vel_status_, this))
  , base_turret_pos_(nh_.subscribe("/quori/base/pos_status", 1, &Quori::on_base_turret_pos_, this))
  , devices_(devices)
  , max_device_joints_(0)
  , device_joint_buffer_(nullptr)
{
  ros::NodeHandle pnh("~");
  // std::cout << "quori" << std::endl;
  // Iterate through each serial device (Arduino) we've been given
  for (auto it = devices_.cbegin(); it != devices_.cend(); ++it)
  {
    // Initialize the serial connection
    SerialDevice::InitializationInfo info = (*it)->initialize();
    
    std::cout << (*it)->getName() << ":" << std::endl;

    device_states_[*it] = quori::message::States();

    std::size_t last_index = joints_.size() - 1;

    std::vector<std::size_t> device_joint_indices;
    // Iterate through the joints this serial device exposes
    for (auto jit = info.joints.cbegin(); jit != info.joints.cend(); ++jit)
    {
      const std::string &joint_name = jit->name;
      const Joint::Mode &joint_mode = jit->mode;

      std::cout << joint_name << std::endl;

      // Create a memory region for its state and command
      joints_.push_back(std::make_shared<Joint>());

      double home_cmd = 0.0;
      pnh.param("home_cmds/" + joint_name, home_cmd, home_cmd);

      std::cout << joint_name << " home cmd " << home_cmd << std::endl;
      
      const Joint::Ptr &joint = joints_.back();

      joint->command = home_cmd;

      joint->name = joint_name;
      joint->mode = joint_mode;

      const std::size_t index = joints_.size() - 1;
      // Store the index we stored this joint at
      joint_indices_[joint_name] = index;
      device_joint_indices.push_back(index);

      hardware_interface::JointStateHandle state_handle(joint_name, &joint->position, &joint->velocity, &joint->effort);
      state_interface_.registerHandle(state_handle);


      hardware_interface::JointHandle handle(state_handle, &joint->command);
      
      switch (joint_mode)
      {
        case Joint::Mode::Position:
        {
          position_interface_.registerHandle(handle);
          break;
        }
        case Joint::Mode::Velocity:
        {
          velocity_interface_.registerHandle(handle);
          break;
        }
      }
      
    }

    device_joints_[*it] = device_joint_indices;
    max_device_joints_ = std::max(max_device_joints_, device_joint_indices.size());
  }


  {
    base_turret_ = std::make_shared<Joint>("base_turret");
    hardware_interface::JointStateHandle state_handle(base_turret_->name, &base_turret_->position, &base_turret_->velocity, &base_turret_->effort);
    state_interface_.registerHandle(state_handle);
    hardware_interface::JointHandle handle(state_handle, &base_turret_->command);
    velocity_interface_.registerHandle(handle);
  }

  {
    base_left_ = std::make_shared<Joint>("base_left");
    hardware_interface::JointStateHandle state_handle(base_left_->name, &base_left_->position, &base_left_->velocity, &base_left_->effort);
    state_interface_.registerHandle(state_handle);
    hardware_interface::JointHandle handle(state_handle, &base_left_->command);
    velocity_interface_.registerHandle(handle);
  }
  
  {
    base_right_ = std::make_shared<Joint>("base_right");
    hardware_interface::JointStateHandle state_handle(base_right_->name, &base_right_->position, &base_right_->velocity, &base_right_->effort);
    state_interface_.registerHandle(state_handle);
    hardware_interface::JointHandle handle(state_handle, &base_right_->command);
    velocity_interface_.registerHandle(handle);
  }

  {
    base_x_ = std::make_shared<Joint>("base_x");
    hardware_interface::JointStateHandle state_handle(base_x_->name, &base_x_->position, &base_x_->velocity, &base_x_->effort);
    hardware_interface::JointHandle handle(state_handle, &base_x_->command);
    velocity_interface_.registerHandle(handle);
  }

  {
    base_y_ = std::make_shared<Joint>("base_y");
    hardware_interface::JointStateHandle state_handle(base_y_->name, &base_y_->position, &base_y_->velocity, &base_y_->effort);
    hardware_interface::JointHandle handle(state_handle, &base_y_->command);
    velocity_interface_.registerHandle(handle);
  }

  {
    base_angle_ = std::make_shared<Joint>("base_angle");
    hardware_interface::JointStateHandle state_handle(base_angle_->name, &base_angle_->position, &base_angle_->velocity, &base_angle_->effort);
    hardware_interface::JointHandle handle(state_handle, &base_angle_->command);
    velocity_interface_.registerHandle(handle);
  }

  {
    base_mode_ = std::make_shared<Joint>("base_mode");
    hardware_interface::JointStateHandle state_handle(base_mode_->name, &base_mode_->position, &base_mode_->velocity, &base_mode_->effort);
    hardware_interface::JointHandle handle(state_handle, &base_mode_->command);
    velocity_interface_.registerHandle(handle);
  }
  
  registerInterface(&state_interface_);
  registerInterface(&position_interface_);
  registerInterface(&velocity_interface_);

  // nh.negotiateTopics();
}

Quori::~Quori()
{
  delete[] device_joint_buffer_;
}

void Quori::read(const ros::Time &time, const ros::Duration &period)
{
  

  const auto start = std::chrono::system_clock::now();
  for(auto it = devices_.cbegin(); it != devices_.cend(); ++it)
  {
    // std::cout << "get state " << (*it)->getName() << std::endl;
    quori::message::States &state = device_states_[*it];
    (*it)->getState(state);
    const auto &indices = device_joints_[*it];

    std::size_t state_index = 0;
    for (auto iit = indices.cbegin(); iit != indices.cend(); ++iit, ++state_index)
    {
      joints_[*iit]->position = state.measured[state_index];
      // joints_[*iit]->velocity = state.velocities[state_index];
      // joints_[*iit]->effort = state.efforts[state_index];
    }
  }

  if (base_vel_ && !last_read_.is_zero())
  {
    base_left_->velocity = -(*base_vel_)->x / WHEEL_RADIUS;
    base_right_->velocity = (*base_vel_)->y / WHEEL_RADIUS;
    base_turret_->velocity = (*base_vel_)->z / 360 * TAU;

    const double delta = (time - last_read_).toSec();
    base_left_->position += base_left_->velocity * delta;
    base_right_->position += base_right_->velocity * delta;
  }

  last_read_ = time;

  const auto end = std::chrono::system_clock::now();
  // std::cout << "Read took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

void Quori::write(const ros::Time &time, const ros::Duration &period)
{
  if (!device_joint_buffer_) device_joint_buffer_ = new double[max_device_joints_];

  const auto start = std::chrono::system_clock::now();

  for (auto it = device_joints_.cbegin(); it != device_joints_.cend(); ++it)
  {
    std::size_t i = 0;
    
    // Collect the commands for all joints this device controls
    for (auto jit = it->second.cbegin(); jit != it->second.cend(); ++jit)
    {
      // std::cout << joints_[*jit]->command << std::endl;
      device_joint_buffer_[i++] = joints_[*jit]->command;
    }

    

    it->first->setPositions(device_joint_buffer_, i);
  }

  if (base_mode_->command >= 0.5)
  {
    // std::cout << "Holonomic mode " << base_x_->command << " m/s, " << base_y_->command << " m/s, " << base_angle_->command << " rad/s" << std::endl;
    geometry_msgs::Vector3 base_holo_vel;
    base_holo_vel.x = base_x_->command;
    base_holo_vel.y = -base_y_->command;
    base_holo_vel.z = base_angle_->command;
    base_holo_vel_pub_.publish(base_holo_vel);
  }
  else
  {
    geometry_msgs::Vector3 base_vel;
    base_vel.x = WHEEL_RADIUS * -base_left_->command;
    base_vel.y = WHEEL_RADIUS * base_right_->command;
    base_vel.z = base_turret_->command / TAU * 360;
    // std::cout << "Z " << base_vel.z << std::endl;
    base_vel_pub_.publish(base_vel);  
  }
  

  

  const auto end = std::chrono::system_clock::now();
  // std::cout << "Write took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

}

void Quori::on_base_vel_status_(const geometry_msgs::Vector3::ConstPtr &msg)
{
  base_vel_ = msg;
}

void Quori::on_base_turret_pos_(const std_msgs::Float32::ConstPtr &msg)
{
  if (!base_turret_) return;
  
  // The received position is in negative revolutions, convert to radians
  base_turret_->position = -msg->data * TAU;

  // The position appears to be incorrect by a constant offset of 45 degrees
  // TODO: Verify this is the case on all quoris
  base_turret_->position += TAU / 8;
}