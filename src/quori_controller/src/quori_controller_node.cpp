#include <controller_manager/controller_manager.h>


#include "SerialDevice.hpp"
#include "Quori.hpp"

#include <array>
#include <chrono>

namespace
{
  

  
}

using namespace quori_controller;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "quori_controller");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::vector<std::string> devices;
  if (!pnh.getParam("devices", devices))
  {
    std::cerr << "Expected ~devices parameter" << std::endl;
    return EXIT_FAILURE;
  }

  boost::asio::io_service service;
  std::vector<SerialDevice::Ptr> serial_devices;
  for (const auto &device : devices)
  {
    serial_devices.push_back(SerialDevice::open(service, device));
  }

  
  Quori robot(nh, serial_devices);

  
  controller_manager::ControllerManager cm(&robot, nh);

  double rate_hz = pnh.param("rate", 100.0);

  ros::Time last_read;
  ros::Time last_update;
  ros::Time last_write;

  ros::Rate rate(rate_hz);
  while (ros::ok())
  {
    service.run_one();

    const ros::Time read_time = ros::Time::now();
    robot.read(read_time, read_time - last_read);
    last_read = read_time;

    const ros::Time update_time = ros::Time::now();
    cm.update(update_time, update_time - last_update);
    last_update = update_time;


    const ros::Time write_time = ros::Time::now();
    robot.write(write_time, write_time - last_write);
    last_write = write_time;

    rate.sleep();
  }
}