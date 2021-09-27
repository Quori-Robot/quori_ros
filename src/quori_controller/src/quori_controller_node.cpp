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

  bool generate_csvs = false;
  pnh.getParam("generate_csvs", generate_csvs);

  boost::asio::io_service service;
  std::vector<SerialDevice::Ptr> serial_devices;
  std::unique_ptr<std::ofstream> csv_file;
  Csv::Ptr csv;
  
  
  if (generate_csvs)
  {
    csv_file = std::make_unique<std::ofstream>("out.csv");
    if (!*csv_file) throw std::runtime_error("Could not open csv file");
    csv = Csv::open(*csv_file);
    std::cerr << "Opened CSV" << std::endl;
  }

  for (const auto &device : devices)
  {
    const auto serial_device = SerialDevice::open(service, device);
    if (csv) serial_device->attachSetPositionsCsv(csv);
    serial_devices.push_back(serial_device);
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