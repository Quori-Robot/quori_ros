#include <dynamic_reconfigure/server.h>

#include <quori_controller/CalibrateConfig.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "calibrate_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  dynamic_reconfigure::Server<calibrate::CalibrateConfig> server;


  pnh.advertise("/")

  quori_controller::CalibrateConfig config;
  config.base_turret_offset = 0.0;

  server.setCallback(config);

  ros::Rate rate(10);
  while (ros::ok())
  {

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}