#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main (int argc, char **argv)
{
  if (argc < 3) {
    std::cerr << "Usage: testvideo <rate in Hz> <list of images>" << std::endl;
    return 0;
  }

  float rate = std::stof(argv[1]); // Hz
  char **image_names = &argv[2];
  std::vector<cv::Mat> images;
  int num_images = argc-2;

  //std::cerr << "num images " << num_images << std::endl;
  for (int i=0; i<num_images; i++) {
    std::cerr << "Reading image " << image_names[i] << std::endl;
    images.push_back(cv::imread(image_names[i], cv::IMREAD_COLOR));
  }

  ros::init(argc, argv, "testvideo");
  ros::NodeHandle rosnode;

  ros::Publisher image_pub =
    rosnode.advertise<sensor_msgs::Image>("quori/face_image", 1);
  std::vector<sensor_msgs::ImagePtr> ros_images;

  for (int i=0; i<num_images; i++) {
    std::cerr << "Converting " << image_names[i] << " to ROS msg"<< std::endl;
    ros_images.push_back(cv_bridge::CvImage(std_msgs::Header(),
					    "bgr8", images[i]).toImageMsg());
  }

  unsigned int period = 1000000/rate;
  for (int nimage=0; rosnode.ok(); nimage = (nimage+1)%num_images) {
    std::cerr << "Sending image " << image_names[nimage] << std::endl;
    image_pub.publish(ros_images[nimage]);
    usleep(period);
  }

#if 0
    std::string path="/home/robotanist/quori/src/quori/quori_description/models/face_model/materials/textures/";  
    std::cerr << "HERE0" << std::endl;
    cv_bridge::CvImage *ros_image = new cv_bridge::CvImage;
    ros_image->image = cv::imread(path + "dog.jpg");
#endif
}
