#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {

    sub = n.subscribe("/quori/base/vel_status", 1000, &SubscribeAndPublish::Callback,this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
     current_time = ros::Time::now();
     last_time = ros::Time::now();
    // current_time = ros::Time(0);
    // last_time = ros::Time(0);


  }

  void Callback(const geometry_msgs::Vector3& msg)
  {

    k++;
    ros::Rate r(hz);
    vl= msg.y;
    vr= -msg.x;
    vx= (vl+vr)/2 * cos(th);
    vy= (vl+vr)/2 * sin(th);
    vth= (vr-vl)/l;

    current_time = ros::Time::now();
    // current_time = ros::Time(0);
    double dt = (current_time - last_time).toSec();
    if (k == 1){
      dt = 1/hz;
    }
  //  printf("%lf",dt);
    double delta_x = vx *  dt;
    double delta_y =  vy *  dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);


    last_time = current_time;
  //  printf("%lf   \n",vl);

  //  PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
//    pub_.publish(output);
   r.sleep();
  }

private:
  ros::NodeHandle n;
  ros::Publisher odom_pub;
  ros::Subscriber sub;
  tf::TransformBroadcaster odom_broadcaster;
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0;
  double vy = 0;
  double vth = 0;
  double hz=20;

  double vl = 0;
  double vr = 0;
  double l = 0.248;
  double t=0;
  double k=0;
  ros::Time current_time, last_time;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "mapping_odometry");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
