#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <sstream>

sensor_msgs::CameraInfo data;

ros::Publisher cam_info;
ros::Publisher img_info;

//sensor_msgs::Image dataimg;
void pub_cam_info(sensor_msgs::CameraInfo msg)
{
  data=msg;
}

void pub_img(sensor_msgs::Image msg)
{
  //dataimg = msg;
  msg.header.stamp = ros::Time::now();
  data.header.stamp = ros::Time::now();
  img_info.publish(msg);
  cam_info.publish(data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rectify");
  ros::NodeHandle n;

  cam_info = n.advertise<sensor_msgs::CameraInfo>("/my_cam/camera_info", 1000);
  img_info = n.advertise<sensor_msgs::Image>("/my_cam/image_raw", 1000);
  ros::Subscriber in_cam_info = n.subscribe("/usb_cam/camera_info", 10, pub_cam_info);
  ros::Subscriber in_img = n.subscribe("/my_camera/image_raw", 10, pub_img);
 // ros::Rate loop_rate(150);
  ros::spin();
  int count = 0;
 /* while (ros::ok())
  {
    
   
    ros::spinOnce();
    loop_rate.sleep();
  }*/


  return 0;
}