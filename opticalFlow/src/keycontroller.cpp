#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <math.h>

using namespace ros;
using namespace std;
using namespace cv_bridge;
using namespace cv;

Publisher pub;
geometry_msgs::PoseStamped msgP;
float x=0.0f,y=0.0f,z=3.0f;

int main(int argc, char **argv)
{
	init(argc, argv, "keycontroller");

	NodeHandle nh;
	pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",50);

	char cmd[50];

	while(ros::ok())
	{
		msgP.header.stamp = ros::Time::now();
    	msgP.header.frame_id = "map";

    	cmd[0]=waitKey(100);

      	if(cmd[0]=='a')
      		x-=.5f;
      	if(cmd[0]=='s')
      		y-=.5f;
      	if(cmd[0]=='w')
      		y+=.5f;
      	if(cmd[0]=='d')
      		x+=.5f;
      	if(cmd[0]=='t')
      		z+=.5f;
      	if(cmd[0]=='g')
      		z-=.5f;
      	if(cmd[0]=='b') break;

      	msgP.pose.position.x=x;
      	msgP.pose.position.y=y;
      	msgP.pose.position.z=z;

      	pub.publish(msgP);

      	cout<<" x= "<<x<<" y= "<<y<<" z= "<<z<<endl;
    	
	}
}