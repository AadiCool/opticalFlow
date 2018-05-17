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

using namespace ros;
using namespace std;
using namespace cv_bridge;
using namespace cv;

image_transport::Subscriber sub;
Publisher pub;
Subscriber sub2;

nav_msgs::Odometry msgP;

float height,focal;

Mat image_prev,image_next,res;
float displacmentX=0.0f,displacmentY=0.0f;

bool mycompare (int i,int j) { return (i<j); }

void heightCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    msgP.header.stamp = ros::Time::now();
    msgP.header.frame_id = "map"; 
    height=msg->pose.position.z;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msgS)
{
	try
  	{
        msgP.header.stamp = ros::Time::now();
        msgP.header.frame_id = "map";

    	image_next=(toCvCopy(msgS, "bgr8")->image).clone();
        if(image_next.empty()) return;
     	
  	}
  	catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msgS->encoding.c_str());
  	}
}

void compute()
{
    if(image_next.empty()) return;

    Mat imgCopy=image_next.clone();
    cvtColor(image_next,image_next,CV_BGR2GRAY);
    if(image_prev.empty())
        image_prev=image_next.clone();

    calcOpticalFlowFarneback(image_prev, image_next, res, .4, 1, 12, 2, 8, 1.2, 0);

    vector<float> dispX,dispY;

    for(int i=0;i<image_prev.rows;i+=5)
        for(int j=0;j<image_prev.cols;j+=5)
           {
                const Point2f& fxy = res.at<Point2f>(i,j);
                float flow=atan2(fxy.y,fxy.x);
                dispX.push_back(fxy.x);
                dispY.push_back(fxy.y);

                Scalar color;
                if(flow>1.57f)
                    color=Scalar(0,0,255);
                else if(flow>0 && flow<=1.57f)
                    color=Scalar(255,0,0);
                else if(flow<0 && flow>=(-1.57f))
                    color=Scalar(0,255,0);
                else
                    color=Scalar(150,190,0);             
                arrowedLine(imgCopy,Point(j,i),Point(cvRound(j+fxy.x), cvRound(i+fxy.y)),color,2,8,0,.5);
            }

    sort (dispX.begin(), dispX.end(), mycompare);
    sort (dispY.begin(), dispY.end(), mycompare);
        
    displacmentX+=dispX[dispX.size()/2]*(-1);
    displacmentY+=dispY[dispY.size()/2]*(-1);
           
    image_prev=image_next.clone();
    imshow("Video",imgCopy);
    waitKey(100);

        //msgP=CvImage(std_msgs::Header(), "bgr8", image_Copy).toImageMsg();
    msgP.pose.pose.position.x = (displacmentX*height/focal);
    msgP.pose.pose.position.y = (displacmentY*height/focal);
    msgP.pose.pose.position.z = 2.0;
    pub.publish(msgP);
}

int main(int argc, char **argv)
{
    init(argc, argv, "opticalFlow");

    cout<<"Enter focal length in pixels ";
    cin>>focal;

    NodeHandle nh;
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/iarc/camera/right/image_raw", 10, imageCallback);
    sub2= nh.subscribe("/mavros/local_position/pose",10, heightCallback);
    pub = nh.advertise<nav_msgs::Odometry>("/myodometry", 50);
    while(ros::ok())
    {
        ros::spinOnce();
        compute();
    }
}