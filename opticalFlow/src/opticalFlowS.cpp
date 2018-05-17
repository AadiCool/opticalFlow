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

float ERROR_A=5;
float ERROR_R=0.1;
const double toDegrees = 180.0f/3.141593f;

using namespace ros;
using namespace std;
using namespace cv_bridge;
using namespace cv;

image_transport::Subscriber sub;
Publisher pub;
Subscriber sub2;
float heightN,heightP=0.0f;

nav_msgs::Odometry msgP;

Mat image_prev,image_next;
vector<Point2f> points[2];
float focal;

TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);

float displacmentX=0.0f,displacmentY=0.0f;//the absolute displacements

bool mycompare (float i,float j) { return (i<j); } //normal comparison

bool mycompare2 (Point2f cord1 ,Point2f cord2) {  return( cord1.y < cord2.y ); }//sorts according to A

bool mycompare3 (Point2f cord1 , Point2f cord2) {  return( cord1.x < cord2.x ); }//sorts according to R

void cpolar(float x, float y, float *r, float *theta)//converts cartesian to polar
{
   *r = sqrt((pow(x,2))+(pow(y,2)));
   *theta = atan2(y,x);
   return;
}

void heightCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)//accepts height 
{
    msgP.header.stamp = ros::Time::now();
    msgP.header.frame_id = "map"; 
    heightN=msg->pose.position.z;
    if(heightP==0.0f)
        heightP=heightN;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msgS)//accepts image
{
	try
  	{
    	msgP.header.stamp = ros::Time::now();
    	msgP.header.frame_id = "map"; 
        Rect roi;

    	image_next=(toCvCopy(msgS, "bgr8")->image).clone();

        roi.width=(int)(image_next.cols*.75f);
        roi.height=(int)(image_next.rows*.75f);
        roi.x=(int)(image_next.cols*.125f);
        roi.y=(int)(image_next.rows*.125f);
        //image_next=image_next(roi);

    	if(image_next.empty()) return;

  	}
  	catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msgS->encoding.c_str());
  	}
}

void findFinalA(vector<Point2f> *arr,float mean)//finds the final set after angle fitration
{
    for(int i=0;i<(*arr).size();i++)
        if(abs((*arr)[i].y-mean)>=(ERROR_A/toDegrees))
        {
            (*arr).erase((*arr).begin()+i);
            i--;
        }
}

float findMeanA(vector<Point2f> *arr)//finds the mean angle of the final set of points
{
    float result=0.0f;
    for(int i=0;i<(*arr).size();i++)
        result+=(*arr)[i].y;

    return (result/(*arr).size());
}

float findFinalR(vector<Point2f> *arr,float mean)//finds the final set after angle fitration
{
    float result=0.0f;
    for(int i=0;i<(*arr).size();i++)
        if(abs((*arr)[i].x-mean)>=(ERROR_R*focal/heightP))
        {
            (*arr).erase((*arr).begin()+i);
            i--;
        }

    for(int i=0;i<(*arr).size();i++)
        result+=(*arr)[i].x;

    return (result/(*arr).size());
    
}

void compute()
{
    if(image_next.empty()) return;
    if(image_prev.empty()) 
    {
        image_prev=image_next.clone();
        displacmentX=displacmentY=0.0f;
        if(image_next.channels() != 1)
            cvtColor(image_prev,image_prev,CV_BGR2GRAY);
    }
        
    goodFeaturesToTrack(image_prev, points[0], 100, 0.02, 40);
    if(points[0].size()<=0) return;
    cornerSubPix(image_prev, points[0], subPixWinSize, Size(-1,-1), termcrit);
    points[1].resize(points[0].size());

    Mat imgCopy=image_next.clone();

    if(image_next.channels() != 1)
        cvtColor(image_next,image_next,CV_BGR2GRAY);

    vector<uchar> status;
    status.resize(points[0].size());
    vector<float> err;
    err.resize(points[0].size()); 
    calcOpticalFlowPyrLK(image_prev,image_next, points[0], points[1], status, err, winSize,
                            3, termcrit, 0, 0.001);
        
    
    float diffX,diffY;
    vector<Point2f> cord;
    vector<float> dispR,dispA;

    Point centreI = Point(image_next.cols/2,image_next.rows/2);


    for(int i=0;i<points[0].size();i++)
    {   
        if(status[i])
        {
            diffX=points[0][i].x-points[1][i].x;
            diffY=points[0][i].y-points[1][i].y;

            if(heightP!=heightN)
            {
                float radiusR = sqrt(pow(points[0][i].x-centreI.x,2)+pow(points[0][i].y-centreI.y,2))*(heightP-heightN)/heightN;
                float radXh = radiusR*cos(atan2(points[0][i].y-centreI.y,points[0][i].x-centreI.x));
                float radYh = radiusR*sin(atan2(points[0][i].y-centreI.y,points[0][i].x-centreI.x));

                diffX+=radXh;
                diffY+=radYh;
            }

            float diffR,diffA;
            cpolar(diffX,diffY,&diffR,&diffA);

            cord.push_back(Point2f(diffR,diffA));
            dispA.push_back(diffA);

            arrowedLine(imgCopy,points[0][i],points[1][i],Scalar(0,0,255),2,8,0,.5);
          }
    } 

    sort (cord.begin(), cord.end(), mycompare2);//sorts co-ordinates according to angle
    sort (dispA.begin(),dispA.end(),mycompare);//sorts the angles to find median
    findFinalA(&cord,dispA[dispA.size()/2]);

    sort (cord.begin(), cord.end(), mycompare3);//sorts co-ordinates according to radius

    for(int i=0;i<cord.size();i++)  dispR.push_back(cord[i].x);//initializes the radius collection

    sort (dispR.begin(),dispR.end(),mycompare);//sorts the radius array

    float finalR=findFinalR(&cord,dispR[dispR.size()/2]);
    float finalA=findMeanA(&cord);
        
    displacmentX+=finalR*cos(finalA);
    displacmentY+=finalR*sin(finalA);        

    std::swap(points[0],points[1]);
    image_prev=image_next.clone();
        //msgP=CvImage(std_msgs::Header(), "bgr8", image_next).toImageMsg();

    msgP.pose.pose.position.x = (displacmentX*heightN/focal);//*(-1);
    msgP.pose.pose.position.y = (displacmentY*heightN/focal);
    msgP.pose.pose.position.z = heightN;

    heightP=heightN;

    cout<<"Found "<<cord.size()<<" dispX "<<msgP.pose.pose.position.x<<" dispY "<<msgP.pose.pose.position.y<<endl;

    namedWindow("Vedio",WINDOW_NORMAL);
    imshow("Vedio",imgCopy);
    waitKey(100);
        
    pub.publish(msgP);   
}

int main(int argc, char **argv)
{
	init(argc, argv, "opticalFlow");

    cout<<"Enter focal length in pixels\n ";
    cin>>focal;

	NodeHandle nh;
	image_transport::ImageTransport it(nh);
	sub = it.subscribe("/iarc/camera/right/image_raw", 10, imageCallback);
    sub2= nh.subscribe("/mavros/local_position/pose",10, heightCallback);
  	pub = nh.advertise<nav_msgs::Odometry>("/myodometry", 50);
  	while(ros::ok())
    {
        ros::spinOnce();
        //cout<<"In while\n";
        if(image_next.empty()) 
        {
            //cout<<"In if\n";
            continue;
        }
        compute();
    }
}