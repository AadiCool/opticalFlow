#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <math.h>

float ERROR_A=5;
float ERROR_R=0.1;
const double toDegrees = 180.0/3.141593;

using namespace ros;
using namespace std;
using namespace cv_bridge;
using namespace cv;

image_transport::Subscriber sub;
Publisher pub;
Subscriber sub2,sub3;
float heightN,heightP=0.0f;

nav_msgs::Odometry msgP;

Mat image_prev,image_next;
vector<Point2f> points[2];
float focal;
double secs;
Time tmPrevious;

TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
Size subPixWinSize(10,10), winSize(31,31);

float displacmentX=0.0f,displacmentY=0.0f;

KalmanFilter KFX(2,2,1);
KalmanFilter KFY(2,2,1);

Mat stateX(2,1,CV_32F); 
Mat measurementX=Mat::zeros(2,1,CV_32F);
Mat controlX(1,1,CV_32F);
Mat stateY(2,1,CV_32F); 
Mat measurementY=Mat::zeros(2,1,CV_32F);
Mat controlY(1,1,CV_32F);

bool mycompare (float i,float j) { return (i<j); }

bool mycompare2 (Point2f cord1 ,Point2f cord2) {  return( cord1.y < cord2.y ); }//sorts according to A

bool mycompare3 (Point2f cord1 , Point2f cord2) {  return( cord1.x < cord2.x ); }//sorts according to R

void cpolar(float x, float y, float *r, float *theta)
{
   *r = sqrt((pow(x,2))+(pow(y,2)));
   *theta = atan2(y,x);
   return;
}

void initializeKalmannParameters()
{
     stateX.at<float>(0,0)=stateY.at<float>(0,0)=0.0f;
     stateX.at<float>(1,0)=stateY.at<float>(1,0)=0.0f;

    setIdentity(KFX.measurementMatrix);
    setIdentity(KFX.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KFX.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KFX.errorCovPost, Scalar::all(1));

    setIdentity(KFY.measurementMatrix);
    setIdentity(KFY.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KFY.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KFY.errorCovPost, Scalar::all(1));
}

void heightCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    msgP.header.stamp = ros::Time::now();
    msgP.header.frame_id = "map"; 
    heightN=msg->pose.position.z;
    if(heightP==0.0f)
        heightP=heightN;
}

void accCallback(const sensor_msgs::Imu::ConstPtr& msgS)
{
    controlX.at<float>(0,0)=msgS->linear_acceleration.x;
    controlY.at<float>(0,0)=msgS->linear_acceleraction.y;
    secs=(Time::now()-tmPrevious).toSec();
    tmPrevious=Time::now();

    KFX.transitionMatrix = (Mat_<float>(2, 2) << 1, secs, 0, 1);
    KFY.transitionMatrix = (Mat_<float>(2, 2) << 1, secs, 0, 1);
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

void findFinalA(vector<Point2f> *arr,float mean)
{
    for(int i=0;i<(*arr).size();i++)
        if(abs((*arr)[i].y-mean)>=ERROR_A)
            (*arr).erase((*arr).begin()+i);
}

float findMeanA(vector<Point2f> *arr)
{
    float result=0.0f;
    for(int i=0;i<(*arr).size();i++)
        result+=(*arr)[i].y;

    return (result/(*arr).size());
}

float findFinalR(vector<Point2f> *arr,float mean)
{
    float result=0.0f;
    for(int i=0;i<(*arr).size();i++)
        if(abs((*arr)[i].x-mean)>=ERROR_R)
            (*arr).erase((*arr).begin()+i);

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


    Point centre = Point(image_next.cols/2.0f,image_next.rows/2.0f);


    for(int i=0;i<points[0].size();i++)
    {   
        if(status[i])
        {
            diffX=points[0][i].x-points[1][i].x;
            diffY=points[0][i].y-points[1][i].y;

            if(heightP!=heightN)
            {
                float radius = sqrt(pow(points[0][i].x-centre.x,2)+pow(points[0][i].y-centre.y,2));
                float radXh = radius*cos(atan2(points[0][i].y-centre.y,points[0][i].x-centre.x));
                float radYh = radius*sin(atan2(points[0][i].y-centre.y,points[0][i].x-centre.x));

                diffX+=radXh*(heightP-heightN)/heightP;
                diffY+=radYh*(heightP-heightN)/heightP;
            }

            float diffR,diffA;
            cpolar(diffX,diffY,&diffR,&diffA);

            cord.push_back(Point2f(diffR,diffA));
            dispA.push_back(diffA);

            arrowedLine(imgCopy,points[0][i],points[1][i],Scalar(0,0,255),2,8,0,.5);
          }
    } 

    sort (cord.begin(), cord.end(), mycompare2);
    sort (dispA.begin(),dispA.end(),mycompare);
    findFinalA(&cord,dispA[dispA.size()/2]);

    sort (cord.begin(), cord.end(), mycompare3);

    for(int i=0;i<cord.size();i++)  dispR.push_back(cord[i].x);

    sort (dispR.begin(),dispR.end(),mycompare);    

    float finalR=findFinalR(&cord,dispR[dispR.size()/2]);
    float finalA=findMeanA(&cord);
        
    displacmentX+=finalR*cos(finalA);
    displacmentY+=finalR*sin(finalA);        

    std::swap(points[0],points[1]);
    image_prev=image_next.clone();
        //msgP=CvImage(std_msgs::Header(), "bgr8", image_next).toImageMsg();

    msgP.pose.pose.position.x = (displacmentX*heightN/focal);
    msgP.pose.pose.position.y = (displacmentY*heightN/focal);
    msgP.pose.pose.position.z = heightN;

    heightP=heightN;

    cout<<"Found "<<cord.size()<<" dispX "<<msgP.pose.pose.position.x<<" dispY "<<msgP.pose.pose.position.y<<endl;

    imshow("Vedio",imgCopy);
    waitKey(100);
        
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
    sub3= nh.subscribe("/mavros/imu/mag",10, accCallback);
  	pub = nh.advertise<nav_msgs::Odometry>("/myodometry", 50);
    tmPrevious=ros::Time::now();
  	while(ros::ok())
    {
        ros::spinOnce();
        compute();
    }
}