#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"
#include<iostream>


bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
int skipFrame = 1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("View", cv_ptr->image);
        //std::cout<<cv_ptr->image<<std::endl;
        //std::cout<<cv_ptr->image.size()<<std::endl;
        waitKey(1);  // time of showing each frame
        detect->update(cv_ptr->image);  //cv_ptr-> image (320x240) is input Mat of detectlane
        // after update vector (size =32) left and right point so call driverCar to control car
        car->driverCar(detect->getLeftLane(), detect->getRightLane(), 40);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess() //When we have video.avi so set STREAM = false
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty()) break;
        
        imshow("View", src);
        detect->update(src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
{
	//std::cout <<"start"<<std::endl;
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    cv::namedWindow("Sign");

    detect = new DetectLane();
    car = new CarControl();

    if (STREAM) {

    	//std::cout<<"STREAM"<<std::endl;
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);

        ros::spin();
    } else {
    	//std::cout<<"videoProcess"<<std::endl;
        videoProcess();
    }
    cv::destroyAllWindows();
}