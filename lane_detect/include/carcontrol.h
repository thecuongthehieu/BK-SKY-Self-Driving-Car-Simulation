#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

#include "detectlane.h"

using namespace std;
using namespace cv;

class CarControl 
{
public:
    CarControl();
    ~CarControl();
    void driverCar(const vector<Point> &left, const vector<Point> &right, float velocity, int st, float area);
	float getVelocity(const vector<Point>& leftLane, const vector<Point>& rightLane, bool flag);

private:
    float errorAngle(const Point &dst);
	bool checkTurningPoint();
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float laneWidth = 70; // need to have track laneWidth func

    float minVelocity = 10.0;
    float maxVelocity = 50.0; //60

    int turnTimerThres = 34;
    int frameCounterThres = 30;
    int velocityDecay = 22;
    int curveError = 17;

    float preError;
	bool turnLeft = false;
	bool turnRight = false;
	int frameCounter = 0;
	int turnTimer = 0;

    float kP;  //haven't used
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;
};

#endif
