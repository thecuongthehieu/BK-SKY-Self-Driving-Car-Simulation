#include "carcontrol.h"

CarControl::CarControl()
{
    
    //cvCreateTrackbar("laneWidth", "Threshold", current_var, max);
    //cvCreateTrackbar("startpoint (=11)", "Threshold", current_var, max);
    //cvCreateTrackbar("fractionOfdist (= 4.0)")", "Threshold", current_var, max);

    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("Team1_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("Team1_speed",10);
}

CarControl::~CarControl() {}

float CarControl::errorAngle(const Point &dst)
{
    //std::cout<<"dst: "<<dst.x<<" "<<dst.y<<std::endl;
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    int i = left.size() - 11; // have to track var = 11
    float error = preError;
    //std::cout<<"left "<<left[31]<<std::endl;
    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0) return;
    }
    if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
    {
        error = errorAngle((left[i] + right[i]) / 2);
    } 
    else if (left[i] != DetectLane::null)
    {
        error = errorAngle(left[i] + Point(laneWidth / 4, 0));
    }
    else
    {
        error = errorAngle(right[i] - Point(laneWidth / 4, 0));
    }

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 