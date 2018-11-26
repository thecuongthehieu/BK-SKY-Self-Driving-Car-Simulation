#ifndef DETECTSIGN_H
#define DETECTSIGN_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>
#include <string>

using namespace std;
using namespace cv;

class DetectSign
{
public:
    DetectSign();
    ~DetectSign();
    static int VIEW_WIDTH;
    static int VIEW_HEIGHT;
    static Mat None;

    int updateSign(const Mat &src);
    Mat detectCircle(const Mat &src);
    Mat arrowRegion(const Mat &circleImg);
    int identifySign(const Mat &arrowImg);   // 1,2,3 is right,left,None

private:

    int minThreshold[3] = {78, 158, 124};  //Detect Cirle
    int maxThreshold[3] = {138, 255, 255};

    int circleSize = 64; // scale size
    int arrowSize = 40;  // scale size
    int SIGNTHRESHOLD = 17000;   // Threshold of sum of each region

};

#endif
