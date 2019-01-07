#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

#include "detectsign.h"

using namespace std;
using namespace cv;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void update(const Mat &src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();

    static int slideThickness;  // have to tuning

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    static Point null; // null is a var with type: Point

private:
    Mat preProcess(const Mat &src);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    void detectLeftRight(const vector<vector<Point> > &points);
    Mat laneInShadow(const Mat &src);


    int minThreshold[3] = {80, 0, 177};
    int maxThreshold[3] = {127, 255, 255};
    
    //int minShadowTh[3] = {90, 43, 36};
    //int maxShadowTh[3] = {120, 81, 171};

    int minLaneInShadow[3] = {100, 55,110};
    int maxLaneInShadow[3] = {255, 255, 255};
    
    int binaryThreshold = 180;

    int skyLine = 105; // Region of interesting
    int shadowParam = 40; // check

    int birdViewvar = 93; // to track Bird View transform

    vector<Point> leftLane, rightLane;
    vector<Point> leftLaneShadow, rightLaneShadow;

};

#endif