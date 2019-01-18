#include "detectlane.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

int DetectLane::slideThickness = 10;
int DetectLane::BIRDVIEW_WIDTH = 240;
int DetectLane::BIRDVIEW_HEIGHT = 320;
int DetectLane::VERTICAL = 0;
int DetectLane::HORIZONTAL = 1;
Point DetectLane::null = Point();

DetectLane::DetectLane() {
   
    cvCreateTrackbar("getPerspectiveVar", "LaneThreshold", &birdViewvar, 200); //add 
    cvCreateTrackbar("skyLine", "LaneThreshold", &skyLine, 320); //add



    cvCreateTrackbar("LowH", "LaneThreshold", &minThreshold[0], 179); //(Varname, Windowname, currentval, maxval) 
    cvCreateTrackbar("HighH", "LaneThreshold", &maxThreshold[0], 179);

    cvCreateTrackbar("LowS", "LaneThreshold", &minThreshold[1], 255);
    cvCreateTrackbar("HighS", "LaneThreshold", &maxThreshold[1], 255);

    cvCreateTrackbar("LowV", "LaneThreshold", &minThreshold[2], 255);
    cvCreateTrackbar("HighV", "LaneThreshold", &maxThreshold[2], 255);


    cvCreateTrackbar("Low2H", "LaneThreshold", &minLaneInShadow[0], 179); //(Varname, Windowname, currentval, maxval) 
    cvCreateTrackbar("High2H", "LaneThreshold", &maxLaneInShadow[0], 179);

    cvCreateTrackbar("Low2S", "LaneThreshold", &minLaneInShadow[1], 255);
    cvCreateTrackbar("High2S", "LaneThreshold", &maxLaneInShadow[1], 255);

    cvCreateTrackbar("Low2V", "LaneThreshold", &minLaneInShadow[2], 255);
    cvCreateTrackbar("High2V", "LaneThreshold", &maxLaneInShadow[2], 255);

    //cvCreateTrackbar("Shadow Param", "Threshold", &shadowParam, 255);

}

DetectLane::~DetectLane(){}

vector<Point> DetectLane::getLeftLane()
{
    return leftLane;
}

vector<Point> DetectLane::getRightLane()
{
    return rightLane;
}

void DetectLane::update(const Mat &src)
{
    //std::cout<<"src: "<<src.channels()<<std::endl; //(size:320x240)
    Mat img = preProcess(src);  //img = dst = birdViewTranform(imgThresholded);
    //std::cout<<"Mat: "<<img.size()<<std::endl; //(size:240x320)

    vector<Mat> layers1 = splitLayer(img); //img = dst
    vector<vector<Point> > points1 = centerRoadSide(layers1);  //get center if each contours region
    // vector<Mat> layers2 = splitLayer(img, HORIZONTAL);
    // vector<vector<Point> > points2 = centerRoadSide(layers2, HORIZONTAL);

    detectLeftRight(points1); // to detect left,right lane center points

    Mat birdView, lane;
    //birdView = Mat::zeros(img.size(), CV_8UC3);
    lane = Mat::zeros(img.size(), CV_8UC3);

    //for (int i = 0; i < points1.size(); i++)
    // {
    //    for (int j = 0; j < points1[i].size(); j++)
    //    {
    //        circle(birdView, points1[i][j], 1, Scalar(0,0,255), 2, 8, 0 );
    //    }
    //}

    // for (int i = 0; i < points2.size(); i++)
    //  {
    //     for (int j = 0; j < points2[i].size(); j++)
    //     {
    //         circle(birdView, points2[i][j], 1, Scalar(0,255,0), 2, 8, 0 );
    //     }
    // }

    // imshow("Debug", birdView);
    // Draw a Lane Detect view

    for (int i = 1; i < leftLane.size(); i++)
    {
        if (leftLane[i] != null)
        {
            circle(lane, leftLane[i], 1, Scalar(0,0,255), 2, 8, 0 );
        }
    }

    for (int i = 1; i < rightLane.size(); i++)
    {
        if (rightLane[i] != null) {
            circle(lane, rightLane[i], 1, Scalar(255,0,0), 2, 8, 0 );
        }
    }
    //std::cout<<"Lane: "<<lane.size()<<std::endl; //(size:240x320)
    imshow("Lane Detect", lane);
}

Mat DetectLane::preProcess(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);
    //imshow("imgHSV",imgHSV);
    Mat temp = birdViewTranform(imgHSV);
    imshow("birdView_RGB",temp);
    //int minThreshold[3] = {0, 0, 180};
    //int maxThreshold[3] = {179, 30, 255};
    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]),
        imgThresholded);   //Img 0 or 255

    //imgThresholded = laneInShadow(src);

    imshow("Lane Norm", imgThresholded); //----------------------------------------------------


    //Combine 2 IMG to have full Lane
    Mat laneShadow = laneInShadow(src);
    imshow("Lane Shadow", laneShadow);  //----------------------------------------------------
    for(int i = 0; i < imgThresholded.rows; i++)
        for(int j = 0; j < imgThresholded.cols; j++)
            {
                //std::cout<<"i = "<<i<<"  j = "<<j<<std::endl;
                //std::cout<<int(imgThresholded.at<uchar>(i,j))<<std::endl;
                bool tmp = bool(int(imgThresholded.at<uchar>(i,j)));
                if(tmp == false)
                    imgThresholded.at<uchar>(i,j) = laneShadow.at<uchar>(i,j);
            }



    GaussianBlur(imgThresholded,imgThresholded, Size(3, 3), 0,0); //Blurring to reduce high frequency noise

    dst = birdViewTranform(imgThresholded);

    imshow("Bird View", dst);

    fillLane(dst);

    imshow("Binary", imgThresholded);

    return dst;
}

Mat DetectLane::laneInShadow(const Mat &src)
{
    Mat laneShadow, imgHSV, dst;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);
    //imshow("imgHSV",imgHSV);

    inRange(imgHSV, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
        Scalar(maxLaneInShadow[0], maxLaneInShadow[1], maxLaneInShadow[2]),
        laneShadow);

    //GaussianBlur(laneShadow,laneShadow, Size(5, 5), 0,0); //Blurring to reduce high frequency noise

    //imshow("laneShadow",laneShadow);//
    //laneShadow = birdViewTranform(laneShadow);

    return laneShadow;
}


void DetectLane::fillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 1);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    }
}

vector<Mat> DetectLane::splitLayer(const Mat &src, int dir) //vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
{
    //int DetectLane::VERTICAL = 0;
    //int DetectLane::HORIZONTAL = 1;
    //(src size:240x320)
    int rowN = src.rows; //= hight = 320
    int colN = src.cols; //=width = 240
    std::vector<Mat> res;

    //int DetectLane::slideThickness = 10;
    if (dir == VERTICAL) //segment image up to down
    {
        for (int i = 0; i < rowN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(0, i, colN, slideThickness);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }

    else 
    {
        for (int i = 0; i < colN - slideThickness; i += slideThickness) {
            Mat tmp;
            Rect crop(i, 0, slideThickness, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    
    return res;
}

vector<vector<Point> > DetectLane::centerRoadSide(const vector<Mat> &src, int dir)
{
    vector<std::vector<Point> > res;
    int inputN = src.size(); //number of sub Mattrixs
    for (int i = 0; i < inputN; i++) {
        std::vector<std::vector<Point> > cnts; //each cnts[i] is one object boundary
        std::vector<Point> tmp; //tmp = [0,0]

        

        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));  //return the set of outlines
        
        int cntsN = cnts.size(); //a number of object // cnts : vector<vector<Point>>
        
        //if 0 contour
        if (cntsN == 0) {
            res.push_back(tmp);//push_back(vector<Point>)
            continue;
        }
        

        for (int j = 0; j < cntsN; j++) {
            int area = contourArea(cnts[j], false); //area of one object boundary
            if (area > 3) { // check val = 3
                        
                // detemine center of each region
                Moments M1 = moments(cnts[j], false); //moment of each contours. false parameter sets for vector<Point> input parameter
                Point2f center1 = Point2f(static_cast<float> (M1.m10 / M1.m00), static_cast<float> (M1.m01 / M1.m00));
                // have center1 of one lane

                if (dir == VERTICAL) { //??????????
                    center1.y = center1.y + slideThickness*i; // to get exactly centers of a full input Mat
                } 
                
                ///////////////
                else {
                    center1.x = center1.x + slideThickness*i;
                }
                if (center1.x > 0 && center1.y > 0) { 
                    tmp.push_back(center1);
                }
            }
        }
        //tmp is 
        res.push_back(tmp);
    }

    //std::cout<<"res_size: "<<res[4].size()<<std::endl;

    return res; //rec[i] is a vector<Point> which is centers of all contours
}

void DetectLane::detectLeftRight(const vector<vector<Point> > &points) //vector<vector<Point> > points1 = centerRoadSide(layers1);
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();
    
    leftLane.clear();
    rightLane.clear();
    //int DetectLane::BIRDVIEW_WIDTH = 240;
    //int DetectLane::BIRDVIEW_HEIGHT = 320;
    for (int i = 0; i < BIRDVIEW_HEIGHT / slideThickness; i ++)
    {
        leftLane.push_back(null);
        rightLane.push_back(null);
    }

    int pointMap[points.size()][20];  //points.size() = the number of subMat
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)//points.size() = the number of subMat
    {
        for (int j = 0; j < points[i].size(); j++) // points[i] : vector of all the centroids
        {
            int err = 320;
            for (int m = 1; m < min(points.size() - 1 - i, 5); m++)
            {
                bool check = false;
                for (int k = 0; k < points[i + m].size(); k ++) ///////////////m->1
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < dis && 
                    abs(points[i + m][k].x - points[i][j].x) < err) {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        postPoint[i + m][k] = j;
                        check = true;
                    }
                }   
                break; 
            }
            
            if (pointMap[i][j] > max) 
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    //cout << points[posMax.x][posMax.y] << endl << points[posMax2.x][posMax2.y];

    if (max == -1) return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1) break;

        posMax.y = prePoint[posMax.x][posMax.y];
        posMax.x += 1;        
        
        max--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1) break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;        
        
        max2--;
    }

    //we have exactly vector<Point> lane1, lane2 

    
    vector<Point> subLane1(lane1.begin(), lane1.begin() + 5); // check val = 5
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

    Vec4f line1, line2;  //(a,b,c,d)

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01); //
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    //int DetectLane::BIRDVIEW_WIDTH = 240;
    //int DetectLane::BIRDVIEW_HEIGHT = 320;

    int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2]; //????
    int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];


    if (lane1X < lane2X)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
    }
    else
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / slideThickness)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / slideThickness)] = lane1[i];
        }
    }

    //cout << leftLane[10] - rightLane[10] << endl;
}


Mat DetectLane::morphological(const Mat &img)
{
    Mat dst;

    // erode(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );
    // dilate( dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(1, 1)) );

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(10, 20)) );

    // blur(dst, dst, Size(3, 3));

    return dst;
}

void transform(Point2f* src_vertices, Point2f* dst_vertices, Mat& src, Mat &dst){
    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

Mat DetectLane::birdViewTranform(const Mat &src) //input:imgThresholded (size:320x240)
{
    Point2f src_vertices[4];

    int width = src.size().width; //320
    int height = src.size().height; //240

    //std::cout<<"width :"<<width<<"height :"<<height<<std::endl;

    src_vertices[0] = Point(0, skyLine);  //(0,85)
    src_vertices[1] = Point(width, skyLine); //(320,85)
    src_vertices[2] = Point(width, height); //(320,240)
    src_vertices[3] = Point(0, height);  //(0,240)

    Point2f dst_vertices[4];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(BIRDVIEW_WIDTH, 0); //(240,0)     
    dst_vertices[2] = Point(BIRDVIEW_WIDTH - birdViewvar, BIRDVIEW_HEIGHT);//(135,320) ********????105 of BTCs
    dst_vertices[3] = Point(birdViewvar, BIRDVIEW_HEIGHT); //(105,320)

    Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

    Mat dst(BIRDVIEW_HEIGHT, BIRDVIEW_WIDTH, CV_8UC3);
    //int DetectLane::BIRDVIEW_WIDTH = 240;
    //int DetectLane::BIRDVIEW_HEIGHT = 320;
    warpPerspective(src, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    

    return dst;
}
