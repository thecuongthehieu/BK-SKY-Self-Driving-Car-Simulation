#include "detectsign.h"

int DetectSign::VIEW_WIDTH = 320;
int DetectSign::VIEW_HEIGHT = 240;
Mat DetectSign::None = Mat::zeros(Size(0,0),CV_64FC3);

DetectSign::DetectSign()
{
    /*cvCreateTrackbar("LowHH", "SignThreshold", &minThreshold[0], 179); //(Varname, Windowname, currentval, maxval)
    cvCreateTrackbar("HighHH", "SignThreshold", &maxThreshold[0], 179);

    cvCreateTrackbar("LowSS", "SignThreshold", &minThreshold[1], 255);
    cvCreateTrackbar("HighSS", "SignThreshold", &maxThreshold[1], 255);

    cvCreateTrackbar("LowVV", "SignThreshold", &minThreshold[2], 255);
    cvCreateTrackbar("HighVV", "SignThreshold", &maxThreshold[2], 255);*/
}

DetectSign::~DetectSign(){}

std::pair<int, float> DetectSign::updateSign(const Mat &src)
{
    std::pair<Mat, float> circleImg = detectCircle(src);
    //cv::imshow("Circle",circleImg);
    Mat arrowImg = arrowRegion(circleImg.first);
    int signType =100;
    signType = identifySign(arrowImg);
    Point carPosition = Point(120, 300);
    // double dist = cv::norm(circleImg.second - carPosition);
    return make_pair(signType, circleImg.second);
}

std::pair<Mat, float> DetectSign::detectCircle(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst, circleImg = None;
    float area = 0.0;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
        Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]),
        imgThresholded);

    //cv::imshow("imgThresholded",imgThresholded);

    std::vector<std::vector<Point> > cnts;

    findContours(imgThresholded, cnts, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    int maxarea = 0;
    std::vector<Point> circleCnt;
    if(cnts.size() >=1)
    {
        for(int i = 0; i < cnts.size(); i++)
        {
            int area = contourArea(cnts[i], false);
            if(area > maxarea)
            {
                circleCnt = cnts[i];
                maxarea = area;
            }
        }
    }

    int t = 239, d = 0, l = 319, r = 0;

    if(circleCnt.size() > 0)
    {
        for(int i = 0; i < circleCnt.size(); i++)
            {
                if(circleCnt[i].x < l)
                    l = circleCnt[i].x;
                if(circleCnt[i].x > r)
                    r = circleCnt[i].x;
                if(circleCnt[i].y < t)
                    t = circleCnt[i].y;
                if(circleCnt[i].y > d)
                    d = circleCnt[i].y;
            }
        if((r-l)*(d-t) > 500) //check S(circleCnt)
            {
                Rect roi(l,t,r-l,d-t);
                //p = (roi.br() + roi.tl()) * 0.5;
                area = (float) (roi.width * roi.height);
                circleImg = Mat(src,roi);
                //std::cout<<area<<std::endl;
            }
    }
    return std::make_pair(circleImg, area);
}

Mat DetectSign::arrowRegion(const Mat &circleImg)
{
    
    Mat recircleImg, resizeImg, mask, arrowImg = None;
    if(circleImg.size() != None.size())
    {
        resize(circleImg,recircleImg,Size(circleSize,circleSize)); //(64,64)
        cvtColor(recircleImg, resizeImg, COLOR_BGR2GRAY);
        threshold(resizeImg,mask,240,255,THRESH_BINARY);

        std::vector<std::vector<Point> > cnts;
        findContours(mask, cnts, RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        //cv::imshow("mask",mask);

        std::vector<Point> arrowCnt;
        int areaNorm = 220; //have check
        if(cnts.size() >=1)
        {
            for(int i = 0; i < cnts.size(); i++)
            {
                int area = contourArea(cnts[i], false);
                if(area > areaNorm)
                {
                    arrowCnt = cnts[i];
                }
            }
        }

        if(arrowCnt.size() > 0)
        {
            Rect boundRect;
            boundRect = boundingRect(arrowCnt);
            arrowImg = Mat(recircleImg, boundRect);
        }
    }


    return arrowImg;
}
int DetectSign::identifySign(const Mat &arrowImg)
{
    std::vector<string> signsTable(2);
    signsTable[0] = "0110"; //RIGHT
    //ignsTable[1] = "1110"; //RIGHT
    signsTable[1] = "1001"; //LEFT
    //signsTable[3] = "1101"; //LEFT

    int signType = 3; //unknown

    if(arrowImg.size() != None.size())
    {
        Mat rearrowImg, resizeImg, mask;
        resize(arrowImg,rearrowImg,Size(arrowSize,arrowSize)); //(64,64)
        cvtColor(rearrowImg,resizeImg, COLOR_BGR2GRAY);
        threshold(resizeImg,mask,240,255,THRESH_BINARY);

        //cv::imshow("RAW_INPUT",mask);


        // ERODE AND DILATE
        int erosion_size = 2;
        Mat element = getStructuringElement(cv::MORPH_CROSS,
              cv::Size( erosion_size + 1,  erosion_size + 1),
              cv::Point(erosion_size, erosion_size) );
        erode(mask,mask,element);

        element = getStructuringElement(cv::MORPH_CROSS,
              cv::Size(2 * erosion_size + 1,2 * erosion_size + 1),
              cv::Point(erosion_size, erosion_size) );
        dilate(mask, mask, element);
        /////////////

        cv::imshow("Sign",mask);

        //SIGNTHRESHOLD = 17000

        int midPoint = arrowSize/2;

        Mat lefttopBlock = Mat(mask, Rect(0,0,midPoint-1,midPoint-1));
        Mat rightopBlock = Mat(mask, Rect(midPoint,0,midPoint-1,midPoint-1));
        Mat leftdownBlock = Mat(mask, Rect(0,midPoint,midPoint-1,midPoint-1));
        Mat rightdownBlock = Mat(mask, Rect(midPoint,midPoint,midPoint-1,midPoint-1));

        int lt = cv::sum(cv::sum(lefttopBlock))[0];
        int rt = cv::sum(cv::sum(rightopBlock))[0];
        int ld = cv::sum(cv::sum(leftdownBlock))[0];
        int rd = cv::sum(cv::sum(rightdownBlock))[0];

        string signLookup;

        lt > TOPTHRESHOLD ? signLookup += string("1") : signLookup += string("0");
        rt> TOPTHRESHOLD ? signLookup += string("1") : signLookup += string("0");
        ld > DOWNTHRESHOLD ? signLookup += string("1") : signLookup += string("0");
        rd > DOWNTHRESHOLD ? signLookup += string("1") : signLookup += string("0");

        //cout<< signLookup <<endl;

        for(int i = 0; i < signsTable.size(); i++)
            {
                if(signLookup == signsTable[i])
                    {
                        if(i == 0)
                            signType = 0; //RIGHT
                        else
                            signType = 1; //LEFT
                    }
            }


    }

    return signType;
}



