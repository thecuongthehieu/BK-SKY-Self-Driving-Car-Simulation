#include "carcontrol.h"
#include "detectsign.h"
#include "unistd.h" // lib for time sleep from detect sign to adjust angle



CarControl::CarControl()
{
    
    //cvCreateTrackbar("laneWidth", "Threshold", current_var, max);
    //cvCreateTrackbar("startpoint (=11)", "Threshold", current_var, max);
    //cvCreateTrackbar("fractionOfdist (= 4.0)")", "Threshold", current_var, max);
    cvCreateTrackbar("timeThres", "Threshold", &turnTimerThres, 100);
    cvCreateTrackbar("frameThres", "Threshold", &frameCounterThres, 100);
    cvCreateTrackbar("velocityDecay", "Threshold", &velocityDecay, 300);
    cvCreateTrackbar("curveError", "Threshold", &curveError, 50);


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


float CarControl::getVelocity(const vector<Point> &leftLane, const vector<Point> &rightLane, bool flag) {
	// detect sign velocity decrease law
	float velocity = maxVelocity;
    if(flag)
        velocity = maxVelocity - (float(velocityDecay)/100)*frameCounter; //50 : 0,4
    
    else
    {
        int nullLeft = 0;
        for (Point p : leftLane)
            if (p != DetectLane::null) 
                nullLeft++;
        int nullRight = 0;
        for (Point p : rightLane)
            if (p != DetectLane::null) 
                nullRight++;

        if(nullLeft >= 15 || nullRight >= 15)
            {
                float w = (nullLeft + nullRight) / (2.0 * leftLane.size());
                //cout<<w<<endl;
                w < 0.6 ? velocity = (w+0.25) * maxVelocity : velocity = maxVelocity;

            }
    }
    return velocity;
}


bool CarControl::checkTurningPoint() {
	return frameCounter >= frameCounterThres;
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity, int st, float
area)//add st as left/right/unknown
{
    int i = left.size() - 11; // have to track var = 11
    float error = preError;
    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0) return;
	}

    int leftPoints = 0;
    for (int i = 0; i < left.size(); i++) {
        if (left[i] != DetectLane::null) leftPoints++;
    }

    int rightPoints = 0;
    for (int i = 0; i < left.size(); i++) {
        if (right[i] != DetectLane::null) rightPoints++;
    }


	if (turnTimer >= turnTimerThres ) {

        //set for the second 
        turnRight = false;
        turnLeft = false;

        frameCounterThres = 30;////////////// for the second////////
        //curveError = 
        //turnTimerThres = 

        frameCounter = 0;
        turnTimer = 0;
	}


	if (turnRight || turnLeft) {
		frameCounter++;  
	}
    //cout<<"ok"<<endl;
    //cout<<frameCounter<<endl;

	bool check = checkTurningPoint();

	if (turnRight && check) {
		cout << "turn time right " << turnTimer << endl;
		error = curveError;
		turnTimer++;
        frameCounterThres = 0;
        frameCounter = 0;
	} else if (turnLeft && check) {
		cout << "turn time left " << turnTimer << endl;
        error = -curveError;
		turnTimer++;
        frameCounterThres = 0;
        frameCounter = 0;
		
	} else {

		if (st == 0 && area >= 500.0) {
			// turn right
			// frameCounter++;
			turnRight = true;
			turnLeft = false;

		} else if (st == 1 && area >= 500.0) {
			// turn left
			// frameCounter++;
			//cout << "turn left" << endl;
			turnLeft = true;
			turnRight = false;
		} 


		if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
		{
			//std::cout<<(right[i]-left[i])<<std::endl;
            //std::cout<<leftPoint<<std::endl;
            Point tmp = right[i] - left[i];
            if(tmp.x <= laneWidth/2)  //////////turning
            {
                if(leftPoints <= 8)
                    error = errorAngle(left[i]);
                else if(rightPoints <= 8)
                    error = errorAngle(right[i]);
            }
            else 
                error = errorAngle((left[i] + right[i]) / 2);

		} 
		else if (left[i] != DetectLane::null)
		{
			if(leftPoints <= 8)///////////////////////////turnning
                i--;
            else 
                error = errorAngle(left[i] + Point(laneWidth / 2.3, 0));  //Need to tunning
		}
		else
		{
            if(rightPoints <=8)
                i--;
			else
                error = errorAngle(right[i] - Point(laneWidth / 2.3, 0));
		}


        
        if(turnLeft)
            error = 0.45; //to curve
        else if(turnRight)
            error = -0.45;

        if(!turnLeft && !turnRight && abs(error) >= 1.5)
        {   
            velocity = getVelocity(left,right,false);
        }


	}
    //cout<<error<<endl;
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 
