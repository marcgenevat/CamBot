/*
  cambot_kalman_filter_node.cpp - Kalman filter's tracker library.
  Created by Marc Genevat, Gener 31, 2016.
  Master's UVic Project: CamBot
  Team members: Fèlix Torres, Toni Badenas, Toni Guasch, Xavier Blasco and Marc Genevat
*/

#define CAMBOT_KALMAN_FILTER_NODE_C_

/************LIBRARIES************/

#include "cambot_kalman_filter_node.h"

using namespace Eigen;

/**********************************/

/*********LOCAL VARIABLES*********/

//SET KALMAN PARAMETERS

//STATE x+ = F_x·x + F_u·u + F_n·n
//State vectors
Vector4d x_before;
Vector4d x_predicted;
Vector4d x_t;

//Noise state added
Matrix4d C_nx;
double sigma_p_x = 10^2;
double sigma_v_x = 5^2;

//Covariance of the state
Matrix4d C_x_before;
Matrix4d C_x_predicted;
Matrix4d C_x;

//MEASUREMENT y = H·x + v
//Measurement matrixes
MatrixXd H(2,4);
MatrixXd H_inv(2,4);
MatrixXd H_T(4,2);

//Measurement vectors
Vector2d y_predicted;
Vector2d y_t;

//Noise measurement added
Matrix2d C_ny;
double sigma_ny;


//Prediction
//x+ = F_x·x + F_u·u
Matrix4d F;
double dT;
double precTick;
double ticks;

//Correction
MatrixXd K(2,4);	//Kalman gain
Matrix4d I;
Matrix4d TEMP;

//More variables
Vector2d distance;
double dist;

/*********************************/

/*********LOCAL FUNCTIONS*********/

CambotKalmanFilter::CambotKalmanFilter():
	node(ros::this_node::getName())
{
	//Loop rate [hz]
	rate = 10;

	//Set publishers
	kalman_tracker = node.advertise<geometry_msgs::Point>("kalman_out", 1);

	//Set subscribers
	face_detected = node.subscribe("/cambot_face_detector/detector_out", 1, &CambotKalmanFilter::faceDetectorCallback, this);

    	//Initial state position
	x_predicted(0) = 320;
    	x_predicted(1) = 240;
    	x_predicted(2) = 0;
    	x_predicted(3) = 0;

	//Initial covariance state
    	C_x_before << 100, 0, 0, 0,
               	   0, 10^2, 0, 0,
                   0, 0, 5^2, 0,
                   0, 0, 0, 5^2;

	//Covariance prediction noise
    	C_nx << sigma_p_x, 0, 0, 0,
         	0, sigma_p_x, 0, 0,
         	0, 0, sigma_v_x, 0,
         	0, 0, 0, sigma_v_x;

	//Covariance measurement
   	H << 1, 0, 0, 0,
    	 0, 1, 0, 0;

	//Covariance measurement noise
	sigma_ny = 20^2;
	C_ny << sigma_ny, 0,
         0, sigma_ny;

	//Prediction variables
	dT = 0;
	precTick = 0;
	ticks = 0;

	F << 1, 0, dT, 0,
   		 0, 1, 0, dT,
    	 0, 0, 1, 0,
         0, 0, 0, 1;

	//Correction variables
	I << 1, 0, 0, 0,
    	 0, 1, 0, 0,
    	 0, 0, 1, 0,
    	 0, 0, 0 ,1;
}

CambotKalmanFilter::~CambotKalmanFilter() 
{
	//Destructor
}

void CambotKalmanFilter::prediction() {
	x_before = x_t;
	C_x_before = C_x;
	x_predicted = F * x_before;

	precTick = ticks;
	ticks = (double) cv::getTickCount();
	dT = (ticks - precTick) / cv::getTickFrequency();	//[sec]

	C_x_predicted = F * C_x_before * F.transpose() + C_nx;
}

void CambotKalmanFilter::correction() {
	H_T = H.transpose();
	y_predicted = H * x_predicted;

	K = C_x_predicted * H_T * ((H * C_x_predicted * H_T) + C_ny).inverse();
	x_t = x_predicted + K*(y_t - y_predicted);

	TEMP = (I - (K * H));
	C_x = TEMP * C_x_predicted * TEMP.transpose() + K * C_ny * K.transpose();
}

double CambotKalmanFilter::distanceMalanovich() {
	Vector2d error_y = y_t - y_predicted;
	MatrixXd H_block = H.block<2,2>(0,0);
	MatrixXd C_x_block = C_x.block<2,2>(0,0);
	Matrix2d TEMP = H_block * C_x_block * H_block.transpose();

	Matrix2d inverse = (C_ny + TEMP).inverse();
	distance = (error_y.transpose() * inverse) + error_y.transpose();
	dist = (distance(0) + distance(1));

	return dist;
}

void CambotKalmanFilter::publish() {
	tracker_msg.x = (uint)x_t[0];
	tracker_msg.y = (uint)x_t[1];
	
	kalman_tracker.publish(tracker_msg);
}

double CambotKalmanFilter::getRate()
{
    return rate;
}

void CambotKalmanFilter::faceDetectorCallback(const geometry_msgs::Point& detect_msg) {
	try {
		y_t[0] = detect_msg.x;
		y_t[1] = detect_msg.y;
		lastCallback = ros::Time::now().toSec();
	} catch (ros::Exception& e) {
		ROS_ERROR("CambotKalmanFilter::faceDetectorCallback(): exception: %s", e.what());
        return;
    }
}

/*********************************/

