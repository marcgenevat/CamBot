/*
  cambot_kalman_filter_node.h - Kalman filter's tracker library.
  Created by Marc Genevat, Gener 31, 2016.
  Master's UVic Project: CamBot
    Team members: Fèlix Torres, Toni Badenas, Toni Guasch, Xavier Blasco and Marc Genevat
*/

#ifndef CAMBOT_KALMAN_FILTER_V2_NODE_H
#define CAMBOT_KALMAN_FILTER_V2_NODE_H

/************LIBRARIES************/

//Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//ROS
#include <ros/ros.h>

//Std C++
#include <iostream>

//OpenCV
#include <opencv/cv.h>

//ROS headers for image I/O
#include <geometry_msgs/Point.h>

/**********************************/

/*****TYPEDEFS AND STRUCTURES*****/

class CambotKalmanFilter {
	
	protected:
		//Node
		ros::NodeHandle node;

		//Subscribe to face detector topic
		ros::Subscriber face_detected;

		//Publish face tracker
		geometry_msgs::Point tracker_msg;
		ros::Publisher kalman_tracker;

		//Wished process rate [hz]
		double rate;
		double lastCallback;

	protected:
		//Callbacks
		void faceDetectorCallback(const geometry_msgs::Point& detect_msg);

	public:
		//Contructor
		CambotKalmanFilter();

		//Destructor
		~CambotKalmanFilter();
		
		//Prediction x(t) & Cx(t)
		void prediction();

		//Correction part of kalman filter
		void correction();

		//Get distance Malanovich
		double distanceMalanovich();

		//Publish kalman msh
		void publish();

		//Returns rate
		double getRate();

};

/**********************************/

#endif
