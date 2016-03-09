/*
  cambot_kalman_filter_main.cpp - Kalman filter's tracker main program.
  Created by Marc Genevat, Gener 31, 2016.
  Master's UVic Project: CamBot
  Team members: Fèlix Torres, Toni Badenas, Toni Guasch, Xavier Blasco and Marc Genevat
*/

#define CAMBOT_KALMAN_FILTER_V2_MAIN_C_

/************LIBRARIES************/

#include "cambot_kalman_filter_V2_node.h"
#include <iostream>

/**********************************/

//Node main
int main (int argc, char **argv) {
	
	//Init ros
	ros::init(argc, argv, "cambot_kalman_filter");

	//Create ros wrapper object
	CambotKalmanFilter kalman_filter;

	//Set node loop rate
	ros::Rate loopRate(kalman_filter.getRate());

	//Node loop
	while( ros::ok() ) {
		
		//Execute pending callbacks
		ros::spinOnce();

		//Prediction
		kalman_filter.prediction();

		//Correction
		kalman_filter.correction();

		//Publish msg
		kalman_filter.publish();

		//Fitting output rate
		loopRate.sleep();
	}

	//Exit program
	return 0;

}
