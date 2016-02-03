#ifndef CAMBOT_IMG_PROCESSOR_NODE_H
#define CAMBOT_IMG_PROCESSOR_NODE_H

/************LIBRARIES************/

#include <ros/ros.h>

#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

/*********************************/

class CambotImgProcessor {
    protected: 
        //ros node handle
        ros::NodeHandle nh;
        
        //image transport
        image_transport::ImageTransport img_tp;
        
        // subscribers to the image and camera info topics
        image_transport::Subscriber image_subs;
        ros::Subscriber camera_info_subs;
		ros::Subscriber detector_subs;
		ros::Subscriber faceSize_subs;
		ros::Subscriber kalmanFilter_subs;

        //publishers
        image_transport::Publisher image_pub;
    
        //wished process rate [hz]
        double rate;
        
    protected: 
        // callbacks
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfo & msg);
		void detectorCallback(const geometry_msgs::Point& msg);
		void faceSizeCallback(const geometry_msgs::Point& msg);
		void kalmanFilterCallback(const geometry_msgs::Point& msg);

    public:
        
        CambotImgProcessor();

        ~CambotImgProcessor();

        void process();
                    
        void publish();                              
        
        double getRate() const;
};
#endif
