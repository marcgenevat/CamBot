#define CAMBOT_IMG_PROCESSOR_H

#include "cambot_img_processor_node.h"

CambotImgProcessor::CambotImgProcessor() : 
    nh(ros::this_node::getName()),
    img_tp(nh)
{
    //loop rate [hz], Could be set from a yaml file 
    rate=10; 
    
    //sets publishers
    image_pub = img_tp.advertise("image_out", 100);
    
    //sets subscribers
    image_subs = img_tp.subscribe("image_in", 1, &CambotImgProcessor::imageCallback, this);
    camera_info_subs = nh.subscribe("camera_info_in", 100, &CambotImgProcessor::cameraInfoCallback, this);
	detector_subs = nh.subscribe("detector_in", 1, &CambotImgProcessor::detectorCallback, this);
	faceSize_subs = nh.subscribe("face_size_in", 1, &CambotImgProcessor::faceSizeCallback, this);
	kalmanFilter_subs = nh.subscribe("kalman_in", 1, &CambotImgProcessor::kalmanFilterCallback, this);
}

CambotImgProcessor::~CambotImgProcessor()
{
    //Destructor
}
/*
void RosImgProcessorNode::process()
{
    //cv::Rect_<int> box;
    //Initialize face detector object
    cv::CascadeClassifier face_detect;

    std::string pkg_path;
    pkg_path = ros::package::getPath("cambot_img_processor");
    //std::cout << std::endl << "*** Path: " << pkg_path << std::endl;

    cv::String face_cascade_name = pkg_path + "/data/lbpcascades/lbpcascade_frontalface.xml";

    //face_detect.load("../data/haarcascades/haarcascade_frontalface_default.xml");
    //face_detect.load("../data/lbpcascades/lbpcascade_frontalcatface.xml"); //This one works worse than haarcascade_frontalface_default.xml
    //face_detect.load("../data/lbpcascades/lbpcascade_frontalface.xml"); //This one is FASTER than haarcascade and has less noise
    //face_detect.load("../data/lbpcascades/lbpcascade_profileface.xml"); //This is for profile faces

    if( !face_detect.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return; };
    
    cv::Mat frame_gray; //Used when convert image to gray

    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {
            //copy the input image to the out one
            cv_img_out_.image = cv_img_ptr_in_->image;

            // Convert the current frame to grayscale:
            cv::cvtColor(cv_img_ptr_in_->image, frame_gray, CV_BGR2GRAY);
            cv::equalizeHist(frame_gray,frame_gray); //This algorithm normalizes the brightness and increases the contrast of the image

            //std::cout <<  "encoding: " << cv_img_ptr_in_->encoding << std::endl ;

            //Detect faces as rectangles
            std::vector<cv::Rect> faces;
            face_detect.detectMultiScale(frame_gray, faces, 1.1, 5, 0 , cv::Size(40, 40), cv::Size(480,480)  );


            int max_face_width = 0;
            float face_center_x = 0; //face center x
            float face_center_y = 0;  //face center y
            cv::Rect max_face_i;

            for (int i = 0; i < faces.size(); i++)
            {

                // Process face by face:
                cv::Rect face_i = faces[i];

                if(face_i.width > max_face_width){
                    max_face_width = face_i.width;
                    face_center_x = face_i.x + (face_i.width * 0.5); //face center x
                    face_center_y = face_i.y + (face_i.height * 0.5);  //face center y
                    max_face_i = face_i;
                }

            }

            geometry_msgs::Point face_center;
            face_center.x=(float)face_center_x;
            face_center.y=(float)face_center_y;

            //Publish array
            face_center_xy_pub_.publish(face_center);

            cv::rectangle(cv_img_out_.image, max_face_i, cv::Scalar(0, 255,0), 1);
         
    }
    
    //reset input image
    cv_img_ptr_in_ = nullptr;
}

void RosImgProcessorNode::publish()
{
    //image_raw topic
    cv_img_out_.header.seq ++;
    cv_img_out_.header.stamp = ros::Time::now();
    cv_img_out_.header.frame_id = "camera"; 
    cv_img_out_.encoding = img_encoding_;
    image_pub_.publish(cv_img_out_.toImageMsg());
}

double RosImgProcessorNode::getRate() const
{
    return rate_;
}

*/
/*****CALLBACKS*****/
/*
void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }      
}
*/
void CambotImgProcessor::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
    //
}

