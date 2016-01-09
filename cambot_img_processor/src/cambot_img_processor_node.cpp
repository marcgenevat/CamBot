#include "cambot_img_processor_node.h"

#include "cv.h"


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"




RosImgProcessorNode::RosImgProcessorNode() : 
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
    //loop rate [hz], Could be set from a yaml file 
    rate_=10; 
    
    //sets publishers
    image_pub_ = img_tp_.advertise("image_out", 100);
    face_center_xy_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("face_center_xy", 100);
    
    //sets subscribers
    image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
    camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);
}

RosImgProcessorNode::~RosImgProcessorNode()
{
    //
}

void RosImgProcessorNode::process()
{
    //cv::Rect_<int> box;
    //Initialize face detector object
    cv::CascadeClassifier face_detect;

    std::string pkg_path;
    pkg_path = ros::package::getPath("cambot_img_processor");
    //std::cout << std::endl << "*** Path: " << pkg_path << std::endl;

    cv::String face_cascade_name = pkg_path + "/data/haarcascades/haarcascade_frontalface_default.xml";

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

            for (int i = 0; i < faces.size(); i++)
            {

                // Process face by face:
                cv::Rect face_i = faces[i];

                float face_center_x = 0; //face center x
                float face_center_y = 0;  //face center y
                face_center_x = face_i.x + (face_i.width * 0.5); //face center x
                face_center_y = face_i.y + (face_i.height * 0.5);  //face center y

                std_msgs::Float32MultiArray face_center;

                face_center.layout.dim.resize(1);
                face_center.layout.dim[0].label = "face_center";
                face_center.layout.dim[0].size = 2;
                face_center.data.resize(2);

                //Clear array
                face_center.data.clear();

                face_center.data[0]=(float)face_center_x;
                face_center.data[1]=(float)face_center_y;
                //std::cout <<  "face_center_x: " << face_center_x << std::endl ;


                // Write all we've found out to the original image!
                // First of all draw a green rectangle around the detected face:
                cv::rectangle(cv_img_out_.image, face_i, cv::Scalar(0, 255,0), 1);

                //Publish array
                face_center_xy_pub_.publish(face_center);
                //Let the world know
                //ROS_INFO("Face Detected. Published center x,y");
                //Do this.
                //nh_.spinOnce();

                //Added a delay so not to spam
                //sleep(2);

            }


          /*
            //sets the bounding box of the detection jj
            box.x = (cv_img_ptr_in_->image.cols/2)-10;
            box.y = (cv_img_ptr_in_->image.rows/2)-10;
            box.width = 20;
            box.height = 20;

            //mark a rectangle in the center: http://docs.opencv.org/2.4.11/modules/core/doc/drawing_functions.html#rectangle
            cv::rectangle(cv_img_out_.image, box, cv::Scalar(0,255,255), 3);

            //Mark an ellipse: http://docs.opencv.org/2.4.11/modules/core/doc/drawing_functions.html#ellipse
            cv::ellipse(cv_img_out_.image,cv::Point(50,50),cv::Size(20,10),27,0,360,cv::Scalar(0,0,255),2);
         */
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

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
    //
}

