# CamBot

This is the first robot project of the UVic-EURECAT's Master on Robotics.
This robot will detect any face or ball throught an usb webcam. This image is published on a Rostopic.
Other pocess will take these images to track the face or ball and pucblish their positions. These position is read for other ros nodes that implements Kalman filter and pid. At the end other process send pwd data to an Arduino board who manage two servos.

Finally this webcamb robot is able to follow the faces or ball.


#Instructions

$ roslaunch cambot_img_processor usb_camera.launch

This will launch ros node that publish images from webcam



$ roslaunch cambot_img_processor cambot_img_processor.launch

This will launch both launcher files , ros node that publish images from webcam and ros node that get these images to detect faces or balls
