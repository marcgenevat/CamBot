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


In order to visualize the robot model with RVIZ:

$ roslaunch cambot_description display.launch


##CAMBOT_MOTION:
#--USE OF PACKAGE:-------------

BUILD CATKIN PACKAGE
$ catkin_make --pkg Cambot_Motion

UPLOAD CODE TU ARDUINO BOARD (ServoControl_ROS.ino)

Terminal 1:
$ roslaunch Cambot_Motion rosserial.launch

Terminal 2:
$ rostopic list

TO SET VELOCITY OF ROBOT (X MOVEMENT):
$ rostopic pub Velocity_X std_msgs/UInt8 XX (Where XX is X Velocity)

TO SET VELOCITY OF WEBCAM (Y MOVEMENT):
$ rostopic pub Velocity_y std_msgs/UInt8 XX (Where XX is Y Velocity)




#--VELOCITY VALUES:
ROBOT X MOVEMENT:
	FROM 100 A 180 LEFT DIRECTION
	FROM 0 A 80 LEFT DIRECTION
ROBOT Y MOVEMENT:
	UP MOVEMENT: To be defined in the integration
	DOWN MOVEMENT: To be defined in the integration

#--NOTES:
Is possible you need to change port in launch File:
Open launch file, and in the following text set your port detected by Arduino:
	<param name="port" value="/dev/ttyUSB2"/>

change "ttyUSB2" for your real Arduino port


