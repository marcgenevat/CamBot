#include <ArduinoHardware.h>
//MOVEMENTS OF SERVOS:
// FROM 100 TO 180 LEFT
// FROM 0 TO 80 RIGHT
// 90 = STOP

#define height 480
#define width 640
#define STOP 90
#define MAX_dreta 0
#define MAX_esquerra 180

#include "Arduino.h"
#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>


ros::NodeHandle  nh;

geometry_msgs::Point pixel_msg;
geometry_msgs::Vector3 rotation_msg;

ros::Publisher pub_pixel("pixel_msg", &pixel_msg);
ros::Publisher pub_rotation("rotation_msg", &rotation_msg);


int velX;
int velY;
Servo myservo_Y;                               
Servo myservo_Xizd;
Servo myservo_Xder;
float center;
float error_X;
float error_Y;
float pitch;
float yaw;
int pos;
int pos_max = 180;
int pos_min = 140;
int init_pos = 160;



void face_center(const geometry_msgs::Point& pixel){
  error_X = ((pixel.x - width/2) / width) * 100;      //Calculate the relative error to the center at the X axis
  error_Y = ((pixel.y - height/2) / height) * 100;    //Calculate the relative error to the center at the Y axis
  
  //Preparing msg
  pixel_msg.x = error_X;
  pixel_msg.y = error_Y;
  pub_pixel.publish(&pixel_msg);  
}

void hand_rotation(const geometry_msgs::Vector3& hand) {
  rotation_msg.x = hand.x;
  rotation_msg.y = hand.y;
  pub_rotation.publish(&rotation_msg);
}
    
void motion_X() {

  myservo_Xizd.write(STOP);                   
  myservo_Xder.write(STOP);
  
  //Setting center zone as +/-5% relative error at the X axis
  if (error_X > 5.0) {
    while (error_X > 5.0) {
      //myservo_Xizd.write(MAX_esquerra);                   
      //myservo_Xder.write(MAX_esquerra);
      myservo_Xizd.write(85);                   
      myservo_Xder.write(85);
      nh.spinOnce();
    }
  } else {
      if  (error_X < -5.0) {
        while (error_X < -5.0) {
          //myservo_Xizd.write(MAX_dreta);                   
          //myservo_Xder.write(MAX_dreta);
          myservo_Xizd.write(100);                   
          myservo_Xder.write(100);
          nh.spinOnce();
        }
      }
  }  
}

void motion_Y() {
    
  //Setting center zone as +/-10% relative error at the Y axis
  if (error_Y < -10.0) {
    while ((error_Y < -10.0) && (pos > pos_min)){
      pos--;
      myservo_Y.write(pos);  //Facing up the camera
      delay(100);      
      nh.spinOnce();
    }
  } else {
      if  (error_Y > -10.0) {
        while ((error_Y > -10.0) && (pos < pos_max)) {
          pos++;
          myservo_Y.write(pos);  //Facing down the camera
          delay(100); 
          nh.spinOnce();
        }
      }
  }
}

ros::Subscriber<geometry_msgs::Point> sub_face("/cambot_img_processor/face_center_xy", &face_center);
ros::Subscriber<geometry_msgs::Vector3> sub_hand("/leapmotion/data", &hand_rotation);

void setup() 
{ 
  myservo_Y.attach(11);      //CAM Servo Pin 11 (Y movements)
  myservo_Xizd.attach(12);  //BOT Servo Left wheel Pin 12 (X movements)
  myservo_Xder.attach(13);  //BOT Servo right wheel Pin 13 (X movements)
  
  myservo_Y.write(init_pos);  //Set CAM servo at the middle of its range position
  
  nh.initNode();            //Initialize ROS node handle, 
  
  nh.subscribe(sub_face);
  nh.subscribe(sub_hand);
  nh.advertise(pub_pixel);
  nh.advertise(pub_rotation);

} 

void loop () {
  nh.spinOnce();

  motion_X();
  motion_Y();  
 
}
