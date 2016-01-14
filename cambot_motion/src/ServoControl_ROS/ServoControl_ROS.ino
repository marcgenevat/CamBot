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


ros::NodeHandle  nh;

geometry_msgs::Point pixel_msg;

ros::Publisher pub_pixel("pixel_msg", &pixel_msg);


int velX;
int velY;
Servo myservo_Y;                               
Servo myservo_Xizd;
Servo myservo_Xder;
float center;
float error_X;
float error_Y;



void face_center(const geometry_msgs::Point& pixel){
  delay(10);
  error_X = ((pixel.x - width/2) / width) * 100;
  pixel_msg.x = error_X;
  pub_pixel.publish(&pixel_msg);  
}
    
void motion_X() {

  myservo_Xizd.write(STOP);                   
  myservo_Xder.write(STOP);
  
  if (error_X > 2.0) {
    while (error_X > 2.0) {
      myservo_Xizd.write(MAX_esquerra);                   
      myservo_Xder.write(MAX_esquerra);
      nh.spinOnce();
    }
  } else {
      if  (error_X < -2.0) {
        while (error_X < -2.0) {
          myservo_Xizd.write(MAX_dreta);                   
          myservo_Xder.write(MAX_dreta);
          nh.spinOnce();
        }
      }
  }
  
  
  /*while (error_X > 1.0) {
    if (error_X >= 20.0) {
      myservo_Xizd.write(MAX_esquerra);                   
      myservo_Xder.write(MAX_esquerra);
      while (error_X >= 20.0) {check_error_X();}
    }
    else {
      myservo_Xizd.write(MAX_esquerra);                   
      myservo_Xder.write(MAX_esquerra);
      while ((error_X < 20.0) && (error_X >= 1.0)) {
        check_error_X();
        myservo_Xizd.write(MAX_esquerra);                   
        myservo_Xder.write(MAX_esquerra);
        }
    }
    
    while (error_X < -1.0) {
    if (error_X <= -20.0) {
      myservo_Xizd.write(MAX_dreta);                   
      myservo_Xder.write(MAX_dreta);
      while (error_X <= 20.0) {check_error_X();}
    }
    else {
      myservo_Xizd.write(MAX_dreta);                   
      myservo_Xder.write(MAX_dreta);
      while ((error_X > 20.0) && (error_X <= 1.0)) {
        check_error_X();
        myservo_Xizd.write(MAX_dreta);                   
        myservo_Xder.write(MAX_dreta);
        }
    }*/
    

}
/*
void motion_Y() {
  
  check_error_Y();   
  while (error_Y > 1.0) {
    if (error_Y >= 20.0) {
      myservo_Y.write(MAX_esquerra);                   
      while (error_Y >= 20.0) {check_error_Y();}
    }
    else {
      myservo_Y.write(MAX_esquerra);                   
      while ((error_Y < 20.0) && (error_Y >= 1.0)) {
        check_error_Y();
        myservo_Y.write(MAX_esquerra);                   
        }
    }
    
    while (error_Y < 1.0) {
    if (error_Y <= 20.0) {
      myservo_Y.write(MAX_dreta);                   
      while (error_Y <= 20.0) {check_error_Y();}
    }
    else {
      myservo_Y.write(MAX_dreta);                   
      while ((error_Y > 20.0) && (error_Y <= 1.0)) {
        check_error_Y();
        myservo_Y.write(MAX_dreta);                   
        }
    } 
}*/

ros::Subscriber<geometry_msgs::Point> sub("/cambot_img_processor/face_center_xy", &face_center);

void setup() 
{ 
  myservo_Y.attach(9);      //CAM Servo Pin 9 (Y movements)
  myservo_Xizd.attach(12);  //BOT Servo Left wheel (X movements)
  myservo_Xder.attach(13);  //BOT Servo right wheel (X movements)
  
  nh.initNode();            //Initialize ROS node handle, 
  
  nh.subscribe(sub);
  nh.advertise(pub_pixel);

} 

void loop ( ) //(int velX, velY)
{
  nh.spinOnce();

  motion_X();
  //motion_Y();  

  //delay(500); 
}
