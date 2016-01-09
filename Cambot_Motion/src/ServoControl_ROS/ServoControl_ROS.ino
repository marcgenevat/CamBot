//MOVEMENTS OF SERVOS:
// FROM 100 TO 180 LEFT
// FROM 0 TO 80 RIGHT
// 90 = STOP

#include "Arduino.h"
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;   
int velX;
int velY;
Servo myservo_Y;                               // create servo objects
Servo myservo_Xizd;
Servo myservo_Xder;
  
                 
void ControlX_cb(const std_msgs::UInt8& Vel)  //Callback function="ControlX_cb", type=std_msgs::UInt8, message name=Vel. FROM Velocity_X Topic
{     

    velX=Vel.data;
    myservo_Xizd.write(velX);                   //Write received velocity to servos
    myservo_Xder.write(velX);
}

void ControlY_cb(const std_msgs::UInt8& Vel2)  //Callback function="ControlY_cb", type=std_msgs::UInt8, message name=Vel. FROM Velocity_Y Topic
{     

    velY=Vel2.data;                
    myservo_Y.write(velY);                    //Write received velocity to servo             
}


ros::Subscriber<std_msgs::UInt8> sub("Velocity_X", &ControlX_cb);  //Instantiate Suscriber. Suscriber topic=Velocity_X, type=std_msgs
ros::Subscriber<std_msgs::UInt8> sub2("Velocity_Y", &ControlY_cb);  //Instantiate Suscriber. Suscriber topic=Velocity_Y, type=std_msgs

 
void setup() 
{ 
  myservo_Y.attach(9);      //CAM Servo Pin 9 (Y movements)
  myservo_Xizd.attach(12);  //BOT Servo Left wheel (X movements)
  myservo_Xder.attach(13);  //BOT Servo right wheel (X movements)
  nh.initNode();            //Initialize ROS node handle, 
  nh.subscribe(sub);        //advertise topics
  nh.subscribe(sub2);       
 
 
} 
 
void loop ( ) //(int velX, velY)
{
 nh.spinOnce();
 delay(15); 
}
   
  
 




