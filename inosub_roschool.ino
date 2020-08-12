#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

Servo servo;
Servo ESC;

int vel = 1515; //min
int ang = 1550; //center

void ESC_cb(const std_msgs::Float32& cmd_msg){
  float temp1 = cmd_msg.data;
  int vel = int(temp1) % 10000;
  int ang = int((cmd_msg.data - vel) * 10000) % 10000; 
  
  ESC.writeMicroseconds(vel);
  servo.writeMicroseconds(ang);
}

ros::Subscriber<std_msgs::Float32> sub_vel("/velocity", ESC_cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub_vel);

  ESC.attach(11);
  servo.attach(10);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
