#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

Servo servo;
Servo ESC;

int vel = 1515; //min
int ang = 1550; //center

void servo_cb(const std_msgs::Float32& cmd_msg){
  servo.writeMicroseconds(cmd_msg.data);
}

void ESC_cb(const std_msgs::Float32& cmd_msg){
  servo.writeMicroseconds(cmd_msg.data);
}

ros::Subscriber<std_msgs::Float32> sub_vel("/velocity",ESC_cb);
ros::Subscriber<std_msgs::Float32> sub_angle("/streer_angle",servo_cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.subscribe(sub_angle);

  ESC.attach(9); //ESC pin
  servo.attach(10);
  
  //ESC.writeMicroseconds(vel); //1515~1700+a
  //servo.writeMicroseconds(ang);

}

void loop() {
  nh.spinOnce();
  delay(1);
  //servo.writeMicroseconds(ang);
  /*
  int speed; //Implements speed variable
  for(speed = 0; speed <= 70; speed += 5) { //Cycles speed up to 70% power for 1 second
      setSpeed(speed); //Creates variable for speed to be used in in for loop
      delay(1000);
  }
  delay(4000); //Stays on for 4 seconds
  for(speed = 70; speed > 0; speed -= 5) { // Cycles speed down to 0% power for 1 second
      
  }
  setSpeed(0); //Sets speed variable to zero no matter what
  delay(1000); //Turns off for 1 second
  */
}
