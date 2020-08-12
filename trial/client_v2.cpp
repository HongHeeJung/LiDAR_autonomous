#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "math.h"
#include "stdlib.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define THRESHOLD_DIS_DIFF1 0.2
#define THRESHOLD_DIS_DIFF2 0.55
#define THRESHOLD_DIS_DIFF3 1.0
#define GO_CENTER_ANG30 40
#define GO_CENTER_ANG50 75
#define GO_CENTER_ANG70 85

#define CENTER_STEER 1553
#define CENTER_PLUS1 40
#define CENTER_PLUS2 90

#define THRESHOLD_SPACE_RATE 1.4
 

std_msgs::Float32 pub_velocity;
ros::Publisher pub_vel;

float angle_detected[400] = {0.0,};
float distance_detected[400] = {0.0,};


int min_Total_distance(int count){
    float temp = 30;
    int min_angle_index = 0;
    for(int i = 0; i < count/2 ; i++){
        if(temp > distance_detected[i] + distance_detected[i+count/2] ){
            temp = distance_detected[i] + distance_detected[i+count/2];
            min_angle_index = i;
        }
    }
    return min_angle_index;
}

int get_steering(float steer_angle){
    float angle_for_Ardu = steer_angle;
    if(angle_for_Ardu < -21) angle_for_Ardu = CENTER_STEER - CENTER_PLUS2;
    else if(-21 <= angle_for_Ardu && angle_for_Ardu < -7)  angle_for_Ardu = CENTER_STEER - CENTER_PLUS1;
    else if(-7 <= angle_for_Ardu && angle_for_Ardu < 7)  angle_for_Ardu = CENTER_STEER;
    else if(7 <= angle_for_Ardu && angle_for_Ardu < 21)  angle_for_Ardu = CENTER_STEER + CENTER_PLUS1;
    else if(21 <= angle_for_Ardu)  angle_for_Ardu = CENTER_STEER + CENTER_PLUS2;
    int under = int(angle_for_Ardu);

    return under;
}

int get_velocity(float steer_angle, int flag){
    if(flag == 0) int velocity = 1560 - steer_angle;
    else if(flag == 1) int velocity = 1560 - steer_angle;
    else if(flag == 2) int velocity = 1540 - steer_angle;
    else if(flag == 3) int velocity = 1550 - steer_angle;
    int over = velocity;

    return over;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        angle_detected[i] = degree; 
        distance_detected[i] = scan->ranges[i];
    }
    int min_angle_index = min_Total_distance(count);
    
    float theta = angle_detected[min_angle_index + count/2];
    float right_dist = distance_detected[min_angle_index];
    float left_dist = distance_detected[min_angle_index + count/2];

    int flag = 0;
    float steer_angle = 0; //틀어야 하는 각도

    float dis_diff = (left_dist < right_dist)? right_dist - left_dist : left_dist - right_dist ;
    if(min_angle_index == 0 || dis_diff == 0){
        steer_angle = 0;
        int velocity = 1560;
        ROS_INFO("================= CENTER! =================");
    }
    else if (dis_diff < THRESHOLD_DIS_DIFF1){
        steer_angle = fabs(theta - 90);
        flag = 0;
        ROS_INFO("================= CASE 1 =================");
    }
    else if(dis_diff>= THRESHOLD_DIS_DIFF1 && dis_diff < THRESHOLD_DIS_DIFF2){
        if (left_dist < right_dist){
            steer_angle = theta - (180 - GO_CENTER_ANG70);
        }
        else if(left_dist > right_dist){
            steer_angle = theta - GO_CENTER_ANG70;
        }
        flag = 1;
        ROS_INFO("================= CASE 2 =================");
    }
    else if(dis_diff>= THRESHOLD_DIS_DIFF2 && dis_diff < THRESHOLD_DIS_DIFF3){
        if (left_dist < right_dist){
            steer_angle = theta - (180 - GO_CENTER_ANG50);
        }
        else if(left_dist > right_dist){
            steer_angle = theta - GO_CENTER_ANG50;
        }
        flag = 2;
        ROS_INFO("================= CASE 3 =================");
    }
    else{
        if (left_dist < right_dist){
            steer_angle = theta - (180 - GO_CENTER_ANG30);
        }
        else if(left_dist > right_dist){
            steer_angle = theta - GO_CENTER_ANG30;
        }
        flag = 3;
        ROS_INFO("================= CASE 4 =================");
    }

    pub_velocity.data = float(over) + float(under+1) / 10000;
    ROS_INFO("ANGLE : [%6.3f]도; LEFT : [%8.4f]cm; RIGHT: [%8.4f]cm ",steer_angle, left_dist*100, right_dist*100);
    pub_vel.publish(pub_velocity);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 500, scanCallback);
    pub_vel = n.advertise<std_msgs::Float32>("/velocity", 500);
    ros::spin();

    return 0;
}
