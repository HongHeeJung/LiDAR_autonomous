#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define THRESHOLD_DIS_DIFF1 0.20
#define THRESHOLD_DIS_DIFF2 0.70
#define THRESHOLD_DIS_DIFF3 1.2
#define GO_CENTER_ANG30 40
#define GO_CENTER_ANG50 75
#define GO_CENTER_ANG70 85

#define CENTER_STEER 1555
#define CENTER_PLUS1 50
#define CENTER_PLUS2 100

#define THRESHOLD_SPACE_RATE 1.4

rplidar_ros::Control pubToArdu;
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

void Find_space_diff(int *where_big,float *space_diff){
    float right_space = 0.0;
    float left_space = 0.0;
    for(int i = 110; i < 161; i += 1){  
        if( (distance_detected[i] < 2.5) && (distance_detected[360-i] < 2.5)){
            right_space += distance_detected[i];
            left_space += distance_detected[360-i];
        }
    }
    if(left_space <= right_space){
        *space_diff = right_space - left_space;
        *where_big = 2;
    }
    else if(left_space > right_space){
        *space_diff = left_space - right_space;
        *where_big = 1;
    }
}

float get_vel_steering(float streer_angle, int flag){
    int under = CENTER_STEER;
    float angle_for_Ardu = streer_angle;
    float angle_plus = 10;
    if(flag == 1 || flag == 2 ){
        int where_big; // 왼쪽이 크면 1, 오른쪽이 크면 2
        float space_diff;
        Find_space_diff(&where_big, &space_diff);

        if(flag == 1 && space_diff < 1){
            angle_plus = 2* space_diff;                                                  
        } 
        else if(flag == 1 && 1 <= space_diff && space_diff < 3 ){
            angle_plus = 2*space_diff;
        }
        else{
            if(flag == 1) angle_plus = 4*space_diff;
            if(flag == 2) angle_plus = 11*space_diff;
        }
     
        if(angle_plus >= 100) angle_plus = 100;

        if(where_big == 1){ // 왼쪽으로 가야함
            angle_for_Ardu = CENTER_STEER + angle_plus;
        }
        if(where_big == 2){ // 오른쪽으로 가야함
            angle_for_Ardu = CENTER_STEER - angle_plus;
        }
        ROS_INFO("SPACE_DIFF:[%7.4f];;   ANGLE-PLUS:[%7.4f];;   Where_big:[%d]",space_diff, angle_plus, where_big);
        under = int(angle_for_Ardu);

    }
    else{
        if(angle_for_Ardu < -30) angle_for_Ardu = CENTER_STEER - CENTER_PLUS2;
        else if(-30 <= angle_for_Ardu && angle_for_Ardu < -10)  angle_for_Ardu = CENTER_STEER - CENTER_PLUS1;
        else if(-10 <= angle_for_Ardu && angle_for_Ardu < 10)  angle_for_Ardu = CENTER_STEER;
        else if(10 <= angle_for_Ardu && angle_for_Ardu < 30)  angle_for_Ardu = CENTER_STEER + CENTER_PLUS1;
        else if(30 <= angle_for_Ardu)  angle_for_Ardu = CENTER_STEER + CENTER_PLUS2;
        under = int(angle_for_Ardu);
    }

    int Avg_vel=1517;
    if(flag == 1){
        if( (angle_plus <= 20) && (-6 <= streer_angle) && (streer_angle < 6) ){
            for(int i = 1; i < 21; i ++){
                if(( (20 - i*1) <= angle_plus ) && ( angle_plus < (20 - (i-1)*1)) ){
                    if(( -(6 - i*0.2) <= streer_angle) && (streer_angle < (6 - i*0.2) )){
                        Avg_vel = int(1550 + i * 4);
                        break;
                    }
                    Avg_vel = 1540 + i * 2;
                    break;
                } 
            }
        }
        else{
            Avg_vel = 1525;
        }      
    }
    else{
        if(flag == 2) Avg_vel = 1525;
        else if(flag == 3) Avg_vel = 1535;
        else if(flag == 4) Avg_vel = 1540;
    }
    int over = Avg_vel;

    return float(over) + float(under+1) / 10000;
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

    int flag = 1;
    float streer_angle = 0; // 자동차가 틀어야 하는 각도

    float dis_diff = (left_dist < right_dist)? right_dist - left_dist : left_dist - right_dist ;    
    if (dis_diff < THRESHOLD_DIS_DIFF1){
        streer_angle = theta - 90;
        flag = 1;
        ROS_INFO("========================== CASE 1 ==========================");
    }
    else if(dis_diff>= THRESHOLD_DIS_DIFF1 && dis_diff < THRESHOLD_DIS_DIFF2){
        if (left_dist < right_dist){
            streer_angle = theta - (180 - GO_CENTER_ANG70);
        }
        else if(left_dist > right_dist){
            streer_angle = theta - GO_CENTER_ANG70;
        }
        flag = 2;
        ROS_INFO("========================== CASE 2 ==========================");
    }
    else if(dis_diff>= THRESHOLD_DIS_DIFF2 && dis_diff < THRESHOLD_DIS_DIFF3){
        if (left_dist < right_dist){
            streer_angle = theta - (180 - GO_CENTER_ANG50);
        }
        else if(left_dist > right_dist){
            streer_angle = theta - GO_CENTER_ANG50;
        }
        flag = 3;
        ROS_INFO("========================== CASE 3 ==========================");
    }
    else{   // 중앙으로 가도록. 심각성 중 최상
        if (left_dist < right_dist){
            streer_angle = theta - (180 - GO_CENTER_ANG30);
        }
        else if(left_dist > right_dist){
            streer_angle = theta - GO_CENTER_ANG30;
        }
        flag = 4;
        ROS_INFO("========================== CASE 4 ==========================");
    }
    pub_velocity.data = get_vel_steering(streer_angle, flag);
    ROS_INFO("ANGLE:[  %6.3f]deg;;  LEFT:[  %8.4f]cm;;  RIGHT:[%8.4f]cm;;[%4.3f]",streer_angle, left_dist*100, right_dist*100, dis_diff);
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
