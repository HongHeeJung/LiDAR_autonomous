/*
    Made by JunHa Song
    각종 주의 사항 :
    1. 센서 데이터 중 inf값은 infinite값으로 처리되며, 가장 작은 수가 될 수 없다. 
*/


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rplidar_ros/Control.h"
#include "std_msgs/Float32.h"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define AVE_VELOCIY 100
#define THRESHOLD_DIS_RATE1 1.4
#define THRESHOLD_DIS_RATE2 2.5
#define GO_CENTER_ANG30 45
#define GO_CENTER_ANG60 68
#define THRESHOLD_SPACE_RATE 1.4
#define DIVIED_ANGLE 3
#define CENTER_STEER 1550
#define CENTER_PLUS 120 
 

rplidar_ros::Control pubToArdu;
std_msgs::Float32 pub_velocity;
std_msgs::Float32 pub_steer;
ros::Publisher pub;
ros::Publisher pub_vel;
ros::Publisher pub_steer_angle;

// 알고리즘 구현에 필요한 전역변수
float angle_detected[400] = {0.0,}; 
float distance_detected[400] = {0.0,}; 

int cur_index = 0;
float angle_min[3] = {-180.0,0,180.0};
float left_dist_min[3] = {1 ,1 ,1};
float right_dist_min[3] = {1 ,1 ,1};


int min_Total_distance(int count){
    // 마주보는 각도의 거리의 합이 최소인 index를 찾아서 return 한다.
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

int middle_angle_index() {
    // angle_min 에 있는 값 중 중간값 인덱스 반환
    float a = angle_min[0]; 
    float b = angle_min[1]; 
    float c = angle_min[2];
    if ((b >= a && c <= a) || (b <= a && c >= a)) return 0;
    if ((a >= b && c <= b) || (a <= b && c >= b)) return 1;
    else return 2;
}

float angle_for_Ardu(float streer_angle){
    float angle_for_Ardu = streer_angle/DIVIED_ANGLE;
    if(angle_for_Ardu < -15) angle_for_Ardu = CENTER_STEER - 2*CENTER_PLUS; // 오른쪽으로 가야함 
    else if(-15 <= angle_for_Ardu && angle_for_Ardu < -5)  angle_for_Ardu = CENTER_STEER - CENTER_PLUS;
    else if(-5 <= angle_for_Ardu && angle_for_Ardu < 5)  angle_for_Ardu = CENTER_STEER;  // 직선 주행
    else if(5 <= angle_for_Ardu && angle_for_Ardu < 15)  angle_for_Ardu = CENTER_STEER + CENTER_PLUS; //
    else if(15 <= angle_for_Ardu)  angle_for_Ardu = CENTER_STEER + 2*CENTER_PLUS;  // 왼쪽으로 가야함
    return angle_for_Ardu;
}

float velocity_for_Ardu(float Avg_vel, int flag){
    // 이 방법이 성공하면 Avg_vel * 0.6 or 0.3 or 0.2 등의 연산은 필요없다.
    if(flag == 0) Avg_vel = 1600;
    else if(flag == 1) Avg_vel = 1547;
    else if(flag == 2) Avg_vel = 1527;
    else if(flag == 3) Avg_vel = 1517;
    
    return Avg_vel;
}

float Find_state_rate(){
    // 곡선 주행 여부를 판별한다.
    float right_space = 0.0;
    float left_space = 0.0;
    for(int i = 110; i < 131; i++){
        if(distance_detected[i] < 15 && distance_detected[360-i] < 15){
            left_space += distance_detected[i];
            right_space += distance_detected[360-i];
        }
    }
    return (left_space < right_space)? right_space/left_space : left_space/right_space ; 
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
// 0. 알고리즘 구현에 필요한 변수 정의.
    int count = scan->scan_time / scan->time_increment; 
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        angle_detected[i] = degree;  
        distance_detected[i] = scan->ranges[i];
    }

    /*
    사용할 수 있는 변수
    1. count : 한 바퀴 회전하면서, 얻은 포인트의 갯수
    2. scan->angle_mim : 아래의 angle_detected 내부의 값 중 최솟값
    3. scan->angle_max : 아래의 angle_detected 내부의 값 중 최댓값
    4. angle_detected[i] : 바로 위의 for에 의해, [0~count-1]까지 값이 수정된다.  -180 ~ 0 ~ +180 값이 순서대로 들어가 있다.
    5. distance_detected[i] : 바로 위의 for에 의해, [0~count-1]까지 값이 수정된다. 
    디버깅 : 
    ROS_INFO("min index = %d]:", min_angle_index);
    ROS_INFO("angle_min = %f  %f  %f]:", angle_min[0],angle_min[1],angle_min[2] );
    ROS_INFO("angle_min =%f  %f  %f  %f]:",angle_min[mid_index] ,theta, left_dist, right_dist);
    */
    // 알고리즘 구현 START ************************************************************************************************************

    // 1. 직선에 대한, 차량의 각도 찾기. 센서 오류를 피하기 위해 라이더가 3바퀴 돈 값을 사용한다.
    int min_angle_index = min_Total_distance(count); // 0부터 180까지의 인덱스 값 반환
    angle_min[cur_index] = angle_detected[min_angle_index + count/2]; // 180이상의 인덱스 양수 범위
    right_dist_min[cur_index] = distance_detected[min_angle_index];
    left_dist_min[cur_index] = distance_detected[min_angle_index + count/2];
    cur_index++;
    if(cur_index == 3) cur_index = 0;
    
    // 2. 각도에 대한 중간값을 찾고, 그 각도와 그 때의 거리를 이용한다. 
    int mid_index = middle_angle_index();  
    float theta = angle_min[mid_index] ; // theta - 90 : 직선 주행을 하기 위해 자동차가 틀어야하는 각도. theta : 라이저 정면 vs 직선 라인의 수선 사이의 각도. 0~180 도
    float left_dist = left_dist_min[mid_index]; //  
    float right_dist =  right_dist_min[mid_index]; 
    
    float Avg_vel = AVE_VELOCIY * ( left_dist + right_dist);
    int flag = 0;
    float streer_angle = 0; // 자동차가 틀어야 하는 각도 
    float dis_rate = (left_dist < right_dist)? right_dist/left_dist : left_dist/right_dist ;     
    if (dis_rate < THRESHOLD_DIS_RATE1){
        streer_angle = theta - 90; 
	flag = 1;
        ROS_INFO("========================== CASE 1 ==========================");
    }
    else if(dis_rate>= THRESHOLD_DIS_RATE1 && dis_rate < THRESHOLD_DIS_RATE2){
        if (left_dist < right_dist){
            streer_angle = theta - (180 - GO_CENTER_ANG60);
        }
        else if(left_dist > right_dist){
            streer_angle = theta - GO_CENTER_ANG60;
        }
        Avg_vel = Avg_vel * 0.6;
        flag = 2;
        ROS_INFO("========================== CASE 2 ==========================");
    }
    else
    {
        if (left_dist < right_dist){
            streer_angle = theta - (180 - GO_CENTER_ANG30);
        }
        else if(left_dist > right_dist){
            streer_angle = theta - GO_CENTER_ANG30;
        }
        Avg_vel = Avg_vel * 0.3;
        flag = 3;
        ROS_INFO("========================== CASE 2 ==========================");
    }
    

    if(left_dist < 0.16 || right_dist < 0.16) {
        Avg_vel = Avg_vel * 0.2;  // 벽에 너무 가까이 있으면 속도를 매우 낮춘다.
        flag = 4;
        ROS_INFO("========================== CASE 2 ==========================");
    }

    pub_velocity.data = velocity_for_Ardu(Avg_vel,flag);                     
    pub_steer.data = angle_for_Ardu(streer_angle);
    ROS_INFO("theta : %f , left distance : %f , right distance : %f]:", theta,left_dist,right_dist );        
    ROS_INFO("Final Velocity= %f  ,streer_angle : %f ", pub_velocity.data, pub_steer.data);

    pub_vel.publish(pub_velocity);
    pub_steer_angle.publish(pub_steer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, scanCallback);
    pub_vel = n.advertise<std_msgs::Float32>("/velocity", 100);	
    pub_steer_angle = n.advertise<std_msgs::Float32>("/streer_angle", 100);	

    ros::spin();

    return 0;
}
