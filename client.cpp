/*
    Made by JunHa Song
    각종 주의 사항 :
    1. 센서 데이터 중 inf값은 infinite값으로 처리되며, 가장 작은 수가 될 수 없다. 
*/


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rplidar_ros/Control.h"
#include "math.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define AVE_VELOCIY 1600
#define THRESHOLD_DIS_RATE1 1.4
#define THRESHOLD_DIS_RATE2 2.5
#define GO_CENTER_ANG30 45
#define GO_CENTER_ANG60 68
#define THRESHOLD_SPACE_RATE 1.4
#define DIVIED_ANGLE 3
 

rplidar_ros::Control pubToArdu;
ros::Publisher pub;

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
    float angle_for_Ardu = streer_angle/DIVIED_ANGLE + 90;
    if(angle_for_Ardu < 75) angle_for_Ardu = 70.0; // 오른쪽으로 가야함 
    else if(75 <= angle_for_Ardu && angle_for_Ardu < 85)  angle_for_Ardu = 80.0;
    else if(85 <= angle_for_Ardu && angle_for_Ardu < 95)  angle_for_Ardu = 90.0;  // 직선 주행
    else if(95 <= angle_for_Ardu && angle_for_Ardu < 105)  angle_for_Ardu = 100.0; //
    else if(105 <= angle_for_Ardu)  angle_for_Ardu = 110.0;  // 왼쪽으로 가야함
    return angle_for_Ardu;
}

float velocity_for_Ardu(float Avg_vel){
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
    ROS_INFO("theta : %f , left distance : %f , right distance : %f]:", theta,left_dist,right_dist );

    float Avg_vel = AVE_VELOCIY * ( left_dist + right_dist);
    // ROS_INFO("left_dist + right_dist : %f, Avg_vel : %f", left_dist + right_dist, Avg_vel);

    float streer_angle = 0; // 자동차가 틀어야 하는 각도
    // ROS_INFO("Avg_vel= %f  ,streer_angle : %f ", Avg_vel, streer_angle);
    // 3. 임계 각도에 따라서, {직진 하다록}} VS {중앙을 가도록} 을 선택한다. 
    float dis_rate = (left_dist < right_dist)? right_dist/left_dist : left_dist/right_dist ;     
    if (dis_rate < THRESHOLD_DIS_RATE1){  // 직진 하도록
        streer_angle = theta - 90; 

        /*
        이 주석을 풀면, 코너를 만나면 속도를 절반으로 줄인다.
        float rotate_rate = Find_state_rate();
        if(rotate_rate >  THRESHOLD_SPACE_RATE){
            Avg_vel = Avg_vel/2; 
            ROS_INFO("go straight But soon coner");
        } 
        */
        ROS_INFO("go straight");
    }
    else if(dis_rate>= THRESHOLD_DIS_RATE1 && dis_rate < THRESHOLD_DIS_RATE2){   // 중앙으로 가도록. 심각성 중
        if (left_dist < right_dist){
            streer_angle = theta - (180 - GO_CENTER_ANG60);
        }
        else if(left_dist > right_dist){
            streer_angle = theta - GO_CENTER_ANG60;
        }
        Avg_vel = Avg_vel * 0.6;
        ROS_INFO("go to center slowly");
    }
    else // 중앙으로 가도록. 심각성 상
    {
        if (left_dist < right_dist){
            streer_angle = theta - (180 - GO_CENTER_ANG30);
        }
        else if(left_dist > right_dist){
            streer_angle = theta - GO_CENTER_ANG30;
        }
        Avg_vel = Avg_vel * 0.3;
        ROS_INFO("go to center urgently");
    }
    

    if(left_dist < 0.20 || right_dist < 0.2) {
        Avg_vel = Avg_vel * 0.2;  // 벽에 너무 가까이 있으면 속도를 매우 낮춘다.
        ROS_INFO("Too close to the wall");
    }
    // 알고리즘 구현 END **************************************************************************************************************
    // 위의 구현 부분에 다음의 변수에 값을 대입해야 한다.
    ROS_INFO("Before precess __for_Ardu fuc, Velocity= %lf  , streer_angle : %lf ", Avg_vel , streer_angle);
    pubToArdu.velocity_on = true;           // 이 변수의 type = Bool
    pubToArdu.streer_on = false;            // 이 변수의 type = Bool
    pubToArdu.velocity = velocity_for_Ardu(Avg_vel);                     
    pubToArdu.streer_angle = angle_for_Ardu(streer_angle);             
    // ROS_INFO("Final Avg_vel= %f  ,Final streer_angle : %f ", pubToArdu.velocity, pubToArdu.streer_angle);

    pub.publish(pubToArdu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    pub = n.advertise<rplidar_ros::Control>("/ToArdu", 1000);	

    ros::spin();

    return 0;
}
