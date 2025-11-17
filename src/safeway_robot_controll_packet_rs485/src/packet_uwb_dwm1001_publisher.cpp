#include <chrono>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <errno.h>
#include <linux/types.h>
#include <math.h>
#include <functional>
#include <memory>
#include <string>

// Odometry 좌표상의 로봇 기구학 계산을 보다 쉽게 하기 위하여,
// ROS2 에서 기본적으로 제공하는 geometry_msgs/msg/twist 를 사용하기 위한 헤더 파일
#include "geometry_msgs/msg/twist.hpp"

// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"

// UWB 설정을 위한 헤더파일
#include "uwb_init.hpp"

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

#define MOTION_CMD_BUFFER_SIZE_MAIN  256
#define arctanOneThree               0.32175055439664

static char *uwbDev[3] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"};

typedef enum _enum_OPERATION_TYPE
{
    OPERATION_UNKNOWN = 0,
    OPERATION_FORWARD,
    OPERATION_LEFT,
    OPERATION_RIGHT,
    OPERATION_BACKWARD,
    OPERATION_STOP
} OPERATION_TYPE;

// buffer for receciving the reply
static char ms_szCmdRBuffer[MOTION_CMD_BUFFER_SIZE_MAIN];

// buffer for seding the commands
static char ms_szCmdWBuffer[MOTION_CMD_BUFFER_SIZE_MAIN];

static int ms_nSendIndex = 0;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t t_mutex = PTHREAD_MUTEX_INITIALIZER;

#define BUF_MAX 512
DWMDATA_tag uwbData[3];
DWMNode_ID  uwbID[3];

int thread_terminate_val = 0;

/* uwb data */
int uwbfd[3];
unsigned int distance[3];
double filtered_dist[3];
unsigned int befDis[3];

int befErr[3] = {0,0,0};
int sameCnt[3] = {0,0,0};
int distErrorCheck[3] = {0, 0, 0};

/* distance data */
double ThrPtDist[3] = {0, };
double ThrPtDistBef[3] = {0, };
UWBXYPlane UWB_Position[3] = {{0.0, 0.0},{UWB_3TAG_XPOS, 0.0},{0.0, UWB_3TAG_YPOS}};
UWBXYPlane UWB_TargetPos;
UWBXYPlane UWB_CenterPos = {UWB_Center_X, UWB_Center_Y};
UWBRPIPlane UWB_TargetDirNDis;

// rclcpp의 Node 클래스를 상속하여 사용
class PacketUwbDWM1001Publisher : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출, 노드 이름을 packet_uwb_dwm1001_publisher 지정, count_는 0으로 초기화
  PacketUwbDWM1001Publisher()
  : Node("packet_uwb_dwm1001_publisher")
  {
    //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 robot_cmd_vel, QoS
    packet_uwb_dwm1001_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      33ms, std::bind(&PacketUwbDWM1001Publisher::packet_uwb_msg, this));
  }

private:
  // keyboard 입력에 따른 속력, 회전각도 조절 및 scale 변수
  double linear_ = 0.0, angular_ = 0.0;
  int linear_dir = 1, angular_dir = 1;
  int selector[3] = {0, };
  int sel_num = 0;
  double angle_comp = 0.0;
  double set_bottom = 0.0;
  double set_hei = 0.0;
  double con_angle = 0.0;

  // 콜백 함수
  void packet_uwb_msg()
  {
    // Geometry_msgs::msg::Twist 타입으로 msg 선언
    geometry_msgs::msg::Twist twist;

    distance[sel_num] = readDWMLocData(&uwbfd[sel_num], &uwbData[sel_num]);
    selector[sel_num] = uwbPointSelector(&uwbID[sel_num], &uwbData[sel_num]);

    if(distance[sel_num] <= 0){
			befErr[sel_num] = 1;
		}
    else if(distance[sel_num] == befDis[sel_num]){
      if(sameCnt[sel_num] > 10){
        sameCnt[sel_num] = 10;
        befErr[sel_num] = 1;
      }
      else{
        sameCnt[sel_num]++;
      }
    }
    else if(befErr[sel_num]){
      sameCnt[sel_num] = 0;
      distErrorCheck[sel_num]++;
      befErr[sel_num] = 0;
    }
    else{
      sameCnt[sel_num] = 0;
    }

    /* change uwb data -> distance data */
    if(!befErr[sel_num]){
      filtered_dist[selector[sel_num]] = (double)distance[sel_num] * (1.0 - LOWPASSFILTER_VEL) + (double)befDis[sel_num] * LOWPASSFILTER_VEL;
    }
    else{
      filtered_dist[selector[sel_num]] = 0.0;
    }

    befDis[sel_num] = distance[sel_num];

    sel_num++;
    if(sel_num > 2)
    {
      sel_num = 0;
    }

    ThrPtDist[sel_num] = filtered_dist[sel_num] * (1.0 - LOWPASSFILTER_PROCESSVEL) + ThrPtDistBef[sel_num] * LOWPASSFILTER_PROCESSVEL;
    ThrPtDistBef[sel_num] = ThrPtDist[sel_num];
//    ThrPtDist[1] = filtered_dist[1] * (1.0 - LOWPASSFILTER_PROCESSVEL) + ThrPtDistBef[1] * LOWPASSFILTER_PROCESSVEL;
//    ThrPtDist[2] = filtered_dist[2] * (1.0 - LOWPASSFILTER_PROCESSVEL) + ThrPtDistBef[2] * LOWPASSFILTER_PROCESSVEL;

    /* x-position calculate */
    if((-ThrPtDist[0]) > (UWB_Position[1].x - ThrPtDist[1])){
      UWB_TargetPos.x = ((UWB_Position[1].x - ThrPtDist[1]) - ThrPtDist[0]) / 2.0;
    }
    else if((UWB_Position[1].x + ThrPtDist[1]) < ThrPtDist[0]){
      UWB_TargetPos.x = ((UWB_Position[1].x + ThrPtDist[1]) + ThrPtDist[0]) / 2.0;
    }
    else{
      UWB_TargetPos.x = (pow(ThrPtDist[0], 2.0) - pow(ThrPtDist[1], 2.0) + pow(UWB_Position[1].x, 2.0))/(2.0 * UWB_Position[1].x);
    }

    /* y-position calculate */
    if((-ThrPtDist[0]) > (UWB_Position[2].y - ThrPtDist[2])){
      UWB_TargetPos.y = ((UWB_Position[2].y - ThrPtDist[2]) - ThrPtDist[0]) / 2.0;
    }
    else if((UWB_Position[2].y + ThrPtDist[2]) < ThrPtDist[0]){
      UWB_TargetPos.y = ((UWB_Position[2].y + ThrPtDist[2]) + ThrPtDist[0]) / 2.0;
    }
    else{
      UWB_TargetPos.y = (pow(ThrPtDist[0], 2.0) - pow(ThrPtDist[2], 2.0) + pow(UWB_Position[2].y, 2.0))/(2.0 * UWB_Position[2].y);
    }

    /* calculate distance */
    UWB_TargetDirNDis.dist = sqrt(pow(UWB_TargetPos.x - UWB_CenterPos.x, 2.0) + pow(UWB_TargetPos.y - UWB_CenterPos.y, 2.0));

    /* calculate direction from wheel center position */
    if(UWB_TargetDirNDis.dist == 0.0){
      angle_comp = 0.0;
    }
    else{
      angle_comp = (UWB_TargetPos.x - UWB_CenterPos.x) / UWB_TargetDirNDis.dist;
    }

    if(angle_comp > 1.0){
      angle_comp = 1.0;
    }
    else if(angle_comp < -1.0){
      angle_comp = -1.0;
    }

    UWB_TargetDirNDis.angle = asin(angle_comp);

    set_bottom = UWB_TargetDirNDis.dist * sin(UWB_TargetDirNDis.angle);
    set_hei = sqrt(pow(UWB_TargetDirNDis.dist, 2.0) - pow(set_bottom, 2.0));
    con_angle = atan(200.0/set_hei);

    // double linear_ = 0.0, angular_ = 0.0;
    if(UWB_TargetDirNDis.dist > 600.0)
    {
      linear_ = (UWB_TargetDirNDis.dist - 600.0);

      if((set_bottom < 200.0) && (set_bottom > -200.0))
      {
        angular_ = 0.0;
      }
      else if(set_bottom >= 200.0)
      {
        angular_ = UWB_TargetDirNDis.angle - con_angle;
      }
      else if(set_bottom <= -200.0)
      {
        angular_ = UWB_TargetDirNDis.angle + con_angle;
      }
    }
    else
    {
      linear_ = 0.0;

      if(UWB_TargetDirNDis.angle < arctanOneThree && UWB_TargetDirNDis.angle > -arctanOneThree)
      {
        angular_ = 0.0;
      }
      else if(UWB_TargetDirNDis.angle >= arctanOneThree) //(M_PI / 6.0)
      {
        angular_ = UWB_TargetDirNDis.angle - arctanOneThree;
      }
      else if(UWB_TargetDirNDis.angle <= -arctanOneThree)
      {
        angular_ = UWB_TargetDirNDis.angle + arctanOneThree;
      }
    }

    // speed limit
    if(linear_ > 4000.0)
    {
      linear_ = 4000.0;
    }

    if(angular_ > (M_PI/2.0))
    {
      angular_ = M_PI/2.0;
    }
    else if(angular_ < -(M_PI/2.0))
    {
      angular_ = -M_PI/2.0;
    }

    // Twist message 에 계산한 결과데이터 입력
    twist.angular.z = angular_;
    twist.linear.x = linear_ * 0.001; // cm -> m

    // logging, RCLCPP_XXX 계열의 함수는 print와 비슷
    RCLCPP_INFO(this->get_logger(), "'%8.2f','%8.2f','%8.2f'|'%8.2f'|'%6.2f','%6.2f'", ThrPtDist[0], ThrPtDist[1], ThrPtDist[2], set_hei ,angular_, linear_);
//    RCLCPP_INFO(this->get_logger(), "'%8.2f','%8.2f','%8.2f'|'%8.2f','%8.2f','%8.2f'", (double)distance[0], (double)distance[1], (double)distance[2], ThrPtDist[0], ThrPtDist[1], ThrPtDist[2]);


    // publishing
    packet_uwb_dwm1001_publisher_->publish(twist);
  }

  // private 변수
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr packet_uwb_dwm1001_publisher_;
  //size_t count_;
};

/*
distance data information in robot

            ********************
            *  DAB8            *
            *  (2)             *
            *                  *
            *                  *
wheel       *  CFA7      5780  * wheel
(1st motor) *  (0)       (1)   * (2nd motor)
            ********************
*/

void uwb_settings_func()
{
  openDevice(uwbDev[0], &uwbfd[0]);
  openDevice(uwbDev[1], &uwbfd[1]);
  openDevice(uwbDev[2], &uwbfd[2]);
  printf("All serial socket start\r\n");
  msleep(50);
  readDWMNodeNum(&uwbfd[0], &uwbID[0]);
  readDWMNodeNum(&uwbfd[1], &uwbID[1]);
  readDWMNodeNum(&uwbfd[2], &uwbID[2]);
  msleep(200);
}

int main(int argc, char * argv[])
{
  // rclcpp 초기화
  rclcpp::init(argc, argv);

  // UWB settings
  uwb_settings_func();

  // 클래스 생성
  auto node = std::make_shared<PacketUwbDWM1001Publisher>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();

  return 0;
}
