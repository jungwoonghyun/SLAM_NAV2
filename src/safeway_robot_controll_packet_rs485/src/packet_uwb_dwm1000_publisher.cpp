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
#include "message_interface/msg/uwbdist.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"

// UWB 설정을 위한 헤더파일
#include "dwm1000_init.hpp"

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

#define MOTION_CMD_BUFFER_SIZE_MAIN  256
#define arctanOneThree               0.32175055439664


static char *uwbDev[3] = {"/dev/ttyUWB0", "/dev/ttyUWB1", "/dev/ttyUSB2"};
/* uwb data */
int uwbfd[3];
unsigned int distance[3];
float dist_ancPos[3];
float dist_ancPos_raw[3];
float dist_ancPoc_bef[3];
float dist_ancPoc_raw_filter[3];
float dist_ancPoc_raw_bef[3];
float filter_dist[3];
float filter_dist_bef[3];
int errCnt[3] = {0};
int selector[3];
DWM1000_INFO dwm1000_readData[3];
UWBXYPlane UWB_TargetPos;
UWBRPIPlane UWB_DirNAng;
float spd = 0.0;
float spdBef = 0.0;
float spdftr = 0.0;
float angVel = 0.0;
float angVelBef = 0.0;
float angVelftr = 0.0;
int angle_sector = 0;

bool stopping_gap = false;

class PacketUwbDWM1000Publisher : public rclcpp::Node
{
public:
  PacketUwbDWM1000Publisher()
  : Node("packet_uwb_dwm1000_publisher")
  {
    //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 robot_cmd_vel, QoS
    packet_uwb_dwm1000_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", qos_profile);

    uwb_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "marker", qos_profile);

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 robot_cmd_vel, QoS
    uwb_distance_data_publisher_ = this->create_publisher<message_interface::msg::Uwbdist>(
      "uwb_distance", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      25ms, std::bind(&PacketUwbDWM1000Publisher::packet_uwb_msg, this));
  }

private:

  // 콜백 함수
  void packet_uwb_msg()
  {
    float target_x1, target_x2;
    float ang_vel_limit_val = 0.0;

    // Geometry_msgs::msg::Twist 타입으로 msg 선언
    geometry_msgs::msg::Twist twist;
    message_interface::msg::Uwbdist uwbdist_msg;
    visualization_msgs::msg::Marker marker;

    float distCalc[2] = {0};

    distance[0] = readDWM1000Data(&uwbfd[0],&dwm1000_readData[0]); // mm
    distance[1] = readDWM1000Data(&uwbfd[1],&dwm1000_readData[1]); // mm

    selector[0] = uwbPointSelector(&dwm1000_readData[0]);
    selector[1] = uwbPointSelector(&dwm1000_readData[1]);

    if(selector[0] < 0 || selector[1] < 0)
    {
      angVel =  0.0;
      spd = 0.0;
      RCLCPP_INFO(this->get_logger(), "No have our anchor.\r\n");
    }
    else
    {
      dist_ancPos_raw[selector[0]] = (float)distance[0];
      dist_ancPos_raw[selector[1]] = (float)distance[1];

      if(dist_ancPos_raw[0] < STAND_HALF_HEIGHT)
      {
        dist_ancPos[0] = 0.0;
      }
      else
      {
        dist_ancPos[0] = sqrtf(powf(dist_ancPos_raw[0],2.0) - powf(STAND_HALF_HEIGHT,2.0));
      }


      if(dist_ancPos_raw[1] < STAND_HALF_HEIGHT)
      {
        dist_ancPos[1] = 0.0;
      }
      else
      {
        dist_ancPos[1] = sqrtf(powf(dist_ancPos_raw[1],2.0) - powf(STAND_HALF_HEIGHT,2.0));
      }

      if(dist_ancPos[0] > 5500.0)
      {
        dist_ancPos[0] = dist_ancPoc_bef[0];
        errCnt[0]++;
      }
      else
      {
        dist_ancPoc_bef[0] = dist_ancPos[0];
        errCnt[0] = 0;
      }

      if(dist_ancPos[1] > 5500.0)
      {
        dist_ancPos[1] = dist_ancPoc_bef[1];
        errCnt[1]++;
      }
      else
      {
        dist_ancPoc_bef[1] = dist_ancPos[1];
        errCnt[1] = 0;
      }

      if(errCnt[0] > 60 || errCnt[1] > 60)
      {
        dist_ancPos[0] = 0.0;
        dist_ancPos[1] = 0.0;
      }

      filter_dist[0] = dist_ancPos[0] * LPF_VALUE + filter_dist_bef[0] * (1.0 - LPF_VALUE);
      filter_dist[1] = dist_ancPos[1] * LPF_VALUE + filter_dist_bef[1] * (1.0 - LPF_VALUE);
      filter_dist_bef[0] = filter_dist[0];
      filter_dist_bef[1] = filter_dist[1];

      dist_ancPoc_raw_filter[0] = dist_ancPos_raw[0] * 0.5 + dist_ancPoc_raw_bef[0] * (1.0 - 0.5);
      dist_ancPoc_raw_filter[1] = dist_ancPos_raw[1] * 0.5 + dist_ancPoc_raw_bef[1] * (1.0 - 0.5);
      dist_ancPoc_raw_bef[0] = dist_ancPoc_raw_filter[0];
      dist_ancPoc_raw_bef[1] = dist_ancPoc_raw_filter[1];

      if((-filter_dist[1] - ROBOT_SET_DIST) > (-filter_dist[0] + ROBOT_SET_DIST)) // 90deg right
      {
        UWB_TargetPos.y = -(filter_dist[0] + filter_dist[1]) / 2.0;
        UWB_TargetPos.x = 0.0;
        UWB_DirNAng.dist = (filter_dist[0] + filter_dist[1]) / 2.0;
        UWB_DirNAng.angle = -(M_PI/2.0);
      }
      else if((filter_dist[1] - ROBOT_SET_DIST) > (filter_dist[0] + ROBOT_SET_DIST)) // 90deg left
      {
        UWB_TargetPos.y = (filter_dist[1] + filter_dist[0]) / 2.0;
        UWB_TargetPos.x = 0.0;
        UWB_DirNAng.dist = (filter_dist[1] + filter_dist[0]) / 2.0;
        UWB_DirNAng.angle = (M_PI/2.0);
      }
      else if((-filter_dist[0] + ROBOT_SET_DIST) > (filter_dist[1] - ROBOT_SET_DIST)) // near center
      {
        UWB_TargetPos.y = 0.0;
        UWB_TargetPos.x = 0.0;
        UWB_DirNAng.dist = 0.0;
        UWB_DirNAng.angle = 0.0;
      }
      else
      {
        UWB_TargetPos.y = -(powf(filter_dist[0],2.0) - powf(filter_dist[1],2.0))/(4.0 * ROBOT_SET_DIST);
        target_x1 = sqrtf(powf(filter_dist[0],2.0) - powf((UWB_TargetPos.y - ROBOT_SET_DIST),2.0));
        target_x2 = sqrtf(powf(filter_dist[1],2.0) - powf((UWB_TargetPos.y + ROBOT_SET_DIST),2.0));
        UWB_TargetPos.x = (target_x1 + target_x2)/2.0;
        UWB_DirNAng.dist = sqrtf(powf(UWB_TargetPos.x, 2.0) + powf(UWB_TargetPos.y, 2.0));
        UWB_DirNAng.angle = asinf(UWB_TargetPos.y/UWB_DirNAng.dist);
      }

      marker.header.frame_id = "base_footprint";
      marker.header.stamp = this->now();
      marker.ns = "rtls_anc1_detect";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action =  visualization_msgs::msg::Marker::MODIFY;
      marker.pose.position.x = (double)UWB_TargetPos.x * 0.001;
      marker.pose.position.y = (double)UWB_TargetPos.y * 0.001;
      marker.pose.position.z = 0.0;
      marker.scale.x = 0.08;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      if(UWB_DirNAng.dist > 1650.0)
      {
        spd = 1000.0 * (2.2 / 3.0);
        ang_vel_limit_val = 0.75;
      }
      else if((UWB_DirNAng.dist > 650.0) && (UWB_DirNAng.dist <= 750.0))
      {
        if(stopping_gap == true)
        {
          spd = 0.0;
          ang_vel_limit_val = 1.0;
        }
        else
        {
          spd = (UWB_DirNAng.dist - 650.0) * (2.2 / 3.0);
          ang_vel_limit_val = 0.75 + ((1650.0 - UWB_DirNAng.dist) / 4000.0);
        }
      }
      else if(UWB_DirNAng.dist > 750.0)
      {
        spd = (UWB_DirNAng.dist - 650.0) * (2.2 / 3.0);
        ang_vel_limit_val = 0.75 + ((1650.0 - UWB_DirNAng.dist) / 4000.0);
        stopping_gap = false;
      }
      else if(UWB_DirNAng.dist <= 650.0)
      {
        spd = 0.0;
        ang_vel_limit_val = 1.0;
        stopping_gap = true;
      }


      angle_sector = (int)(UWB_DirNAng.angle / 0.07);

      if(UWB_DirNAng.dist < 800.0)
      {
        if((UWB_DirNAng.angle < 0.20) && (UWB_DirNAng.angle > -0.20))
        {
          angVel = 0.0;
        }
        else if(UWB_DirNAng.angle > 0.20)
        {
          angVel = (UWB_DirNAng.angle - 0.20);// * ang_vel_limit_val;
        }
        else if(UWB_DirNAng.angle < -0.20)
        {
          angVel = (UWB_DirNAng.angle + 0.20);// * ang_vel_limit_val;
        }
      }
      else
      {
        if(UWB_DirNAng.dist * sinf(UWB_DirNAng.angle) > 160.00)
        {
          angVel = (UWB_DirNAng.angle - fabsf(asinf(160.00/UWB_DirNAng.dist)));// * ang_vel_limit_val;
        }
        else if(UWB_DirNAng.dist * sinf(UWB_DirNAng.angle) < -160.00)
        {
          angVel = (UWB_DirNAng.angle + fabsf(asinf(160.00/UWB_DirNAng.dist)));// * ang_vel_limit_val;
        }
        else
        {
          angVel = 0.0;
        }
      }

//      angVel = (float)angle_sector * 0.15;
    }

    spdftr = spd * MOV_LPF_VALUE + spdBef * (1.0 - MOV_LPF_VALUE);
    angVelftr = angVel * (MOV_LPF_VALUE) + angVelBef * (1.0 - (MOV_LPF_VALUE));

    spdBef = spdftr; // target
    angVelBef = angVelftr; // target

    twist.angular.z = angVelftr; //angVelftr;
    twist.linear.x = spdftr * 0.001; //out_dire * 0.001; // mm -> m

    RCLCPP_INFO(this->get_logger(), "%X %X , %8.2f , %8.2f, %X %X , %8.2f , %8.2f , spd : %.2f, angvel : %.2f, sector : %d",
    dwm1000_readData[selector[0]].shtAddr_dev[0],dwm1000_readData[selector[0]].shtAddr_dev[1], filter_dist[0], dist_ancPos_raw[0],
    dwm1000_readData[selector[1]].shtAddr_dev[0],dwm1000_readData[selector[1]].shtAddr_dev[1], filter_dist[1], dist_ancPos_raw[1],
    spdftr, angVelftr, angle_sector); //%8.2f

    uwbdist_msg.tag1dist = dist_ancPoc_raw_filter[0];
    uwbdist_msg.tag2dist = dist_ancPoc_raw_filter[1];
    uwbdist_msg.tag1filtdist = filter_dist[0];
    uwbdist_msg.tag2filtdist = filter_dist[1];
    uwbdist_msg.calcdata = UWB_DirNAng.dist;

    // publishing
    packet_uwb_dwm1000_publisher_->publish(twist);
    uwb_distance_data_publisher_->publish(uwbdist_msg);
    uwb_marker_publisher_->publish(marker);
  }

  // private 변수
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr packet_uwb_dwm1000_publisher_;
  rclcpp::Publisher<message_interface::msg::Uwbdist>::SharedPtr uwb_distance_data_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr uwb_marker_publisher_;
  //size_t count_;
};

void dwm1000_settings_func()
{
  openDevice(uwbDev[0], &uwbfd[0]);
  msleep(10);
  openDevice(uwbDev[1], &uwbfd[1]);
  msleep(10);
//  openDevice(uwbDev[2], &uwbfd[2]);
//  msleep(10);
  printf("All serial socket start\r\n");
  msleep(200);
}

void dwm1000_end_func()
{
  close(uwbfd[0]);
  msleep(10);
  close(uwbfd[1]);
  msleep(10);
//  openDevice(uwbDev[2], &uwbfd[2]);
//  msleep(10);
  printf("All serial socket end\r\n");
  msleep(200);
}

int main(int argc, char * argv[])
{
  // rclcpp 초기화
  rclcpp::init(argc, argv);

  // UWB settings
  dwm1000_settings_func();

  // 클래스 생성
  auto node = std::make_shared<PacketUwbDWM1000Publisher>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();
  dwm1000_end_func();

  return 0;
}
