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
#include "visualization_msgs/msg/marker.hpp"
#include "message_interface/msg/uwbdist.hpp"

// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"

// UWB 설정을 위한 헤더파일
#include "dwm1000_init.hpp"

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

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

int marker_number = 0;

class PacketRTLSSystem : public rclcpp::Node
{
public:
  PacketRTLSSystem()
  : Node("packet_rtls_system")
  {
    //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 robot_cmd_vel, QoS
    rtls_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "marker", qos_profile);

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 robot_cmd_vel, QoS
    rtls_distance_data_publisher_ = this->create_publisher<message_interface::msg::Uwbdist>(
      "uwb_distance", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      25ms, std::bind(&PacketRTLSSystem::rtls_service, this));
  }

private:

  void rtls_service()
  {
    float target_x1, target_x2;
    message_interface::msg::Uwbdist rtls_msg;
    visualization_msgs::msg::Marker marker[5];

    float distCalc[2] = {0};

    distance[0] = readDWM1000Data(&uwbfd[0],&dwm1000_readData[0]); // mm
    distance[1] = readDWM1000Data(&uwbfd[1],&dwm1000_readData[1]); // mm

    selector[0] = uwbPointSelector(&dwm1000_readData[0]);
    selector[1] = uwbPointSelector(&dwm1000_readData[1]);

    if(selector[0] < 0 || selector[1] < 0)
    {
      RCLCPP_INFO(this->get_logger(), "No have our anchor.\r\n");
    }

    else
    {
      dist_ancPos[selector[0]] = (float)distance[0];
      dist_ancPos[selector[1]] = (float)distance[1];

      filter_dist[0] = dist_ancPos[0] * LPF_VALUE + filter_dist_bef[0] * (1.0 - LPF_VALUE);
      filter_dist[1] = dist_ancPos[1] * LPF_VALUE + filter_dist_bef[1] * (1.0 - LPF_VALUE);
      filter_dist_bef[0] = filter_dist[0];
      filter_dist_bef[1] = filter_dist[1];

      if((-filter_dist[1] - RTLS_SET_DIST) > (-filter_dist[0] + RTLS_SET_DIST)) // 90deg right
      {
        UWB_TargetPos.y = -(filter_dist[0] + filter_dist[1]) / 2.0;
        UWB_TargetPos.x = 0.0;
        UWB_DirNAng.dist = (filter_dist[0] + filter_dist[1]) / 2.0;
        UWB_DirNAng.angle = -(M_PI/2.0);
      }
      else if((filter_dist[1] - RTLS_SET_DIST) > (filter_dist[0] + RTLS_SET_DIST)) // 90deg left
      {
        UWB_TargetPos.y = (filter_dist[1] + filter_dist[0]) / 2.0;
        UWB_TargetPos.x = 0.0;
        UWB_DirNAng.dist = (filter_dist[1] + filter_dist[0]) / 2.0;
        UWB_DirNAng.angle = (M_PI/2.0);
      }
      else if((-filter_dist[0] + RTLS_SET_DIST) > (filter_dist[1] - RTLS_SET_DIST)) // near center
      {
        UWB_TargetPos.y = 0.0;
        UWB_TargetPos.x = 0.0;
        UWB_DirNAng.dist = 0.0;
        UWB_DirNAng.angle = 0.0;
      }
      else
      {
        UWB_TargetPos.y = -(powf(filter_dist[0],2.0) - powf(filter_dist[1],2.0))/(4.0 * RTLS_SET_DIST);
        target_x1 = sqrtf(powf(filter_dist[0],2.0) - powf((UWB_TargetPos.y - RTLS_SET_DIST),2.0));
        target_x2 = sqrtf(powf(filter_dist[1],2.0) - powf((UWB_TargetPos.y + RTLS_SET_DIST),2.0));
        UWB_TargetPos.x = (target_x1 + target_x2)/2.0;
        UWB_DirNAng.dist = sqrtf(powf(UWB_TargetPos.x, 2.0) + powf(UWB_TargetPos.y, 2.0));
        UWB_DirNAng.angle = asinf(UWB_TargetPos.y/UWB_DirNAng.dist);
      }
    }

    switch(marker_number)
    {
      case 0:
        marker[0].header.frame_id = "odom";
        marker[0].header.stamp = this->now();
        marker[0].ns = "rtls_anc1_detect";
        marker[0].id = 0;
        marker[0].type = visualization_msgs::msg::Marker::SPHERE;
        marker[0].action =  visualization_msgs::msg::Marker::ADD;
        marker[0].pose.position.x = (double)0.0;
        marker[0].pose.position.y = (double)(RTLS_SET_DIST * 0.001);
        marker[0].pose.position.z = 0.0;
        marker[0].scale.x = 0.08;
        marker[0].scale.y = 0.08;
        marker[0].scale.z = 0.08;
        marker[0].color.a = 1.0;
        marker[0].color.r = 0.0;
        marker[0].color.g = 1.0;
        marker[0].color.b = 0.0;

        rtls_marker_publisher_->publish(marker[0]);
        break;
      case 1:
        marker[1].header.frame_id = "odom";
        marker[1].header.stamp = this->now();
        marker[1].ns = "rtls_anc1_circle";
        marker[1].id = 0;
        marker[1].type = visualization_msgs::msg::Marker::CYLINDER;
        marker[1].action =  visualization_msgs::msg::Marker::ADD;
        marker[1].pose.position.x = (double)0.0;
        marker[1].pose.position.y = (double)(RTLS_SET_DIST * 0.001);
        marker[1].pose.position.z = 0.0;
        marker[1].scale.x = (double)(filter_dist[0] * 0.002);
        marker[1].scale.y = (double)(filter_dist[0] * 0.002);
        marker[1].scale.z = 0.001;
        marker[1].color.a = 0.5;
        if(filter_dist[0] < 650.0)
        {
          marker[1].color.r = 1.0;
          marker[1].color.g = 1.0;
          marker[1].color.b = 1.0;
        }
        else
        {
          marker[1].color.r = 1.0;
          marker[1].color.g = 1.0;
          marker[1].color.b = 0.0;
        }

        rtls_marker_publisher_->publish(marker[1]);
        break;
      case 2:
        marker[2].header.frame_id = "odom";
        marker[2].header.stamp = this->now();
        marker[2].ns = "rtls_anc2_detect";
        marker[2].id = 0;
        marker[2].type = visualization_msgs::msg::Marker::SPHERE;
        marker[2].action =  visualization_msgs::msg::Marker::ADD;
        marker[2].pose.position.x = (double)0.0;
        marker[2].pose.position.y = (double)(-RTLS_SET_DIST * 0.001);
        marker[2].pose.position.z = 0.0;
        marker[2].scale.x = 0.08;
        marker[2].scale.y = 0.08;
        marker[2].scale.z = 0.08;
        marker[2].color.a = 1.0;
        marker[2].color.r = 0.0;
        marker[2].color.g = 1.0;
        marker[2].color.b = 0.0;

        rtls_marker_publisher_->publish(marker[2]);
        break;
      case 3:
        marker[3].header.frame_id = "odom";
        marker[3].header.stamp = this->now();
        marker[3].ns = "rtls_anc2_circle";
        marker[3].id = 0;
        marker[3].type = visualization_msgs::msg::Marker::CYLINDER;
        marker[3].action =  visualization_msgs::msg::Marker::ADD;
        marker[3].pose.position.x = (double)0.0;
        marker[3].pose.position.y = (double)(-RTLS_SET_DIST * 0.001);
        marker[3].pose.position.z = 0.0;
        marker[3].scale.x = (double)(filter_dist[1] * 0.002);
        marker[3].scale.y = (double)(filter_dist[1] * 0.002);
        marker[3].scale.z = 0.001;
        marker[3].color.a = 0.5;
        if(filter_dist[1] < 650.0)
        {
          marker[3].color.r = 1.0;
          marker[3].color.g = 1.0;
          marker[3].color.b = 1.0;
        }
        else
        {
          marker[3].color.r = 0.0;
          marker[3].color.g = 1.0;
          marker[3].color.b = 1.0;
        }

        rtls_marker_publisher_->publish(marker[3]);
        break;
      case 4:
        marker[4].header.frame_id = "odom";
        marker[4].header.stamp = this->now();
        marker[4].ns = "rtls_tag_detect";
        marker[4].id = 0;
        marker[4].type = visualization_msgs::msg::Marker::SPHERE;
        marker[4].action =  visualization_msgs::msg::Marker::ADD;
        marker[4].pose.position.x = (double)(UWB_TargetPos.x * 0.001);
        marker[4].pose.position.y = (double)(UWB_TargetPos.y * 0.001);
        marker[4].pose.position.z = 0.0;
        marker[4].scale.x = 0.08;
        marker[4].scale.y = 0.08;
        marker[4].scale.z = 0.08;
        marker[4].color.a = 1.0;
        marker[4].color.r = 0.0;
        marker[4].color.g = 0.0;
        marker[4].color.b = 1.0;

        rtls_marker_publisher_->publish(marker[4]);
        break;
      default:
        break;
    }

    // marker[0].header.frame_id = "odom";
    // marker[0].header.stamp = this->now();
    // marker[0].ns = "rtls_anc1_detect";
    // marker[0].id = 0;
    // marker[0].type = visualization_msgs::msg::Marker::SPHERE;
    // marker[0].action =  visualization_msgs::msg::Marker::ADD;
    // marker[0].pose.position.x = (double)0.0;
    // marker[0].pose.position.y = (double)(RTLS_SET_DIST * 0.001);
    // marker[0].pose.position.z = 0.0;
    // marker[0].scale.x = 0.08;
    // marker[0].scale.y = 0.08;
    // marker[0].scale.z = 0.08;
    // marker[0].color.a = 1.0;
    // marker[0].color.r = 0.0;
    // marker[0].color.g = 1.0;
    // marker[0].color.b = 0.0;

    // marker[1].header.frame_id = "odom";
    // marker[1].header.stamp = this->now();
    // marker[1].ns = "rtls_anc1_circle";
    // marker[1].id = 0;
    // marker[1].type = visualization_msgs::msg::Marker::CYLINDER;
    // marker[1].action =  visualization_msgs::msg::Marker::ADD;
    // marker[1].pose.position.x = (double)0.0;
    // marker[1].pose.position.y = (double)(RTLS_SET_DIST * 0.001);
    // marker[1].pose.position.z = 0.0;
    // marker[1].scale.x = (double)(filter_dist[0] * 0.002);
    // marker[1].scale.y = (double)(filter_dist[0] * 0.002);
    // marker[1].scale.z = 0.001;
    // marker[1].color.a = 0.5;
    // marker[1].color.r = 0.0;
    // marker[1].color.g = 1.0;
    // marker[1].color.b = 1.0;

    // marker[2].header.frame_id = "odom";
    // marker[2].header.stamp = this->now();
    // marker[2].ns = "rtls_anc2_detect";
    // marker[2].id = 0;
    // marker[2].type = visualization_msgs::msg::Marker::SPHERE;
    // marker[2].action =  visualization_msgs::msg::Marker::ADD;
    // marker[2].pose.position.x = (double)0.0;
    // marker[2].pose.position.y = (double)(-RTLS_SET_DIST * 0.001);
    // marker[2].pose.position.z = 0.0;
    // marker[2].scale.x = 0.08;
    // marker[2].scale.y = 0.08;
    // marker[2].scale.z = 0.08;
    // marker[2].color.a = 1.0;
    // marker[2].color.r = 0.0;
    // marker[2].color.g = 1.0;
    // marker[2].color.b = 0.0;

    // marker[3].header.frame_id = "odom";
    // marker[3].header.stamp = this->now();
    // marker[3].ns = "rtls_anc2_circle";
    // marker[3].id = 0;
    // marker[3].type = visualization_msgs::msg::Marker::CYLINDER;
    // marker[3].action =  visualization_msgs::msg::Marker::ADD;
    // marker[3].pose.position.x = (double)0.0;
    // marker[3].pose.position.y = (double)(-RTLS_SET_DIST * 0.001);
    // marker[3].pose.position.z = 0.0;
    // marker[3].scale.x = (double)(filter_dist[1] * 0.002);
    // marker[3].scale.y = (double)(filter_dist[1] * 0.002);
    // marker[3].scale.z = 0.001;
    // marker[3].color.a = 0.5;
    // marker[3].color.r = 1.0;
    // marker[3].color.g = 1.0;
    // marker[3].color.b = 0.0;

    // marker[4].header.frame_id = "odom";
    // marker[4].header.stamp = this->now();
    // marker[4].ns = "rtls_tag_detect";
    // marker[4].id = 0;
    // marker[4].type = visualization_msgs::msg::Marker::SPHERE;
    // marker[4].action =  visualization_msgs::msg::Marker::ADD;
    // marker[4].pose.position.x = (double)(UWB_TargetPos.x * 0.001);
    // marker[4].pose.position.y = (double)(UWB_TargetPos.y * 0.001);
    // marker[4].pose.position.z = 0.0;
    // marker[4].scale.x = 0.08;
    // marker[4].scale.y = 0.08;
    // marker[4].scale.z = 0.08;
    // marker[4].color.a = 1.0;
    // marker[4].color.r = 0.0;
    // marker[4].color.g = 0.0;
    // marker[4].color.b = 1.0;

    rtls_msg.tag1filtdist = filter_dist[0];
    rtls_msg.tag2filtdist = filter_dist[1];

    rtls_distance_data_publisher_->publish(rtls_msg);
    // rtls_marker_publisher_->publish(marker[0]);
    // rtls_marker_publisher_->publish(marker[1]);
    // rtls_marker_publisher_->publish(marker[2]);
    // rtls_marker_publisher_->publish(marker[3]);
    // rtls_marker_publisher_->publish(marker[4]);

    marker_number++;
    if(marker_number > 4)
    {
      marker_number = 0;
    }

    RCLCPP_INFO(this->get_logger(), "%X %X , %8.2f , %X %X , %8.2f",
    dwm1000_readData[selector[0]].shtAddr_dev[0],dwm1000_readData[selector[0]].shtAddr_dev[1], filter_dist[0],
    dwm1000_readData[selector[1]].shtAddr_dev[0],dwm1000_readData[selector[1]].shtAddr_dev[1], filter_dist[1]);

  }
  // private 변수
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rtls_marker_publisher_;
  rclcpp::Publisher<message_interface::msg::Uwbdist>::SharedPtr rtls_distance_data_publisher_;
};

void rtls_settings_func()
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

void rtls_end_func()
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
  rtls_settings_func();

  // 클래스 생성
  auto node = std::make_shared<PacketRTLSSystem>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();
  rtls_end_func();

  return 0;
}
