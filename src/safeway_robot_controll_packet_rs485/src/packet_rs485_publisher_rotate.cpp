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

// keyboard function 을 받아오기 위한 헤더파일
#include "keyboard_setting_function.hpp"

// Odometry 좌표상의 로봇 기구학 계산을 보다 쉽게 하기 위하여,
// ROS2 에서 기본적으로 제공하는 geometry_msgs/msg/twist 를 사용하기 위한 헤더 파일
#include "geometry_msgs/msg/twist.hpp"
#include "message_interface/msg/odomdata.hpp"


// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"

// bind 함수의 대체자 역할을 위해 _1로 선언
using std::placeholders::_1;

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

double odom_x = 0.0, odom_y = 0.0, odom_angle = 0.0;

// rclcpp의 Node 클래스를 상속하여 사용
class PacketRS485PublisherRotate : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출, 노드 이름을 packet_RS485_publisher 지정, count_는 0으로 초기화
  PacketRS485PublisherRotate()
  : Node("packet_RS485_publisher_rotate")
  {
    //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 robot_cmd_vel, QoS
    packet_RS485_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", qos_profile);

    odometry_data_ = this->create_subscription<message_interface::msg::Odomdata>(
      "odom_data",
      qos_profile,
      std::bind(&PacketRS485PublisherRotate::subscribe_odom_message, this, _1));


    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PacketRS485PublisherRotate::packet_RS485_msg, this));
  }

private:
  // keyboard 입력에 따른 속력, 회전각도 조절 및 scale 변수
  double linear_ = 0.0, angular_ = 0.0;
  double target_angle = 0.0;
  int linear_dir = 1, angular_dir = 1;
  int dt = 0;
  int rotate_sw = 0;

  void subscribe_odom_message(const message_interface::msg::Odomdata::SharedPtr msg) const
  {
    odom_angle = (double)(msg->z_angle);
  }

  // 콜백 함수
  void packet_RS485_msg()
  {
    // Geometry_msgs::msg::Twist 타입으로 msg 선언
    geometry_msgs::msg::Twist twist;

    if(rotate_sw == 0)
    {
      target_angle = odom_angle + M_PI;
      if(target_angle > M_PI)
      {
        target_angle -= 2.0 * M_PI;
      }
      rotate_sw = 1;
    }
    else if(rotate_sw == 1)
    {
      if(target_angle > 0.0)
      {
        angular_ = (target_angle - odom_angle);
        if(target_angle - odom_angle < 0.1)
        {
          angular_ = 0.0;
          rotate_sw = 2;
        }
      }
      else
      {
        angular_ = (target_angle - odom_angle);
        if(target_angle - odom_angle > -0.1)
        {
          angular_ = 0.0;
          rotate_sw = 2;
        }
      }
    }
    else
    {
      dt += 1;
      if(dt >= 20)
      {
        dt = 0;
        rotate_sw = 0;
      }
    }

    // if(dt < 40)
    // {
    //   angular_ = 0.0;
    //   linear_ = 0.25;
    // }
    // else if(dt < 60)
    // {
    //   angular_ = 0.0;
    //   linear_ = 0.0;
    // }
    // else if(dt < 100)
    // {
    //   angular_ = 0.25 * M_PI;
    //   linear_ = 0.0;
    // }
    // else if(dt < 120)
    // {
    //   angular_ = 0.0;
    //   linear_ = 0.0;
    // }

    // dt += 1;

    // if(dt >= 120)
    // {
    //   dt = 0;
    // }

    // Twist message 에 계산한 결과데이터 입력
    twist.angular.z = angular_;
    twist.linear.x = linear_;

    // logging, RCLCPP_XXX 계열의 함수는 print와 비슷
    RCLCPP_INFO(this->get_logger(), "Published message: '%.2f', '%.2f' / key : %2d",
                      twist.angular.z, twist.linear.x, rotate_sw);

    // publishing
    packet_RS485_publisher_->publish(twist);

  }

  // private 변수
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr packet_RS485_publisher_;
  rclcpp::Subscription<message_interface::msg::Odomdata>::SharedPtr odometry_data_;
  //size_t count_;
};

int main(int argc, char * argv[])
{
  // rclcpp 초기화
  rclcpp::init(argc, argv);

  init_keyboard();

  // 클래스 생성
  auto node = std::make_shared<PacketRS485PublisherRotate>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();

  close_keyboard();

  return 0;
}
