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

#include "rs485_settings.hpp"

// keyboard function 을 받아오기 위한 헤더파일
#include "keyboard_setting_function.hpp"

// Odometry 좌표상의 로봇 기구학 계산을 보다 쉽게 하기 위하여,
// ROS2 에서 기본적으로 제공하는 geometry_msgs/msg/twist 를 사용하기 위한 헤더 파일
#include "geometry_msgs/msg/twist.hpp"

// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"

#include "message_interface/msg/mode.hpp"
#include "message_interface/msg/ulsonicdist.hpp"

// bind 함수의 대체자 역할을 위해 _1로 선언
using std::placeholders::_1;
    char ch;
// Math.h 의 변수 사용을 위한 매크로
#define _USE_MATH_DEFINES

// RS485 통신을 위한 데이터 - 시작 -
#define MOTION_CMD_BUFFER_SIZE  256
#define MOTION_CMD_MAX_LEN      (MOTION_CMD_BUFFER_SIZE - 10)

// buffer for receciving the reply
static char s_szCmdRBuffer[MOTION_CMD_BUFFER_SIZE];

COMM_HANDLE hComm;

static char *pszDevName = "/dev/ttySerial";
static const int nBaud = 2000000;
// RS485 통신을 위한 데이터 - 종료 -

// Parameter 데이터를 위한 변수
double step_p_rot;
double wheel_dia;
double m_cir_dia;

int mode_state = 0; // 0 : non-following mode / 1 : Mode on, following On / 2 : Mode on, following Off
int before_mode = 0;

// 계산을 위한 변수 선언
double angular_, linear_;
double spd_right, spd_left;
double scale_linear, scale_angular;

int follow_state = 0;
// 0 : following off / 1 : following on / 2 : object lost

// bind 함수의 대체자 역할을 위해 _1로 선언
using std::placeholders::_1;

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

// rclcpp의 Node클래스를 상속하여 사용
class PacketRS485SubscriberRobot : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출하고 노드 이름을 packet_RS485_subscriber 지정
  PacketRS485SubscriberRobot()
  : Node("packet_RS485_subscriber_robot")
  {
    // Parameter를 불러오기 위한 선언 및 초기화(Rooty 모델 기준 초기화)
    this->declare_parameter("step_per_rotate", 120.0);
    this->declare_parameter("wheel_diameter", 0.254);
    this->declare_parameter("motion_circle_diameter", 0.455); // 0.66 / 0.455

    // QoS depth 10으로 하여 버퍼에 10개 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 구독할 토픽의 메시지 타입과 토픽의 이름, QoS 설정, 수신받은 메시지를 처리할 콜백함수를 기입
    packet_RS485_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      qos_profile,
      std::bind(&PacketRS485SubscriberRobot::subscribe_topic_message, this, _1));

    // 구독할 토픽의 메시지 타입과 토픽의 이름, QoS 설정, 수신받은 메시지를 처리할 콜백함수를 기입
    packet_state_subscriber_ = this->create_subscription<message_interface::msg::Mode>(
      "following_state",
      qos_profile,
      std::bind(&PacketRS485SubscriberRobot::subscribe_mode_message, this, _1));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정
    packet_Mode_Publisher_ = this->create_publisher<message_interface::msg::Mode>(
      "mode_selector_pub", qos_profile);

    packet_Ulsonic_Publisher_ = this->create_publisher<message_interface::msg::Ulsonicdist>(
      "ultrasonic_distance", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      25ms, std::bind(&PacketRS485SubscriberRobot::mode_msg, this));
  }

private:

  void mode_msg()
  {

    message_interface::msg::Mode c_mode;
    message_interface::msg::Ulsonicdist ultrasonic_distance_data;

    int nBytesCached = 0;
    int nBytesRead;

    // 읽은 message 데이터에 대한 RS485 명령 수행
    if(mode_state == 0)
    {
      if(mode_state != before_mode)
      {
        format_motion_command(hComm, "SPEED@1=0;SPEED@2=0;START=SS");
      }
      else
      {
        format_motion_command(hComm, "NOP");
      }
      before_mode = mode_state;
    }
    else if(mode_state == 1)
    {
      if(follow_state == 2) //object lost
      {
        if(mode_state != before_mode)
        {
          format_motion_command(hComm, "SPEED@1=0;SPEED@2=0;UMODE=4;START=SS", (int)spd_right, (int)spd_left);
        }
        else
        {
          format_motion_command(hComm, "SPEED@1=0;SPEED@2=0;UMODE=4", (int)spd_right, (int)spd_left);
        }
        before_mode = mode_state;
      }
      else
      {
        if(mode_state != before_mode)
        {
          format_motion_command(hComm, "SPEED@1=%d;SPEED@2=%d;START=SS", (int)spd_right, (int)spd_left);
        }
        else
        {
          format_motion_command(hComm, "SPEED@1=%d;SPEED@2=%d", (int)spd_right, (int)spd_left);
        }
        before_mode = mode_state;
      }

    }
    else
    {
      if(mode_state != before_mode)
      {
        format_motion_command(hComm, "SPEED@1=0;SPEED@2=0;START=SS");
      }
      else
      {
        format_motion_command(hComm, "SPEED@1=0;SPEED@2=0");
      }
      before_mode = mode_state;
    }
    msleep(1);

    nBytesRead = recv_comm_dev(hComm, s_szCmdRBuffer + nBytesCached, sizeof(s_szCmdRBuffer));

    if (nBytesRead < 0)
    {
      // read error
      printf("\nRead error\n");
      nBytesCached = 0;
    }
    else if (nBytesRead > 0)
    {
      // parse the received packet
      int nResult = parse_recv_packet(s_szCmdRBuffer, nBytesRead + nBytesCached);
//      dump_recv_packet(s_szCmdRBuffer, nBytesRead);

      // need more data?
      if (nResult > 0)
      {
        // need more data
        nBytesCached += nBytesRead;

//        printf("need more data: %d\n", nResult);
        msleep(1);
      }
      else
      {
        // fail to parse?
        if (nResult < 0)
        {
            // fail to parse the received packet => dump
            dump_recv_packet(s_szCmdRBuffer, nBytesRead);
        }
#if 1
        else
        {
            printf("%d bytes parsed successfully\n", nBytesRead);
        }
#endif

        nBytesCached = 0;
      }
    }
    printf("User Mode Data: %d, %d, %d\n",
    g_RobotInfo.nUserModeValue1, g_RobotInfo.nUserModeValue2, g_RobotInfo.nUserModeValue3);

    ultrasonic_distance_data.frontdist = g_RobotInfo.nUserModeValue2;
    ultrasonic_distance_data.backdist = g_RobotInfo.nUserModeValue3;

    if(g_RobotInfo.nUserModeValue1 == 10)
    {
      mode_state = 1;
      c_mode.modenum = 1;
    }
    else if(g_RobotInfo.nUserModeValue1 == 4)
    {
      mode_state = 1;
      c_mode.modenum = 1;
    }
    else if(g_RobotInfo.nUserModeValue1 == 8)
    {
      mode_state = 2;
      c_mode.modenum = 0;
    }
    else
    {
      mode_state = 0;
      c_mode.modenum = 0;
    }
    packet_Mode_Publisher_->publish(c_mode);
    packet_Ulsonic_Publisher_->publish(ultrasonic_distance_data);
  }

  void subscribe_mode_message(const message_interface::msg::Mode::SharedPtr msg) const
  {
    follow_state = msg->modenum;
  }

  // 콜백함수
  void subscribe_topic_message(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {

    /*****************************************

       |---m_cir_dia---|

      ******************* -
      * *             * * | wheel_dia
      * *             * * |
      ***             *** -
        *             *
        *             *
        *             *
        *             *
        *             *
        ***************

    1) about linear_x
    1 step_p_rot = 60 rpm = 1 rps
    1 rps = (Jinuxing Motor) wheel_dia * M_PI m/s

    2) about angular_z
    distance between Motors : m_cir_dia mm
    wheel speed (1 rps) : wheel_dia * pi mm/s
    if robot rotate : pi rad/s, wheel rotate : m_cir_dia / (2 * wheel_dia) rps


    linear speed(m/s) : step_p_rot(sps) * (1.0 / (wheel_dia * M_PI)) * linear_x
    angular speed(rad/s) : angular_z * m_cir_dia/(2 * wheel_dia * pi)

    *****************************************/

    // 저장된 Parameter 데이터 로딩
    step_p_rot = this->get_parameter("step_per_rotate").get_value<double>();
    wheel_dia = this->get_parameter("wheel_diameter").get_value<double>();
    m_cir_dia = this->get_parameter("motion_circle_diameter").get_value<double>();

    // Publisher 로부터 읽어들인 데이터를 변수에 저장한 후 출력
    scale_linear = step_p_rot * (1.0 / (wheel_dia * M_PI));
    scale_angular = scale_linear * (m_cir_dia / (wheel_dia * 2.0 * M_PI));
    angular_ = msg->angular.z; // rad/s
    linear_ = msg->linear.x; // m/s
    RCLCPP_INFO(this->get_logger(), "Subscribed message: '%.2f', '%.2f'", angular_, linear_);
    RCLCPP_INFO(this->get_logger(), "Parameter: '%.2f', '%.2f', '%.3f'", step_p_rot, wheel_dia, m_cir_dia);

    spd_right = scale_linear * linear_ + scale_angular * angular_; // M_PI;
    spd_left = scale_linear * linear_ - scale_angular * angular_; // M_PI;

  }

  // private 변수로 사용되는 packet_RS485_subscriber_ 선언
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr packet_RS485_subscriber_;
  rclcpp::Publisher<message_interface::msg::Mode>::SharedPtr packet_Mode_Publisher_;
  rclcpp::Subscription<message_interface::msg::Mode>::SharedPtr packet_state_subscriber_;
  rclcpp::Publisher<message_interface::msg::Ulsonicdist>::SharedPtr packet_Ulsonic_Publisher_;
};

int main(int argc, char * argv[])
{
  hComm = open_comm_dev(pszDevName, nBaud);
  start_robot(hComm);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PacketRS485SubscriberRobot>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  end_robot(hComm);
  close_comm_dev(hComm);

  return 0;
}
