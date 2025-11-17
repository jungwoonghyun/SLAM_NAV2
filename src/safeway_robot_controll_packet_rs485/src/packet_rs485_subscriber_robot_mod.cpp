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
#include "message_interface/msg/stepsub.hpp"

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



int mode_state = 1;
// remote controller state
// 0 : following mode off
// 1 : following mode on, following function On
// 2 : following mode on, following function Off
int before_mode = 0;

int alt_state = 0;

int follow_state = 0;
// PC state
// 0 : obj_detection off
// 1 : obj_detection on, following
// 2 : obj_detection on, but object lost
// 3 : play back
// 4 : replay

int determinant_val = 0;
// follow vs Play Back and Replay
// 0 : follow & other
// 1 : play back & replay

// 계산을 위한 변수 선언
double angular_, linear_;
double linear_bef = 0.0;
double spd_right, spd_left;
double scale_linear, scale_angular;

//
int64_t right_step_bef, left_step_bef;
int64_t right_step_curr, left_step_curr;

int imu_init = 0;
float imu_cali = 0.0;
float bef_imu_cali = 0.0;

int first_starting = 0;

int calib_buff[40];
int calib_setting = 0;
int calib_cnt = 0;

int usonic_cnt = 0;
int ulsonic_cur = 0;
int ulsonic_bef = 0;

// bind 함수의 대체자 역할을 위해 _1로 선언
using std::placeholders::_1;

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

// rclcpp의 Node클래스를 상속하여 사용
class PacketRS485SubscriberRobotMod : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출하고 노드 이름을 packet_RS485_subscriber 지정
  PacketRS485SubscriberRobotMod()
  : Node("packet_rs485_subscriber_robot_mod")
  {
    // Parameter를 불러오기 위한 선언 및 초기화(Rooty 모델 기준 초기화)
    this->declare_parameter("step_per_rotate", 120.0);
    this->declare_parameter("wheel_diameter", 0.254);
    this->declare_parameter("motion_circle_diameter", 0.47); // 0.66 / 0.455

    // QoS depth 10으로 하여 버퍼에 10개 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 구독할 토픽의 메시지 타입과 토픽의 이름, QoS 설정, 수신받은 메시지를 처리할 콜백함수를 기입
    packet_RS485_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      qos_profile,
      std::bind(&PacketRS485SubscriberRobotMod::subscribe_topic_message, this, _1));

    // 구독할 토픽의 메시지 타입과 토픽의 이름, QoS 설정, 수신받은 메시지를 처리할 콜백함수를 기입
    packet_state_subscriber_ = this->create_subscription<message_interface::msg::Mode>(
      "following_state",
      qos_profile,
      std::bind(&PacketRS485SubscriberRobotMod::subscribe_mode_message, this, _1));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정
    packet_Mode_Publisher_ = this->create_publisher<message_interface::msg::Mode>(
      "mode_selector_pub", qos_profile);

    packet_Ulsonic_Publisher_ = this->create_publisher<message_interface::msg::Ulsonicdist>(
      "ultrasonic_distance", qos_profile);

    packet_Enc_Publisher_ = this->create_publisher<message_interface::msg::Stepsub>(
      "motor_encoder_sub", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      25ms, std::bind(&PacketRS485SubscriberRobotMod::mode_msg, this));
  }

private:

  void mode_msg()
  {

    message_interface::msg::Mode c_mode;
    message_interface::msg::Ulsonicdist ultrasonic_distance_data;
    message_interface::msg::Stepsub step_sub;

    int nBytesCached = 0;
    int nBytesRead;

    // 읽은 message 데이터에 대한 RS485 명령 수행
    if(mode_state == 0) // remote controller : following mode off
    {
      // if(mode_state != before_mode)
      // {
      // }
      format_motion_command(hComm, "NOP");
      before_mode = mode_state;
    }
    else if(mode_state == 1) // remote controller : following mode on, following function on
    {
      if(follow_state == 2) // following mode : object lost
      {
        if(mode_state != before_mode)
        {
          format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=0;SPEED@2=0;UMODE=4;START=SS", (int)spd_right, (int)spd_left);
        }
        else
        {
          format_motion_command(hComm, "SPEED@1=0;SPEED@2=0;UMODE=4", (int)spd_right, (int)spd_left);
        }
        before_mode = mode_state;
      }
      else // following, playback, and replay
      {
        if(determinant_val == 1) // playback, and replay
        {
          if(mode_state != before_mode)
          {
            format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=%d,180,sps,180,sps;SPEED@2=%d,180,sps,180,sps;START=SS", (int)spd_right, (int)spd_left);
            alt_state = determinant_val;
          }
          else
          {
            if(alt_state == determinant_val)
            {
              format_motion_command(hComm, "SPEED@1=%d,180,sps,180,sps;SPEED@2=%d,180,sps,180,sps", (int)spd_right, (int)spd_left);
            }
            else
            {
              format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=%d,180,sps,180,sps;SPEED@2=%d,180,sps,180,sps;START=SS", (int)spd_right, (int)spd_left);
              alt_state = determinant_val;
            }
          }
        }
        else if (determinant_val == 2)
        {
          if(mode_state != before_mode)
          {
            format_motion_command(hComm, "COAST");
            alt_state = determinant_val;
          }
          else
          {
            if(alt_state == determinant_val)
            {
              format_motion_command(hComm, "NOP");
            }
            else
            {
              format_motion_command(hComm, "COAST");
              alt_state = determinant_val;
            }
          }
        }
        else if(determinant_val == 3) // joystick
        {
          if(mode_state != before_mode)
          {
            format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=%d,60,sps,180,sps;SPEED@2=%d,60,sps,180,sps;START=SS", (int)spd_right, (int)spd_left);
            alt_state = determinant_val;
          }
          else
          {
            if(alt_state == determinant_val)
            {
              format_motion_command(hComm, "SPEED@1=%d,60,sps,180,sps;SPEED@2=%d,60,sps,180,sps", (int)spd_right, (int)spd_left);
            }
            else
            {
              format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=%d,60,sps,180,sps;SPEED@2=%d,60,sps,180,sps;START=SS", (int)spd_right, (int)spd_left);
              alt_state = determinant_val;
            }
          }
        }
        else // following
        {
          if(mode_state != before_mode)
          {
            format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=%d,180,sps,180,sps;SPEED@2=%d,180,sps,180,sps;START=SS", (int)spd_right, (int)spd_left);
            alt_state = determinant_val;
          }
          else
          {
            if(alt_state == determinant_val)
            {
              format_motion_command(hComm, "SPEED@1=%d,180,sps,180,sps;SPEED@2=%d,180,sps,180,sps", (int)spd_right, (int)spd_left);
            }
            else
            {
              format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=%d,180,sps,180,sps;SPEED@2=%d,180,sps,180,sps;START=SS", (int)spd_right, (int)spd_left);
              alt_state = determinant_val;
            }
          }
        }

        before_mode = mode_state;
      }

    }
    else // remote controller : following mode on, following function off
    {
      if(mode_state != before_mode)
      {
        format_motion_command(hComm, "CONMODE=SPEED;SPEED@1=0;SPEED@2=0;START=SS");
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

    // read usonic data
    ultrasonic_distance_data.usonicnum = g_RobotInfo.nNumberofUsonic;
    ulsonic_cur = g_RobotInfo.nUsonicFrontValue;

    // if(ulsonic_cur == ulsonic_bef)
    // {
    //   usonic_cnt++;
    //   if(usonic_cnt > 15)
    //   {
    //     ulsonic_bef = ulsonic_cur;
    //     ulsonic_cur = 2000;
    //     usonic_cnt = 15;
    //   }
    //   else
    //   {
    //     ulsonic_bef = ulsonic_cur;
    //   }
    // }
    // else
    // {
    //   ulsonic_bef = ulsonic_cur;
    //   usonic_cnt = 0;
    // }

    ultrasonic_distance_data.frontdist = ulsonic_cur;
    ultrasonic_distance_data.backdist = g_RobotInfo.nUsonicBackValue;

// int usonic_cnt = 0;
// int ulsonic_bef = 0;
// ulsonic_cur


    // calculate step sub
    right_step_curr = g_RobotState[0].nPosition;
    left_step_curr = g_RobotState[1].nPosition;

    if(first_starting == 0)
    {
      step_sub.rightwheel_step = 0.0;
      step_sub.leftwheel_step = 0.0;
      first_starting = 1;
    }
    else
    {
      step_sub.rightwheel_step = ((float)(right_step_curr - right_step_bef) * (float)g_RobotInfo.nPolePair * 6.0) / (float)g_RobotInfo.dbSensorResolution;
      step_sub.leftwheel_step = ((float)(left_step_curr - left_step_bef) * (float)g_RobotInfo.nPolePair * 6.0) / (float)g_RobotInfo.dbSensorResolution;
    }

    right_step_bef = right_step_curr;
    left_step_bef = left_step_curr;

    step_sub.use_imu = false;

// int calib_buff[10];
// int calib_cnt = 0;

    if(calib_cnt < 40)
    {
      calib_buff[calib_cnt] = (g_RobotInfo.nUserModeValue2 / 100000);
      imu_cali = 0.0;
      calib_cnt++;
    }
    else if(calib_cnt == 40)
    {
      for(int i = 0; i < 40; i++)
      {
        calib_setting += calib_buff[i];
      }
      calib_setting = calib_setting / 40 + 0.0023;
      imu_cali = 0.0;
      calib_cnt++;
    }
    else
    {
      imu_init = (g_RobotInfo.nUserModeValue2 / 100000) - calib_setting;
      imu_cali = (float)imu_init / 100.0;
    }


    step_sub.imu_yaw = (imu_cali * 0.2 + bef_imu_cali * 0.8)* 1.15;
    bef_imu_cali = imu_cali;
    // imu_init = 1;

    // step_sub.imu_yaw = ((float)g_RobotInfo.nUserModeValue2 / 10000000.0) - imu_cali;

    // print data
    printf("Encoder Sub : %.2f, %.2f / Usonic : %d [%d, %d] / IMU.Z : %f / %d, %d\n",
    step_sub.rightwheel_step, step_sub.leftwheel_step,       // user mode value #2
    g_RobotInfo.nNumberofUsonic, g_RobotInfo.nUsonicFrontValue, g_RobotInfo.nUsonicBackValue,
    step_sub.imu_yaw, g_RobotInfo.nUserModeValue1, g_RobotInfo.nUserModeValue3);

    packet_Mode_Publisher_->publish(c_mode);
    packet_Ulsonic_Publisher_->publish(ultrasonic_distance_data);
    packet_Enc_Publisher_ ->publish(step_sub);
  }

  void subscribe_mode_message(const message_interface::msg::Mode::SharedPtr msg) const
  {
    follow_state = msg->modenum;
    determinant_val = msg->determinant;
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

    *****************************************/

    // 저장된 Parameter 데이터 로딩
    step_p_rot = this->get_parameter("step_per_rotate").get_value<double>();
    wheel_dia = this->get_parameter("wheel_diameter").get_value<double>();
    m_cir_dia = this->get_parameter("motion_circle_diameter").get_value<double>();

    // Publisher 로부터 읽어들인 데이터를 변수에 저장한 후 출력
    scale_linear = step_p_rot * (9.0/7.0); // (0.254 * M_PI); // 1m/s scale : 120 step/s = 1 rotate/s = 0.7777778(7/9) m/s
    scale_angular = scale_linear * (m_cir_dia / 2.0); //
    angular_ = msg->angular.z; // rad/s
    linear_ = msg->linear.x; // m/s
  //  if(linear_bef < 0.001 || linear_bef > -0.001 )

 //   linear_bef = linear_;
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
  rclcpp::Publisher<message_interface::msg::Stepsub>::SharedPtr packet_Enc_Publisher_;
};

int main(int argc, char * argv[])
{
  hComm = open_comm_dev(pszDevName, nBaud);
  start_robot(hComm);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PacketRS485SubscriberRobotMod>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  end_robot(hComm);
  close_comm_dev(hComm);

  return 0;
}
