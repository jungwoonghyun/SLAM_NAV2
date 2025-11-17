#include "esp32_init.hpp"

// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"
#include "message_interface/msg/buttonstate.hpp"
#include "message_interface/msg/ledcomm.hpp"

// bind 함수의 대체자 역할을 위해 _1로 선언
using std::placeholders::_1;

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

static char * esp32Uart = "/dev/ttyLED";
int fd;
uint8_t led_comm[16];
uint8_t firmware_comm = 1;
uint8_t transmit_comm = 1;
int subscriber_counter = 0;

uint8_t g_read_buf_bef[16] = {0, };
int g_error_cnt = 0;
int g_rw_error[2] = {0, };

class PacketESP32RobotLight : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출, 노드 이름을 packet_RS485_publisher 지정, count_는 0으로 초기화
  PacketESP32RobotLight()
  : Node("esp32_robot_light")
  {
    //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    packet_ESP32_Subscriber_ = this->create_subscription<message_interface::msg::Ledcomm>(
      "beacon_light_instructor",
      qos_profile,
      std::bind(&PacketESP32RobotLight::state_message, this, _1));

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PacketESP32RobotLight::esp32_msg, this));
  }
private:

  void state_message(const message_interface::msg::Ledcomm & msg) const
  {
    for(int k = 0; k < 16; k++)
    {
      if(msg.led[k] >= 1)
      {
        led_comm[k] = 1;
      }
      else
      {
        led_comm[k] = 0;
      }

      std::cout << k << " " << msg.led[k] << std::endl;
    }

    // firmware_comm = (uint8_t)msg.firmwarecommand;
    // transmit_comm = (uint8_t)msg.transmitcommand;
    firmware_comm = 1;
    transmit_comm = 1;
    subscriber_counter++;
  }

  void esp32_msg()
  {
    message_interface::msg::Buttonstate buttonData;

    uint8_t read_buf[16] = {0, };
    int packet_dial = -1;
    uint8_t packet_buf[16] = {0, };

    uint16_t read_sum = 0;
    uint16_t read_checkSum = 0;

    uint8_t error_code = 0;

    uint8_t cal_data = 0;

    uint8_t write_buf[8] = {0, };
    int led_control = 127;

    uint16_t sum = 0;
    uint16_t complement = 0;
    uint16_t checksum = 0;

    g_rw_error[0] = read(fd, read_buf, 16);
    g_rw_error[1] = 1;

    if(g_rw_error[0] < 0)
    {
      close(fd);
      openEsp32Device(esp32Uart, &fd);
    }

    // Packet dial setting (for reordering)
    for(int i = 0;i < 16;i++)
    {
      if(read_buf[i] == 0xEA)
      {
        if(i+1 > 15) //i == 15
        {
          if(read_buf[i-15] == 0xAD)
          {
            if(packet_dial == -1)
              packet_dial = i;
          }
        }
        else //i == 0~14
        {
          if(read_buf[i+1] == 0xAD)
          {
            if(packet_dial == -1)
              packet_dial = i;
          }
        }
      }
    }

    // Packet reordering
    for(int j = 0;j < 16; j++)
    {
      if(packet_dial != -1)
      {
        packet_buf[j] = read_buf[packet_dial];
        packet_dial++;
        if(packet_dial > 15)
        {
          packet_dial = 0;
        }
      }
      else
      {
        packet_buf[j] = read_buf[j];
      }
    }
    read_sum = (uint16_t)packet_buf[2] + (uint16_t)packet_buf[3] + (uint16_t)packet_buf[4]
            + (uint16_t)packet_buf[5] + (uint16_t)packet_buf[6] + (uint16_t)packet_buf[7]
            + (uint16_t)packet_buf[8] + (uint16_t)packet_buf[9] + (uint16_t)packet_buf[10]
            + (uint16_t)packet_buf[11] + (uint16_t)packet_buf[12] + (uint16_t)packet_buf[13];

    read_checkSum = (read_sum + (uint16_t)packet_buf[15]) % 0x100;

    // checksum error
    if(read_checkSum != 0)
    {
      error_code += 1;
    }

    // marker error or read timeout
    if(packet_dial == -1)
    {
      error_code += 2;
    }

    if(error_code > 0) // error state : read before packet
    {
      for(int l=0;l < 8;l++)
      {
        packet_buf[l] = g_read_buf_bef[l];
      }
      // printf("Read packet error, error state : %d / read_checkSum : %d\r\n",
      //        error_code, read_checkSum);

      printf("Read packet error : %2x %2x | %2x %2x %2x %2x | %2x %2x | %2x %2x %2x | %2x | %2x %2x | %2x %2x\r\n",
            (uint16_t)packet_buf[0], (uint16_t)packet_buf[1], (uint16_t)packet_buf[2],
            (uint16_t)packet_buf[3], (uint16_t)packet_buf[4], (uint16_t)packet_buf[5],
            (uint16_t)packet_buf[6], (uint16_t)packet_buf[7], (uint16_t)packet_buf[8],
            (uint16_t)packet_buf[9], (uint16_t)packet_buf[10], (uint16_t)packet_buf[11],
            (uint16_t)packet_buf[12], (uint16_t)packet_buf[13], (uint16_t)packet_buf[14],
            (uint16_t)packet_buf[15]);

      g_error_cnt++;
      if(g_error_cnt > 10)
      {
        // close(fd);
        // openEsp32Device(esp32Uart, &fd);
        g_error_cnt = 0;
      }
    }
    else
    {
      for(int m=0;m < 8;m++)
      {
        g_read_buf_bef[m] = packet_buf[m];
      }

      // RCLCPP_INFO(this->get_logger(), "Read packet success : %2x %2x | %2x %2x %2x %2x | %2x %2x | %2x %2x %2x | %2x | %2x %2x | %2x %2x\r\n",
      //       (uint16_t)packet_buf[0], (uint16_t)packet_buf[1], (uint16_t)packet_buf[2],
      //       (uint16_t)packet_buf[3], (uint16_t)packet_buf[4], (uint16_t)packet_buf[5],
      //       (uint16_t)packet_buf[6], (uint16_t)packet_buf[7], (uint16_t)packet_buf[8],
      //       (uint16_t)packet_buf[9], (uint16_t)packet_buf[10], (uint16_t)packet_buf[11],
      //       (uint16_t)packet_buf[12], (uint16_t)packet_buf[13], (uint16_t)packet_buf[14],
      //       (uint16_t)packet_buf[15]);

      printf("Read packet success : %2x %2x | %2x %2x %2x %2x | %2x %2x | %2x %2x %2x | %2x | %2x %2x | %2x %2x\r\n",
            (uint16_t)packet_buf[0], (uint16_t)packet_buf[1], (uint16_t)packet_buf[2],
            (uint16_t)packet_buf[3], (uint16_t)packet_buf[4], (uint16_t)packet_buf[5],
            (uint16_t)packet_buf[6], (uint16_t)packet_buf[7], (uint16_t)packet_buf[8],
            (uint16_t)packet_buf[9], (uint16_t)packet_buf[10], (uint16_t)packet_buf[11],
            (uint16_t)packet_buf[12], (uint16_t)packet_buf[13], (uint16_t)packet_buf[14],
            (uint16_t)packet_buf[15]);
    }

    // write data (led, comm. state)
    write_buf[0] = 0xAD; // mark 1
    write_buf[1] = 0xEA; // mark 2
    write_buf[2] = 0x01; // header
    write_buf[3] = 0x00;
    write_buf[4] = (uint8_t)(!led_comm[7]) * 128 + (uint8_t)(!led_comm[6]) * 64 + (uint8_t)(!led_comm[5]) * 32 + (uint8_t)(!led_comm[4]) * 16
                + (uint8_t)(!led_comm[3]) * 8 + (uint8_t)(!led_comm[2]) * 4 + (uint8_t)(!led_comm[1]) * 2 + (uint8_t)(!led_comm[0]) * 1; // LED State

    if(firmware_comm == 0) // communication off
    {
      write_buf[5] = 0x44;
      write_buf[6] = 0x32;
    }
    else if(firmware_comm == 1) // communication on
    {
      write_buf[5] = 0x13;
      write_buf[6] = 0x24;
    }
    else // other
    {
      write_buf[5] = 0;
      write_buf[6] = 0;
    }

    sum = (uint16_t)write_buf[2] + (uint16_t)write_buf[3] + (uint16_t)write_buf[4] + (uint16_t)write_buf[5] + (uint16_t)write_buf[6];
    complement = ~sum;
    checksum = (complement + 1) % 256;
    write_buf[7] = (uint8_t)checksum; // checksum

    if(transmit_comm == 1)
    {
      g_rw_error[0] = write(fd, write_buf, 8); //Esp32WriteData(&fd, write_buf);
      if(g_rw_error[0] < 0)
      {
        close(fd);
        openEsp32Device(esp32Uart, &fd);
      }

      // printf("Write packet : %X %X | %X | %X %X | %X %X | %X\r\n",
      //        write_buf[0], write_buf[1], write_buf[2], write_buf[3],
      //        write_buf[4], write_buf[5], write_buf[6], write_buf[7]);
    }

  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<message_interface::msg::Buttonstate>::SharedPtr packet_ESP32_Publisher_;
  rclcpp::Subscription<message_interface::msg::Ledcomm>::SharedPtr packet_ESP32_Subscriber_;
};

int main(int argc, char * argv[])
{
  // rclcpp 초기화
  rclcpp::init(argc, argv);

  openEsp32Device(esp32Uart, &fd);

  // poll_events.fd = fd;
  // poll_events.events = POLLIN | POLLERR;
  // poll_events.revents = 0;

  // 클래스 생성
  auto node = std::make_shared<PacketESP32RobotLight>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();

  close(fd);

  return 0;
}
