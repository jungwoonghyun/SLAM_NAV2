#include "esp32_init.hpp"

// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"
#include "message_interface/msg/buttonstate.hpp"
#include "message_interface/msg/ledcomm.hpp"

// bind 함수의 대체자 역할을 위해 _1로 선언
using std::placeholders::_1;

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

static char * esp32Uart = "/dev/ttyIO";
int fd;
uint8_t led_comm[16];
uint8_t firmware_comm = 1;
uint8_t transmit_comm = 1;
int subscriber_counter = 0;

uint8_t read_buf_bef[8] = {0, };

// struct pollfd poll_events;
// int poll_state;

class PacketESP32Controller : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출, 노드 이름을 packet_RS485_publisher 지정, count_는 0으로 초기화
  PacketESP32Controller()
  : Node("ESP32_controller")
  {
    //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    packet_ESP32_Subscriber_ = this->create_subscription<message_interface::msg::Ledcomm>(
      "led",
      qos_profile,
      std::bind(&PacketESP32Controller::state_message, this, _1));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정
    packet_ESP32_Publisher_ = this->create_publisher<message_interface::msg::Buttonstate>(
      "button_status", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PacketESP32Controller::esp32_msg, this));
  }

private:

  void esp32_msg()
  {
    uint8_t read_buf[8];

    uint8_t packet_buf[8];
    int packet_dial = -1;

    uint8_t write_buf[8];

    uint16_t read_sum = 0;
    uint16_t read_checkSum = 0;

    uint8_t error_code = 0;

    uint16_t sum = 0;
    uint16_t complement = 0;
    uint16_t checksum = 0;

    uint8_t cal_data = 0;

    message_interface::msg::Buttonstate buttonData;

    Esp32ReadData(&fd, read_buf);

    // poll_state = poll((struct pollfd*)&poll_events, 1, 10);
    // if(poll_state > 0)
    // {
    //   if(poll_events.revents & POLLIN)
    //   {
    //     Esp32ReadData(&fd, read_buf);

    //   }
    //   if(poll_events.revents & POLLERR)
    //   {
    //     printf("Read Error\r\n");
    //   }
    // }
//    usleep(1000);

    // Packet dial setting (for reordering)
    for(int i = 0;i < 8;i++)
    {
      if(read_buf[i] == 0xAA)
      {
        if(i+1 > 7) //i == 7
        {
          if(read_buf[i-7] == 0xBB)
          {
            if(packet_dial == -1)
              packet_dial = i;
          }
        }
        else //i == 0~6
        {
          if(read_buf[i+1] == 0xBB)
          {
            if(packet_dial == -1)
              packet_dial = i;
          }
        }
      }
    }

    // Packet reordering
    for(int j = 0;j < 8; j++)
    {
      if(packet_dial != -1)
      {
        packet_buf[j] = read_buf[packet_dial];
        packet_dial++;
        if(packet_dial > 7)
        {
          packet_dial = 0;
        }
      }
      else
      {
        if(j == 7)
          packet_buf[j] = 5;
        else
          packet_buf[j] = 0;
      }
    }

    read_sum = (uint16_t)packet_buf[2] + (uint16_t)packet_buf[3] + (uint16_t)packet_buf[4]
             + (uint16_t)packet_buf[5] + (uint16_t)packet_buf[6];

    read_checkSum = (read_sum + (uint16_t)packet_buf[7]) % 0x100;

    // checksum error
    if(read_checkSum != 0)
    {
      error_code += 1;
    }

    // header error
    if(packet_buf[2] != 0x01)
    {
      error_code += 2;
    }

    // marker error or read timeout
    if(packet_dial == -1)
    {
      error_code += 4;
    }

    if(error_code > 0) // error state : read before packet
    {
      for(int l=0;l < 8;l++)
      {
        packet_buf[l] = read_buf_bef[l];
      }
      printf("Packet error, reuse before packet : %X %X | %X | %X %X | %X %X | %X, error : %d\r\n",
             packet_buf[0], packet_buf[1], packet_buf[2], packet_buf[3],
             packet_buf[4], packet_buf[5], packet_buf[6], packet_buf[7], error_code);
    }
    else
    {
      printf("Read packet : %X %X | %X | %X %X | %X %X | %X\r\n",
             packet_buf[0], packet_buf[1], packet_buf[2], packet_buf[3],
             packet_buf[4], packet_buf[5], packet_buf[6], packet_buf[7]);
    }

    // save button state data (Button 1))
    buttonData.button[15] = packet_buf[3] / 128;
    cal_data = packet_buf[3] % 128;

    buttonData.button[14] = cal_data / 64;
    cal_data = packet_buf[3] % 64;

    buttonData.button[13] = cal_data / 32;
    cal_data = packet_buf[3] % 32;

    buttonData.button[12] = cal_data / 16;
    cal_data = packet_buf[3] % 16;

    buttonData.button[11] = cal_data / 8;
    cal_data = packet_buf[3] % 8;

    buttonData.button[10] = cal_data / 4;
    cal_data = packet_buf[3] % 4;

    buttonData.button[9] = cal_data / 2;
    cal_data = packet_buf[3] % 2;

    buttonData.button[8] = cal_data / 1;

    // save button state data (Button 2))
    buttonData.button[7] = packet_buf[4] / 128;
    cal_data = packet_buf[4] % 128;

    buttonData.button[6] = cal_data / 64;
    cal_data = packet_buf[4] % 64;

    buttonData.button[5] = cal_data / 32;
    cal_data = packet_buf[4] % 32;

    buttonData.button[4] = cal_data / 16;
    cal_data = packet_buf[4] % 16;

    buttonData.button[3] = cal_data / 8;
    cal_data = packet_buf[4] % 8;

    buttonData.button[2] = cal_data / 4;
    cal_data = packet_buf[4] % 4;

    buttonData.button[1] = cal_data / 2;
    cal_data = packet_buf[4] % 2;

    buttonData.button[0] = cal_data / 1;

    // write data
    write_buf[0] = 0xBB; // mark 1
    write_buf[1] = 0xAA; // mark 2
    write_buf[2] = 0x01; // header

    write_buf[3] = (uint8_t)(!led_comm[15]) * 128 + (uint8_t)(!led_comm[14]) * 64 + (uint8_t)(!led_comm[13]) * 32 + (uint8_t)(!led_comm[12]) * 16
                  + (uint8_t)(!led_comm[11]) * 8 + (uint8_t)(!led_comm[10]) * 4 + (uint8_t)(!led_comm[9]) * 2 + (uint8_t)(!led_comm[8])* 1;
    // LED 1
    write_buf[4] = (uint8_t)(!led_comm[7]) * 128 + (uint8_t)(!led_comm[6]) * 64 + (uint8_t)(!led_comm[5]) * 32 + (uint8_t)(!led_comm[4]) * 16
                  + (uint8_t)(!led_comm[3]) * 8 + (uint8_t)(!led_comm[2]) * 4 + (uint8_t)(!led_comm[1]) * 2 + (uint8_t)(!led_comm[0]) * 1;
    // LED 2

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
    // firmware command

    sum = (uint16_t)write_buf[2] + (uint16_t)write_buf[3] + (uint16_t)write_buf[4] + (uint16_t)write_buf[5] + (uint16_t)write_buf[6];
    complement = ~sum;
    checksum = (complement + 1) % 256;
    write_buf[7] = (uint8_t)checksum; // checksum

    // RCLCPP_INFO(this->get_logger(), "Button State : %2d, %2d, %2d | LED Operation Command : %2d, %2d, %2d\r\n",
    //             button_state[0], button_state[1], button_state[2], led_state[0], led_state[1], led_state[2]);

    if(transmit_comm == 1)
    {
      Esp32WriteData(&fd, write_buf);
      printf("Write packet : %X %X | %X | %X %X | %X %X | %X\r\n",
             write_buf[0], write_buf[1], write_buf[2], write_buf[3],
             write_buf[4], write_buf[5], write_buf[6], write_buf[7]);
    }
    packet_ESP32_Publisher_->publish(buttonData);

    for(int m=0;m < 8;m++)
    {
      read_buf_bef[m] = packet_buf[m];
    }
  }

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
    }

    firmware_comm = (uint8_t)msg.firmwarecommand;
    transmit_comm = (uint8_t)msg.transmitcommand;
    subscriber_counter++;

    // RCLCPP_INFO(this->get_logger(), "Receive %2d, %2d, %2d mode num from PC(LED Operation Command)\r\n",
    //             led_state[0], led_state[1], led_state[2]);
  }

  // private 변수
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<message_interface::msg::Buttonstate>::SharedPtr packet_ESP32_Publisher_;
  rclcpp::Subscription<message_interface::msg::Ledcomm>::SharedPtr packet_ESP32_Subscriber_;
  //size_t count_;
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
  auto node = std::make_shared<PacketESP32Controller>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();

  close(fd);

  return 0;
}
