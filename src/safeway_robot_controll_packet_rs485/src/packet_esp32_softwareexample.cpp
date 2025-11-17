// Node 클래스를 사용하기 위한 rclcpp 헤더 파일
#include "rclcpp/rclcpp.hpp"
#include "message_interface/msg/buttonstate.hpp"
#include "message_interface/msg/ledcomm.hpp"

// bind 함수의 대체자 역할을 위해 _1로 선언
using std::placeholders::_1;

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;
uint32_t button_state[16];
int lednum = 0;
int cnt = 0;

class PacketESP32SoftwareExample : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출, 노드 이름을 packet_RS485_publisher 지정, count_는 0으로 초기화
  PacketESP32SoftwareExample()
  : Node("ESP32_softwareexample")
  {
    //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    packet_ESP32_SoftSub_ = this->create_subscription<message_interface::msg::Buttonstate>(
      "button_status",
      qos_profile,
      std::bind(&PacketESP32SoftwareExample::button_reading, this, _1));

    // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정
    packet_ESP32_SoftPub_ = this->create_publisher<message_interface::msg::Ledcomm>(
      "led_instructor", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PacketESP32SoftwareExample::led_command_example, this));
  }

private:
  void button_reading(const message_interface::msg::Buttonstate & msg) const //subscribe
  {
    // save button data
    for(int k = 0; k < 16; k++)
    {
      if(msg.button[k] >= 1)
      {
        button_state[k] = 1;
      }
      else
      {
        button_state[k] = 0;
      }
    }

    // print button data
    printf("Button read : %d, %d, %d ,%d, %d, %d, %d, %d | %d, %d, %d, %d, %d, %d, %d, %d\r\n",
           button_state[15], button_state[14], button_state[13], button_state[12],
           button_state[11], button_state[10], button_state[9], button_state[8],
           button_state[7], button_state[6], button_state[5], button_state[4],
           button_state[3], button_state[2], button_state[1], button_state[0]);
  }

  void led_command_example() //publish
  {
    message_interface::msg::Ledcomm ledcomm;

    // Control 3 Led : 3 push button state
    int cal_num = 0;

    ledcomm.led[2] = button_state[6];

    ledcomm.led[1] = button_state[5];

    ledcomm.led[0] = button_state[4];

    // firmware command (esp32 write control) : 1 = firmware communication on, 0 = firmware communication off
    ledcomm.firmwarecommand = 1;
    // ros2 driver transmit control : 1 = communication on, 0 = communication off
    ledcomm.transmitcommand = 1;
    packet_ESP32_SoftPub_->publish(ledcomm);

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<message_interface::msg::Buttonstate>::SharedPtr packet_ESP32_SoftSub_;
  rclcpp::Publisher<message_interface::msg::Ledcomm>::SharedPtr packet_ESP32_SoftPub_;
};

int main(int argc, char * argv[])
{
  // rclcpp 초기화
  rclcpp::init(argc, argv);

  // 클래스 생성
  auto node = std::make_shared<PacketESP32SoftwareExample>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();

  return 0;
}
