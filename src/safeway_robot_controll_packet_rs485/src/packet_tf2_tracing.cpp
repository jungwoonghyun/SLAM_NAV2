#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"

// 추후 500ms, 1s 와 같이 시간을 가시성 있게 문자로 표현하기 위한 namespace
using namespace std::chrono_literals;

geometry_msgs::msg::TransformStamped t_bef;
int delay_count = 0;

// rclcpp의 Node 클래스를 상속하여 사용
class PacketTf2Tracing : public rclcpp::Node
{
public:
  // Node 클래스의 생성자를 호출, 노드 이름을 packet_tf2_tracing 지정, count_는 0으로 초기화
  PacketTf2Tracing() : Node("packet_tf2_tracing")
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

    // Target frame 을 찾아내기 위한 파라미터를 설정
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 marked_vel, QoS
    packet_tf2_tracing_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", qos_profile);

    // 콜백 함수를 수행, 지정한 시간마다 지정한 콜백함수를 실행
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PacketTf2Tracing::tf2_data_msg, this));
  }
private:
  void tf2_data_msg()
  {
    // 읽고자 하는 frame (A 에 대한 B) 에 대한 설정과 그 frame 데이터를 저장할 transform 변수 설정
    std::string fromFrameRel = "odom";  //target_frame_.c_str();
    std::string toFrameRel = "base_footprint";
    geometry_msgs::msg::TransformStamped t;

    // transform 읽어들이기
    // t = tf_buffer_->lookupTransform(
    //     toFrameRel, fromFrameRel,
    //     tf2::TimePointZero);
    t = tf_buffer_->lookupTransform(
        "odom", "base_footprint",
        tf2::TimePointZero);

    visualization_msgs::msg::Marker marker;
//    marker = visualization_msgs::msg::Marker();
    marker.header.stamp.sec = 0;
    marker.header.stamp.nanosec = 0;
    marker.header.frame_id="odom";
    marker.ns = "while_line";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = t_bef.transform.translation.x;
    marker.pose.position.y = t_bef.transform.translation.y;
    marker.pose.position.z = 0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 1.0;
    marker.color.r = 0.0;
//    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    // auto point1 = geometry_msgs::msg::Point();
    // point1.x = t_bef_1.transform.translation.x;
    // point1.y = t_bef_1.transform.translation.y;
    // point1.z = 0.0;
    // auto point2 = geometry_msgs::msg::Point();
    // point2.x = t_bef_2.transform.translation.x;
    // point2.y = t_bef_2.transform.translation.y;
    // point2.z = 0.0;
    // marker.points.push_back(point1);
    // marker.points.push_back(point2);
    packet_tf2_tracing_->publish(marker);

    delay_count++;
    if(delay_count > 10)
    {
      t_bef.transform.translation.x = t.transform.translation.x;
      t_bef.transform.translation.y = t.transform.translation.y;
      delay_count = 0;
    }

    RCLCPP_INFO(this->get_logger(), "%7.3f, %7.3f", t.transform.translation.x, t.transform.translation.y);

  }
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr packet_tf2_tracing_;
  std::string target_frame_;

};

int main(int argc, char * argv[])
{
  // rclcpp 초기화
  rclcpp::init(argc, argv);

  // 클래스 생성
  auto node = std::make_shared<PacketTf2Tracing>();

  // 콜백 함수 실행
  rclcpp::spin(node);

  // ctrl+c와 같은 인터럽트 시그널 예외 상황에서 노드 종료
  rclcpp::shutdown();

  return 0;
}
