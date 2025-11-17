#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include "message_interface/msg/odomdata.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BasefootprintListner:public rclcpp::Node
{
  public:
    BasefootprintListner()
    : Node("basefootprint_listner") {
      tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

      bfootprint_data_ = this->create_publisher<message_interface::msg::Odomdata>(
      "foot_data", qos_profile);

          // Call on_timer function every second
      timer_ = this->create_wall_timer(
        25ms, std::bind(&BasefootprintListner::on_timer, this));
    }
  private:
    void on_timer()
    {
      std::string fromFrameRel = "base_footprint";
      std::string toFrameRel = "map";

      geometry_msgs::msg::TransformStamped t;
      message_interface::msg::Odomdata data_calc;

      try {
        t = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel,
          tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
      }

      data_calc.x_pos = t.transform.translation.x;
      data_calc.y_pos = t.transform.translation.y;

      data_calc.z_angle = -(atan2(2.0 * (t.transform.rotation.z * t.transform.rotation.w + t.transform.rotation.x * t.transform.rotation.y),
                                -1.0 * 2.0 * (t.transform.rotation.w * t.transform.rotation.w + t.transform.rotation.x * t.transform.rotation.x))
                                + M_PI);

      if(data_calc.z_angle < -M_PI)
      {
        data_calc.z_angle = data_calc.z_angle + 2.0 * M_PI;
      }

      if(data_calc.z_angle > M_PI)
      {
        data_calc.z_angle = data_calc.z_angle - 2.0 * M_PI;
      }

      data_calc.z_angle = data_calc.z_angle * 2.0;

      bfootprint_data_->publish(data_calc);
    }

    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    rclcpp::Publisher<message_interface::msg::Odomdata>::SharedPtr bfootprint_data_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasefootprintListner>());
  rclcpp::shutdown();
  return 0;
}
