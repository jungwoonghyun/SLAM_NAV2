#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "message_interface/msg/odomdata.hpp"
#include "message_interface/msg/navcancel.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class NavigationCancelGoal : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigationCancelGoal() : Node("navigation_cancel_goal"),
  action_client_(rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose"))
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    point_data_ = this->create_subscription<message_interface::msg::Odomdata>("point_data", qos_profile, std::bind(&NavigationCancelGoal::point_callback, this, _1));
    nav_cancel_signal_ = this->create_subscription<message_interface::msg::Navcancel>("cancel_signal", qos_profile, std::bind(&NavigationCancelGoal::signal_callback, this, _1));

    timer_ = this->create_wall_timer(
        25ms, std::bind(&NavigationCancelGoal::on_timer, this));

  }

  void send_goal(double x_pos, double y_pos, double z_rot)
  {

    // if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Action server not available!");
    //   return;
    // }

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.00, 0.00, z_rot);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = x_pos;
    goal_msg.pose.pose.position.y = y_pos;
    goal_msg.pose.pose.orientation.x = quaternion.x();
    goal_msg.pose.pose.orientation.y = quaternion.y();
    goal_msg.pose.pose.orientation.z = quaternion.z();
    goal_msg.pose.pose.orientation.w = quaternion.w();

    RCLCPP_INFO(this->get_logger(), "goal init");
    action_client_->async_send_goal(goal_msg);
    RCLCPP_INFO(this->get_logger(), "Sending goal...");
    RCLCPP_INFO(this->get_logger(), "Goal accepted.");
  }

  void cancel_goal()
  {
    action_client_->async_cancel_all_goals();//async_cancel_goal(goal_handle_);
    RCLCPP_INFO(this->get_logger(), "Goal successfully cancelled.");
  }

private:

  void on_timer()
  {

  }

  void point_callback(const std::shared_ptr<message_interface::msg::Odomdata> msg)
  {
    double pose_x = 0.0;
    double pose_y = 0.0;
    double angle_z = 0.0;

    pose_x = (double)(msg->x_pos);
    pose_y = (double)(msg->y_pos);
    angle_z = (double)(msg->z_angle);

    send_goal(pose_x, pose_y, angle_z);
  }

  void signal_callback(const std::shared_ptr<message_interface::msg::Navcancel> msg)
  {
    if(msg->nav_cancel == true)
    {
      cancel_goal();
    }
    RCLCPP_INFO(this->get_logger(), "nav_cancel message == %d.", msg->nav_cancel);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<message_interface::msg::Odomdata>::SharedPtr point_data_;
  rclcpp::Subscription<message_interface::msg::Navcancel>::SharedPtr nav_cancel_signal_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  typename GoalHandle::SharedPtr goal_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<NavigationCancelGoal>());
  rclcpp::shutdown();
  return 0;
}
