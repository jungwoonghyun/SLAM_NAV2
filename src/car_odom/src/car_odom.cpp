#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "message_interface/msg/odomdata.hpp"
#include "message_interface/msg/stepsub.hpp"

using std::placeholders::_1;

float right_w = 0.0, left_w = 0.0;
float imu_val = 0.0;
bool using_imu = false;

class CarOdom:public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_raw;
        rclcpp::Subscription<message_interface::msg::Stepsub>::SharedPtr step_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        double scale_x = 1.0;
        double scale_y = 1.0;
        double scale_z = 1.0;

        double delta_time = 0.0;
        double vel_x = 0.0;
        double vel_y = 0.0;
        double vel_z = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
        double delta_z = 0.0;
        double pos_x = 0.0;
        double pos_y = 0.0;
        double pos_z = 0.0;

        rclcpp::Time last_time;
        geometry_msgs::msg::TransformStamped tf;

    public:
        CarOdom() : Node("car_odom") {
            this->declare_parameter<double>("scale_x", 1.0);
            this->declare_parameter<double>("scale_y", 1.0);
            this->declare_parameter<double>("scale_z", 1.0);

            this->get_parameter<double>("scale_x", scale_x);
            this->get_parameter<double>("scale_y", scale_y);
            this->get_parameter<double>("scale_z", scale_z);

            tf.header.frame_id = "odom";
            tf.child_frame_id = "base_footprint";

            //  QoS 설정을 위해 KeepLast 형태로 depth를 10으로 설정하여 퍼블리시할 데이터를 버퍼에 10개까지 저장
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

            // node클래스의 create_publisher함수를 이용하여 퍼블리셔 설정, 메시지 타입으로 Twist, 토픽 이름으로 robot_cmd_vel, QoS
            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", qos_profile);

            odometry_data_ = this->create_publisher<message_interface::msg::Odomdata>(
            "odom_data", qos_profile);

            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            sub_vel_raw = this->create_subscription<geometry_msgs::msg::TwistStamped>("vel_raw", 10, std::bind(&CarOdom::vel_raw_callback, this, _1));
            step_sub_ = this->create_subscription<message_interface::msg::Stepsub>("motor_encoder_sub", 10, std::bind(&CarOdom::enc_imu_callback, this, _1));
        }

    private:
        void enc_imu_callback(const std::shared_ptr<message_interface::msg::Stepsub> msg) {
            right_w = (msg->rightwheel_step / 120.0) * (2.0 * M_PI);
            left_w = (msg->leftwheel_step / 120.0) * (2.0 * M_PI);
            using_imu = msg->use_imu;
            imu_val = msg->imu_yaw;
        }

        void vel_raw_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
            // delta time  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr packet_RS485_publisher_;
            nav_msgs::msg::Odometry odom_settings;
            message_interface::msg::Odomdata data_calc;

            // double delta_s = 0.0;
            // double delta_theta = 0.0;

            // double theta = 0.0;
            // static double last_theta = 0.0;

            // double v = 0.0;
            // double w = 0.0;

            rclcpp::Time curr_time = msg->header.stamp;
            if (last_time.seconds() == 0) {
                last_time = curr_time;
                return;
            }
            delta_time = (curr_time - last_time).seconds();
            last_time = curr_time;
            // recv value

            // if (std::isnan(left_w)) {
            //     left_w = 0.0;
            // }

            // if (std::isnan(right_w)) {
            //     right_w = 0.0;
            // }

            // delta_s = 0.127 * (double)(left_w + right_w) / 2.0;

            // if(using_imu)
            // {
            //     delta_theta = (double)imu_val * delta_time;
            //     theta += delta_theta;
            // }
            // else
            // {
            //     theta = 0.127 * (double)(left_w + right_w) / 0.47;
            //     delta_theta = theta;
            // }

            // // position value
            // pos_x += delta_s * cos(pos_z + (delta_theta / 2.0));
            // pos_y += delta_s * sin(pos_z + (delta_theta / 2.0));
            // pos_z += delta_theta;

            // vel_x = delta_s / delta_time;
            // vel_y = 0.0;
            // vel_z = delta_theta / delta_time;

            // recv value
            vel_x = msg->twist.linear.x * scale_x;
            vel_y = msg->twist.linear.y * scale_y;
            vel_z = msg->twist.angular.z * scale_z;
            // delta value
            delta_x = (vel_x * cos(pos_z) - vel_y * sin(pos_z)) * delta_time;
            delta_y = (vel_x * sin(pos_z) + vel_y * cos(pos_z)) * delta_time;
            delta_z = vel_z * delta_time;
            // position value
            pos_x += delta_x;
            pos_y += delta_y;
            pos_z += delta_z;

            // Euler to Quaternion
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.00, 0.00, pos_z);

            data_calc.x_pos = pos_x;
            data_calc.y_pos = pos_y;
            data_calc.z_angle = pos_z;

            while(data_calc.z_angle > M_PI)
            {
                data_calc.z_angle -= 2.0 * M_PI;
            }
            while(data_calc.z_angle < -M_PI)
            {
                data_calc.z_angle += 2.0 * M_PI;
            }

            // publish tf
            tf.header.stamp = curr_time;
            tf.transform.translation.x = pos_x;
            tf.transform.translation.y = pos_y;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = quaternion.x();
            tf.transform.rotation.y = quaternion.y();
            tf.transform.rotation.z = quaternion.z();
            tf.transform.rotation.w = quaternion.w();

            tf_broadcaster->sendTransform(tf);

            // publish odom message
            odom_settings.header.stamp = curr_time;
            odom_settings.header.frame_id = "odom";

            odom_settings.pose.pose.position.x = pos_x;
            odom_settings.pose.pose.position.y = pos_y;
            odom_settings.pose.pose.position.z = 0.0;

            odom_settings.pose.pose.orientation.x = quaternion.x();
            odom_settings.pose.pose.orientation.y = quaternion.y();
            odom_settings.pose.pose.orientation.z = quaternion.z();
            odom_settings.pose.pose.orientation.w = quaternion.w();

            odom_settings.child_frame_id = "base_footprint";
            odom_settings.twist.twist.linear.x = vel_x;
            odom_settings.twist.twist.linear.y = vel_y;
            odom_settings.twist.twist.angular.z = vel_z;

            odom_publisher_->publish(odom_settings);
            odometry_data_->publish(data_calc);
        }

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Publisher<message_interface::msg::Odomdata>::SharedPtr odometry_data_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CarOdom>());
	rclcpp::shutdown();
    return 0;
}
