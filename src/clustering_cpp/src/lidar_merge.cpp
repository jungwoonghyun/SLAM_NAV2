#include <iostream>
#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <message_filters/parameter_adapter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// class LidarMergeNode : public rclcpp::Node {
//     public:
//         LidarMergeNode() : Node("lidar_merge") {
//             lidar1_sub_.subscribe(this, "/lidar1/scan");
//             lidar2_sub_.subscribe(this, "/lidar2/scan");

//             sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), lidar1_sub_, lidar2_sub_));
//             sync_->registerCallback(std::bind(&LidarMergeNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

//             merged_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("merged_scan",10);

//             RCLCPP_INFO(this->get_logger(), "Lidar Merge Node Started");
//     }

//     private:
//         using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

//         //pub sub
//         rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub_;
//         message_filters::Subscriber::<sensor_msgs::msg::Laserscan> lidar1_sub_;
//         message_filters::Subscriber::<sensor_msgs::msg::Laserscan> lidar2_sub_;
//         std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
        
//         void sync_callback(const sensor_msgs::msg::LaserScan scan1, const sensor_msgs::msg::LaserScan scan2); 
// };

// void sync_callback(const sensor_msgs::msg::LaserScan scan1, const sensor_msgs::msg::LaserScan scan2) {
    
// }