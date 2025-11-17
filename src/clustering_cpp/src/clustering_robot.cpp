#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <ctime>
#include <thread>
#include <alsa/asoundlib.h>
#include <fstream>
#include <unistd.h>
#include <sys/reboot.h>
#include <linux/reboot.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <iomanip> // for setw, setfill
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/point.hpp"
#include <alsa/asoundlib.h>
#include <fstream>
#include <SFML/Audio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_interface/msg/mode.hpp" //#
#include "message_interface/msg/iscoast.hpp"
//#include "message_interface/msg/state.hpp"
#include "message_interface/msg/mode.hpp"
#include "message_interface/msg/buttonstate.hpp"
#include "message_interface/msg/ledcomm.hpp"
#include "message_interface/msg/ulsonicdist.hpp"
#include "message_interface/msg/odomdata.hpp"

using namespace std::chrono_literals;

#define ROI_WIDTH                   0.3
#define ROI_HEIGHT                  0.3
#define ROI_RANGE                   0.3
#define TARGET_EXCLUSIVE            0.4
#define COLLISION_DISTANCE          0.6
#define ULTRASONIC_LIMIT_DISTANCE   600    // 500
#define PLAYBACK_SPEED              0.3
#define STOP_DISTANCE               0.1

int g_target_detecting_state = 0;
int before_state = 0;
int sound_play_thread_controller = 1;
int shutdown = 0;

struct ObjInfo
{
    int id;
    std::vector<double> position{0.0,0.0};
    std::vector<double> leftbottom{0.0,0.0};
    std::vector<double> righttop{0.0,0.0};
    double width;
    double height;
};

class clusteringNode : public rclcpp::Node
{
    public:
        clusteringNode()
        : Node("clustering_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
        {
            // Qos depth = 10
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

            laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
            rclcpp::SensorDataQoS(), std::bind(&clusteringNode::msgCallback_lidar, this, std::placeholders::_1));
            // joystick_sub = this->create_subscription<message_interface::msg::Buttonstate>("joystick_status",
            // qos_profile, std::bind(&clusteringNode::msgCallback_joystick, this, std::placeholders::_1));
            button_sub = this->create_subscription<message_interface::msg::Buttonstate>("button_status",
            10, std::bind(&clusteringNode::msgCallback_button, this, std::placeholders::_1)); //
            ultrasonic_sub = this->create_subscription<message_interface::msg::Ulsonicdist>("ultrasonic_distance",
            qos_profile, std::bind(&clusteringNode::msgCallback_ultrasonic, this, std::placeholders::_1));
            odom_data_sub = this->create_subscription<message_interface::msg::Odomdata>("foot_data",
            qos_profile, std::bind(&clusteringNode::msgCallback_odom, this, std::placeholders::_1));

            cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
            map_out_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("out_cloud", qos_profile);
            rviz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("obj_detect_marker_array", 10);
            footprint_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("base_footprint_marker_array", 10);
            led_pub = this->create_publisher<message_interface::msg::Ledcomm>("led_instructor", 10);
            controller_pub = this->create_publisher<message_interface::msg::Iscoast>("is_coast", 10);
            following_state_pub = this->create_publisher<message_interface::msg::Mode>("following_state", qos_profile);

            timer = this->create_wall_timer(100ms, std::bind(&clusteringNode::timer_callback, this));
        }

        ~clusteringNode()
        {

        }

        int target_id = -2;
        double min_laser = 0.4;
        double roi_center_x = 0.5, roi_center_y = 0;
        int frontdist = 0, backdist = 0;
        int bef_state[3] = {0};
        int coast_bef_state = 0, point_bef_state = 0;
        int button_led[3] = {0};
        int joystick_state[4] = {0};
        int button_state[3] = {0};
        int shutdown_input = 0;
        int coast_state = 0, point_state = 0;
        bool button_return, blink = 0;
        int g_target_lost = 0, g_target_detect = 0;
        bool remote_controller = false;
        double prev_target_distance = 0, prev_target_angle = 0, prev_vel_dest = 0;
        std::vector<double> g_linear_data, g_angular_data, g_pos_x, g_pos_y, g_rot_z;
        float odom_x, odom_y, odom_z;
        float odom_z_save;
        std::vector<int> g_spin_data;
        int g_cmdvel_index = 0, g_spin_index = 0, play_re_index = 0;
        int end_point = 0;
        int dt = 0, dt_spin_start, dt_spin_end=0, cnt = 0;
        bool truefalse_playback = true, truefalse_replay = true, reset_data = true; //

    private:

        std::chrono::steady_clock::time_point start_time;

        // variable
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_filtered{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        // rclcpp::Subscription<message_interface::msg::Buttonstate>::SharedPtr joystick_sub;
        rclcpp::Subscription<message_interface::msg::Buttonstate>::SharedPtr button_sub;
        rclcpp::Subscription<message_interface::msg::Ulsonicdist>::SharedPtr ultrasonic_sub;
        rclcpp::Subscription<message_interface::msg::Odomdata>::SharedPtr odom_data_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_out_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_pub;
        rclcpp::Publisher<message_interface::msg::Ledcomm>::SharedPtr led_pub;
        rclcpp::Publisher<message_interface::msg::Iscoast>::SharedPtr controller_pub;
        rclcpp::Publisher<message_interface::msg::Mode>::SharedPtr following_state_pub;
        rclcpp::TimerBase::SharedPtr timer;

        // function
        void msgCallback_lidar(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        // void msgCallback_joystick(const message_interface::msg::Buttonstate & msg);
        void msgCallback_button(const message_interface::msg::Buttonstate & msg);
        void msgCallback_ultrasonic(const message_interface::msg::Ulsonicdist msg);
        void msgCallback_odom(const message_interface::msg::Odomdata msg);
        void timer_callback();

        void makeRefMap(void);
        void revMakeRefMap(void);
        void mapDownsampling(void);
        void filteredEuclideanClustering(std::vector<ObjInfo>& info);
        void euclideanClustering(std::vector<ObjInfo>& info);
        int targetDetect(std::vector<ObjInfo> obj_info);
        int roiDetect(std::vector<ObjInfo> obj_info);
        int collisionAvoidance(std::vector<ObjInfo> obj_info);
        void mapOutPub(std::vector<ObjInfo> obj_info_vector, int id);
        void pubMarker(std::vector<ObjInfo> obj_info);
        void footprintMarker(void);
};

void clusteringNode::msgCallback_lidar(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    sensor_msgs::msg::LaserScan scan_msg = *msg;
    filtered_lidar_cloud->points.clear();
    lidar_cloud->points.clear();

    //thread
    for (size_t i = 0; i < scan_msg.ranges.size(); i++)
    {
        double angle = (scan_msg.angle_increment) * i + scan_msg.angle_min;
        double range = scan_msg.ranges[i];

        if(std::isnan(range)) //|| (range > ROI_RANGE && range < ROI_RANGE + TARGET_EXCLUSIVE)
        {
            range = 0.0;
        }

        if(range < COLLISION_DISTANCE)
        {
            min_laser = range;
        }

        double lidar_x = range * cos(angle);
        double lidar_y = range * sin(angle);

        if( sqrt(pow(roi_center_x - lidar_x, 2) + pow(roi_center_y - lidar_y, 2)) < ROI_RANGE
        || sqrt(pow(roi_center_x - lidar_x, 2) + pow(roi_center_y - lidar_y, 2)) > ROI_RANGE + TARGET_EXCLUSIVE
        && sqrt(pow(lidar_x, 2) + pow(lidar_y, 2)) > 0.05)
        {
            filtered_lidar_cloud->points.push_back(pcl::PointXYZ(lidar_x, lidar_y, 0.0));
        }

        lidar_cloud->points.push_back(pcl::PointXYZ(lidar_x, lidar_y, 0.0));
    }
    filtered_lidar_cloud->width = filtered_lidar_cloud->points.size();
    filtered_lidar_cloud->height = 1;
    filtered_lidar_cloud->is_dense = true;

    lidar_cloud->width = lidar_cloud->points.size();
    lidar_cloud->height = 1;
    lidar_cloud->is_dense = true;
}

// void clusteringNode::msgCallback_joystick(const message_interface::msg::Buttonstate & msg) // joystick can be pushed one by one
// {
//     for(int i = 0; i < 4; i++)
//     {
//         joystick_state[i] = msg.button[i];
//     }
//     RCLCPP_INFO(this->get_logger(), "Button msg : %2d, %2d, %2d, %2d ", msg.button[0], msg.button[1], msg.button[2], msg.button[3]);
//     RCLCPP_INFO(this->get_logger(), "Joystick State : %2d, %2d, %2d, %2d ", joystick_state[0], joystick_state[1], joystick_state[2], joystick_state[3]);
// }

void clusteringNode::msgCallback_button(const message_interface::msg::Buttonstate & msg)
{
    geometry_msgs::msg::Twist twist;
    int io_input = 0, coast_input = 0, point_input = 0;

    for(int i = 0; i < 4; i++)
    {
        joystick_state[i] = msg.button[i];
    }

    for(int i=0; i < 3; i++)
    {
        io_input = io_input + msg.button[i+4];
    }

    coast_input = coast_input + msg.button[8];
    point_input = point_input + msg.button[9];

    // following
    if(io_input < 2) // maximum one button clicked
    {
        if(msg.button[4] == 1)
        {
            if(bef_state[0] != msg.button[4])
            {
                if(button_state[0] == 1)
                {
                    end_point++;
                }
                button_state[0]++;
                if(button_state[0] > 1) // button_state[0]: 0 stop, 1 following
                {
                    button_state[0] = 0;
                }
                bef_state[0] = msg.button[4];
            }
        }
        else
        {
            bef_state[0] = msg.button[4];
        }
    }
    else // more than one button clicked
    {
        bef_state[0] = 0;
        button_state[0] = 0;
    }

    // playback & reverse playback
    for(int i = 1; i < 3; i++) // io board
    {
        if(io_input < 2) // maximum one button clicked
        {
            if(msg.button[i+4] == 1)
            {
                if(bef_state[i] != msg.button[i+4])
                {
                    button_state[i]++;
                    if(button_state[i] > 2) // button_state[1]: 0 stop, 1 blink, 2 playback
                                            // button_state[2]: 0 stop, 1 blink, 2 reverse playback
                    {
                        button_state[i] = 0;
                    }
                    bef_state[i] = msg.button[i+4];
                }
            }
            else
            {
                bef_state[i] = msg.button[i+4];
            }
        }
        else // more than one button clicked
        {
            bef_state[i] = 0;
            button_state[i] = 0;
        }
    }

    io_input = 0;

    for(int i=0; i < 3; i++)
    {
        if(button_state[i] > 0)
        {
            io_input++;
        }
    }

    if(io_input>=2)
    {
        bef_state[0] = 0;
        bef_state[1] = 0;
        bef_state[2] = 0;
        button_state[0] = 0;
        button_state[1] = 0;
        button_state[2] = 0;
    }

    if(msg.button[7] == 0)
    {
       shutdown_input = 0;
    }
    else
    {
        shutdown_input++;
    }

    if(shutdown_input >= 40) // shutdown button
    {
        twist.angular.z = 0;
        twist.linear.x = 0;

        cmd_vel_pub->publish(twist);

        shutdown = 1;
        // system("sudo /sbin/shutdown -h now");
    }

    // if(coast_input < 2) // maximum one button clicked
    // {
    //     if(msg.button[8] == 1)
    //     {
    //         if(coast_bef_state != msg.button[8])
    //         {
    //             coast_state++;
    //             if(coast_state > 1) // button_state[0]: 0 stop, 1 following
    //             {
    //                 coast_state = 0;
    //             }
    //             coast_bef_state = msg.button[8]; // coast button
    //         }
    //     }
    //     else
    //     {
    //         coast_bef_state = msg.button[8];
    //     }
    // }
    
    // else // more than one button clicked
    // {
    //     coast_bef_state = 0;
    //     coast_state = 0;
    // }

    if(point_state < 2)
    {
        if(msg.button[9] == 1 && button_state[0] == 1)
        {
            if(point_bef_state != msg.button[9])
            {
                point_state++;
                if(point_state > 1)
                {
                    point_state = 0;
                }
                point_bef_state = msg.button[9]; // coast button
            }
        }
        else
        {
            point_bef_state = msg.button[9];
        }
    }
    else
    {
        point_state = 0;
        point_bef_state = 0;
    }
    

    // RCLCPP_INFO(this->get_logger(), "Button msg : %2d, Coast_bef_state %2d Coast state %2d ", msg.button[8], coast_bef_state, coast_state);
    // RCLCPP_INFO(this->get_logger(), "Button State : %2d, %2d, %2d ", button_state[0], button_state[1], button_state[2]);
}

void clusteringNode::msgCallback_ultrasonic(const message_interface::msg::Ulsonicdist msg)
{
    // ultrasonic unit is mm

    frontdist = msg.frontdist;
    backdist = msg.backdist;
}

void clusteringNode::msgCallback_odom(const message_interface::msg::Odomdata msg)
{
    odom_x = msg.x_pos;
    odom_y = msg.y_pos;
    odom_z = msg.z_angle;
}

//////////////////////////////////////

void clusteringNode::timer_callback()
{
    double linear_ = 0.0, angular_ = 0.0;
    double target_distance = 0, target_angle = 0, target_vel = 0, robot_vel = 0, vel_dest = 0;
    double velocity_comm = 0, bef_vel_comm = 0;
    int collision;
    double weight_linear = 0.9, weight_angular = 0.92;
    double distance_sub;
    double spin_angle;

    geometry_msgs::msg::Twist twist;
    message_interface::msg::Ledcomm ledcomm;
    message_interface::msg::Iscoast is_motor_coast;
    message_interface::msg::Ulsonicdist ultrasonic_distance_data;
    message_interface::msg::Mode determinant_value;

    std::vector<ObjInfo> target_obj_info_vector;
    std::vector<ObjInfo> collision_obj_info_vector;

    if (filtered_lidar_cloud->points.size() == 0)
    {
        return;
    }

    makeRefMap();

    mapDownsampling();

    filteredEuclideanClustering(target_obj_info_vector);
    euclideanClustering(collision_obj_info_vector); // make cluster and give info(like id, width...)
                                                    // to each cluster

    // target detecting or not
    if(target_id == -2)
    {
        target_id = targetDetect(target_obj_info_vector); // target_id = -1 detect failed
        remote_controller = true;
    }
    else
    {
        target_id = roiDetect(target_obj_info_vector); // Get roi center position & get target id

        if(button_state[0] == 0 && button_state[1] == 0 && button_state[2] == 0) // mode off
        {
            roi_center_x = 0.5;
            roi_center_y = 0;
            g_target_lost = 0;
            g_target_detect = 0;
            dt = 0;
            reset_data = true;
            truefalse_playback = true;
            truefalse_replay = true;

            if(coast_state != 1) // joystick
            {
                if(joystick_state[0] == 1)
                {
                    if((frontdist < ULTRASONIC_LIMIT_DISTANCE) && (frontdist > 0))//ULTRASONIC_LIMIT_DISTANCE)
                    {
                        linear_ = 0.0;
                    }
                    else
                    {
                        linear_ = 0.4;
                    }
                    angular_ = 0.0;
                }
                else if(joystick_state[1] == 1)
                {
                    if((backdist < ULTRASONIC_LIMIT_DISTANCE) && (backdist > 0)) //ULTRASONIC_LIMIT_DISTANCE
                    {
                        linear_ = 0.0;
                    }
                    else
                    {
                        linear_ = -0.4;
                    }
                    angular_ = 0.0;
                }
                else if(joystick_state[2] == 1)
                {
                    linear_ = 0.0;
                    angular_ = 0.4;
                }
                else if(joystick_state[3] == 1)
                {
                    linear_ = 0.0;
                    angular_ = -0.4;
                }
                else
                {
                    linear_ = 0.0;
                    angular_ = 0.0;
                }

                remote_controller = true;
                button_led[0] = 0;
                button_led[1] = 0;
                button_led[2] = 0;

                twist.angular.z = angular_;
                twist.linear.x = linear_;
                ledcomm.led[0] = button_led[0];
                ledcomm.led[1] = button_led[1];
                ledcomm.led[2] = button_led[2];
                is_motor_coast.iscoast = remote_controller;

                g_target_detecting_state = 0;

                determinant_value.determinant = 0;
            }
            else
            {
                remote_controller = true;
                button_led[0] = 0;
                button_led[1] = 0;
                button_led[2] = 0;

                twist.angular.z = angular_;
                twist.linear.x = linear_;
                ledcomm.led[0] = button_led[0];
                ledcomm.led[1] = button_led[1];
                ledcomm.led[2] = button_led[2];
                is_motor_coast.iscoast = remote_controller;

                g_target_detecting_state = 0;

                determinant_value.determinant = 2;
            }

            if (end_point != 0) // end point
            {
                g_pos_x.push_back(odom_x);
                g_pos_y.push_back(odom_y);
                g_rot_z.push_back(odom_z);
                g_linear_data.push_back(linear_);
                g_angular_data.push_back(angular_);
                end_point = 0;
            }
        }
        else if (button_state[0] == 1 && button_state[1] == 0 && button_state[2] == 0) // following mode and memorize cmd_vel to go back starting point
        {
            if(truefalse_playback == false) // valid value kill reset
            {
                truefalse_playback = true;
            }

            if(truefalse_replay == false)
            {
                truefalse_replay = true;
            }

            if(reset_data == true)
            {
                g_linear_data.clear();
                g_angular_data.clear();
                g_pos_x.clear();
                g_pos_y.clear();
                g_rot_z.clear();
                g_linear_data.shrink_to_fit();
                g_angular_data.shrink_to_fit();
                g_pos_x.shrink_to_fit();
                g_pos_y.shrink_to_fit();
                g_rot_z.shrink_to_fit();
                g_cmdvel_index = 0;

                g_pos_x.push_back(0.0);
                g_pos_y.push_back(0.0);
                g_rot_z.push_back(0.0);
                g_linear_data.push_back(0.0);
                g_angular_data.push_back(0.0);
                g_cmdvel_index++;
                reset_data = false;
            }

            // start of following
            if(target_id != -1)
            {
                if(g_target_lost == 1) // lost
                {
                    linear_ = 0.0;
                    angular_ = 0.0;
                    roi_center_x = 0.5;
                    roi_center_y = 0;
                    remote_controller = true;

                    if(blink == false)
                    {
                        blink = true;
                        ledcomm.led[0] = 1;
                    }
                    else
                    {
                        blink = false;
                        ledcomm.led[0] = 0;
                    }

                    g_target_detecting_state = 2;
                }
                else // move
                {
                    target_distance = sqrt(pow(target_obj_info_vector[target_id].position[0],2) + pow(target_obj_info_vector[target_id].position[1],2));
                    target_angle = atan2(target_obj_info_vector[target_id].position[1], target_obj_info_vector[target_id].position[0]);

                    velocity_comm = 2.0 * (target_distance - COLLISION_DISTANCE);
                    if(velocity_comm < 0.0)
                    {
                        velocity_comm = 0.0;
                    }
                    
                    linear_ = weight_linear*velocity_comm + (1 - weight_linear)*bef_vel_comm;
                    angular_ = 1.0 * target_angle;

                    if((frontdist < ULTRASONIC_LIMIT_DISTANCE) && (frontdist > 0))
                    {
                        linear_ = 0.0;
                        angular_ = 0.0;
                    }

                    if(linear_ >= 1.3)
                    {
                        linear_ = 1.3;
                    }

                    if(abs(angular_) < M_PI/180)
                    {
                        angular_ = 0.0;
                    }

                    prev_target_distance = linear_;
                    prev_target_angle = angular_;
                    bef_vel_comm = velocity_comm;

                    remote_controller = false;

                    g_target_detect = 1;
                    ledcomm.led[0] = 1;

                    g_target_detecting_state = 1;
                }
            }
            else if(target_id == -1)
            {
                linear_ = 0.0;
                angular_ = 0.0;
                roi_center_x = 0.5;
                roi_center_y = 0;
                remote_controller = true;

                if(g_target_detect == 1) // lost
                {
                    g_target_lost = 1;

                    if(blink == false)
                    {
                        blink = true;
                        ledcomm.led[0] = 1;
                    }
                    else
                    {
                        blink = false;
                        ledcomm.led[0] = 0;
                    }

                    g_target_detecting_state = 2;
                }
                else // init
                {
                    g_linear_data.clear();
                    g_angular_data.clear();
                    g_pos_x.clear();
                    g_pos_y.clear();
                    g_rot_z.clear();
                    g_linear_data.shrink_to_fit();
                    g_angular_data.shrink_to_fit();
                    g_pos_x.shrink_to_fit();
                    g_pos_y.shrink_to_fit();
                    g_rot_z.shrink_to_fit();
                    g_cmdvel_index = 0;

                    g_pos_x.push_back(0.0);
                    g_pos_y.push_back(0.0);
                    g_rot_z.push_back(0.0);
                    g_linear_data.push_back(0.0);
                    g_angular_data.push_back(0.0);
                    g_cmdvel_index++;
                    ledcomm.led[0] = 1;

                    std::cout << "init\n\n\n\n\n";
                }
            }
            else
            {
                linear_ = 0.0;
                angular_ = 0.0;
                remote_controller = true;
                button_state[0] = 0;
                button_state[1] = 0;
                button_state[2] = 0;
                std::cout << "exit\n";
            }

            // collision
            collision = collisionAvoidance(collision_obj_info_vector);

            if(collision == 1 || ((frontdist < ULTRASONIC_LIMIT_DISTANCE) && (frontdist > 0)))
            {
                linear_ = 0.0;
                angular_ = 0.0;
            }
            else
            {
                linear_ = linear_;
                angular_ = angular_;
            }

            ///////////////////////////
            //  memorize linear and angular data if following on
            //  during following mode if lost happened, datas are most useless so let it know
            ///////////////////////////

            if(g_target_lost != 1)
            {
                remote_controller = false;

                if(g_cmdvel_index == 1) // start point
                {
                    g_pos_x.push_back(odom_x);
                    g_pos_y.push_back(odom_y);
                    g_rot_z.push_back(odom_z);
                    g_linear_data.push_back(linear_);
                    g_angular_data.push_back(angular_);
                    g_cmdvel_index++;
                    std::cout << "x: " << odom_x << " y: " << odom_y << std::endl;
                }
                else if (point_state == 1) // mark point by button
                {
                    g_pos_x.push_back(odom_x);
                    g_pos_y.push_back(odom_y);
                    g_rot_z.push_back(odom_z);
                    g_linear_data.push_back(linear_);
                    g_angular_data.push_back(angular_);
                    g_cmdvel_index++;
                }
                
                // if(linear_ != 0.0 || angular_ != 0.0) // trace point
                // {                    
                //     cnt++; // processing point
                //     if(cnt == 8)
                //     {
                //         g_pos_x.push_back(odom_x);
                //         g_pos_y.push_back(odom_y);
                //         g_rot_z.push_back(odom_z);
                //         g_linear_data.push_back(linear_);
                //         g_angular_data.push_back(angular_);
                //         g_cmdvel_index++;
                //         cnt = 0;
                //     }
                // }
            } //collision
            else if(g_target_lost == 1)
            {
                linear_ = 0.0;
                angular_ = 0.0;
            }

            ///////// up is following down is joystick ///////////
            // if(joystick_state[0] == 1)
            // {
            //     if((frontdist < ULTRASONIC_LIMIT_DISTANCE) && (frontdist > 0))//ULTRASONIC_LIMIT_DISTANCE)
            //     {
            //         linear_ = 0.0;
            //     }
            //     else
            //     {
            //         linear_ = 0.4;
            //     }
            //     angular_ = 0.0;
            // }
            // else if(joystick_state[1] == 1)
            // {
            //     if((backdist < ULTRASONIC_LIMIT_DISTANCE) && (backdist > 0)) //ULTRASONIC_LIMIT_DISTANCE
            //     {
            //         linear_ = 0.0;
            //     }
            //     else
            //     {
            //         linear_ = -0.4;
            //     }
            //     angular_ = 0.0;
            // }
            // else if(joystick_state[2] == 1)
            // {
            //     linear_ = 0.0;
            //     angular_ = 0.4;
            // }
            // else if(joystick_state[3] == 1)
            // {
            //     linear_ = 0.0;
            //     angular_ = -0.4;
            // }
            // else
            // {
            //     linear_ = 0.0;
            //     angular_ = 0.0;
            // }
            // ledcomm.led[0] = 1;
            // collision = collisionAvoidance(collision_obj_info_vector);
            // if(collision == 1 || ((frontdist < ULTRASONIC_LIMIT_DISTANCE) && (frontdist > 0)))
            // {
            //     linear_ = 0.0;
            //     angular_ = 0.0;
            // }
            // else
            // {
            //     linear_ = linear_;
            //     angular_ = angular_;
            // }

            //  memorize linear and angular data if following on
            //  during following mode if lost happened, datas are most useless so let it know
            // if(g_cmdvel_index == 0) // start point
            // {
            //     g_pos_x.push_back(odom_x);
            //     g_pos_y.push_back(odom_y);
            //     g_rot_z.push_back(odom_z);
            //     g_linear_data.push_back(linear_);
            //     g_angular_data.push_back(angular_);
            //     g_cmdvel_index++;
            // }

            // if(linear_ != 0.0 || angular_ != 0.0)
            // {                    
            //     cnt++; // processing point
            //     if(cnt == 8)
            //     {
            //         g_pos_x.push_back(odom_x);
            //         g_pos_y.push_back(odom_y);
            //         g_rot_z.push_back(odom_z);
            //         g_linear_data.push_back(linear_);
            //         g_angular_data.push_back(angular_);
            //         g_cmdvel_index++;
            //         cnt = 0;
            //     }
            // }

            // if(linear_ != 0.0) 
            // {
            //     std::cout << "lin " << g_cmdvel_index << std::endl;
            // }
            // else if(angular_!= 0.0) 
            // {
            //     std::cout << "ang " << g_cmdvel_index << std::endl;
            // }
            // else
            // {
            //     std::cout << "both zero\n";
            // }
            //////////////////////////////////////////
            // std::cout << "follow cmdvel indexd: " << g_cmdvel_index << std::endl;

            twist.angular.z = angular_;
            twist.linear.x = linear_;
            ledcomm.led[1] = 0;
            ledcomm.led[2] = 0;
            is_motor_coast.iscoast = remote_controller;
            determinant_value.determinant = 0;
        }
        else if(button_state[0] == 0 && button_state[1] == 1 && button_state[2] == 0) // go back to starting point
        {
            ////////////////////////////////////////////
            //  blink to ask playback to starting point
            ////////////////////////////////////////////

            // led part
            // button_led = 0 is off, button_led = 1 is blink, button_led = 2 is on
            if(blink == false)
            {
                blink = true;
                ledcomm.led[1] = 1;
            }
            else
            {
                blink = false;
                ledcomm.led[1] = 0;
            }

            twist.angular.z = 0;
            twist.linear.x = 0;
            ledcomm.led[0] = 0;
            ledcomm.led[2] = 0;
            is_motor_coast.iscoast = remote_controller;
            g_target_detecting_state = 0;
            determinant_value.determinant = 1;
        }
        else if(button_state[0] == 0 && button_state[1] == 2 && button_state[2] == 0) // go back to starting point
        {
            ///////////////////////////
            //  play memorized linear and angular data from last data to first data
            //  robot moves like rear wheel drive => might be need to change data + to - , - to +
            ///////////////////////////
            remote_controller = false;

            if(truefalse_playback == true) // valid value kill
            {
                g_cmdvel_index--;
                truefalse_playback = false;
            }

            // collision
            collision = collisionAvoidance(collision_obj_info_vector);

            if(collision == 1 || ((frontdist < ULTRASONIC_LIMIT_DISTANCE) && (frontdist > 0)))
            {
                linear_ = 0.0;
                angular_ = 0.0;
                ledcomm.led[1] = 1;
            }
            else if (g_pos_x.empty() || g_pos_x[1] == odom_x) // g_pos_x[1] == odom_x
            {
                linear_ = 0;
                angular_ = 0;
                button_state[0] = 0;
                button_state[1] = 0;
                button_state[2] = 0;
                ledcomm.led[1] = 0;
            }
            else
            {
                distance_sub = sqrt(pow(g_pos_x[g_cmdvel_index] - (double)odom_x, 2)
                + pow(g_pos_y[g_cmdvel_index] - (double)odom_y, 2));

                ledcomm.led[1] = 1;

                if(dt < 10)
                {
                    angular_ = 0.0;
                    linear_ = 0.0;
                    dt += 1;
                }
                else if(dt < 50)
                {
                    angular_ = -0.25 * M_PI;
                    linear_ = 0.0;
                    dt += 1;
                }
                else if(dt < 80)
                {
                    angular_ = 0.0;
                    linear_ = 0.0;
                    dt += 1;
                }
                else
                {
                    if (g_cmdvel_index == 1)
                    {
                        if(distance_sub < STOP_DISTANCE)
                        {
                            linear_ = 0;
                            angular_ = 0;
                            button_state[0] = 0;
                            button_state[1] = 0;
                            button_state[2] = 0;
                            ledcomm.led[1] = 0;
                        }
                        else
                        {
                            if(g_pos_x[g_cmdvel_index] - odom_x > 0)
                            {
                                angular_ = atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x)) - odom_z;

                                if(angular_ < -M_PI_2)
                                {
                                    angular_ = angular_ + M_PI;
                                }
                                else if (angular_ > M_PI_2)
                                {
                                    angular_ = angular_ - M_PI;
                                }
                            }
                            else if (g_pos_x[g_cmdvel_index] - odom_x  == 0)
                            {
                                if(g_pos_y[g_cmdvel_index] - odom_y > 0) angular_ = -M_PI_2;
                                else if(g_pos_y[g_cmdvel_index] - odom_y == 0) angular_ = 0;
                                else if(g_pos_y[g_cmdvel_index] - odom_y <0) angular_ = M_PI_2;
                            }
                            else if (g_pos_x[g_cmdvel_index] - odom_x  < 0)
                            {
                                if(g_pos_y[g_cmdvel_index] - odom_y >= 0)
                                {
                                    angular_ = -M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x))- odom_z;

                                    if(angular_ < -M_PI_2)
                                    {
                                        angular_ = angular_ + M_PI;
                                    }
                                    else if (angular_ > M_PI_2)
                                    {
                                        angular_ = angular_ - M_PI;
                                    }
                                }
                                else if(g_pos_y[g_cmdvel_index] - odom_y <0)
                                {
                                    angular_ = M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y),(g_pos_x[g_cmdvel_index] - odom_x)) - odom_z;

                                    if(angular_ < -M_PI_2)
                                    {
                                        angular_ = angular_ + M_PI;
                                    }
                                    else if (angular_ > M_PI_2)
                                    {
                                        angular_ = angular_ - M_PI;
                                    }
                                }
                            }

                            if(distance_sub < 0.5)      linear_ = PLAYBACK_SPEED * 0.5;
                            else if(distance_sub < 1)   linear_ = PLAYBACK_SPEED * 0.7;
                            else                        linear_ = PLAYBACK_SPEED;
                        }
                    }
                    else if(g_cmdvel_index > 1)
                    {
                        // move to next point
                        // after arrive stop and spin toward next point

                        // if(distance_sub < STOP_DISTANCE)
                        // {
                        //     g_cmdvel_index--;
                        //     odom_z_save = odom_z;
                        //     linear_ = 0;
                        //     angular_ = 0;
                        // }
                        // else if (odom_z_save - g_rot_z[g_cmdvel_index] >= M_PI) // if odom_z and g_rot[i]'s difference is big. Spin in a position
                        // {
                        //     linear_ = 0;
                        //     angular_ = -PLAYBACK_SPEED;
                        //     if(g_rot_z[g_cmdvel_index] >= 0)
                        //     {
                        //         if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                        //         {
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     if(dt < 100)
                        //     {
                        //         dt += 1;
                        //         angular_ = 0;
                        //     }
                        // }
                        // else if(odom_z_save - g_rot_z[g_cmdvel_index] >= 0 
                        // && odom_z_save - g_rot_z[g_cmdvel_index] < M_PI)
                        // {
                        //     linear_ = 0;
                        //     angular_ = PLAYBACK_SPEED;
                        //     if(g_rot_z[g_cmdvel_index] >= 0)
                        //     {
                        //         if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                        //         {
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     else if (g_rot_z[g_cmdvel_index] < 0)
                        //     {
                        //         if (abs(odom_z - g_rot_z[g_cmdvel_index] - M_PI) <= M_PI/100)
                        //         {
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     if(dt < 100)
                        //     {
                        //         dt += 1;
                        //         angular_ = 0;
                        //     }
                        // }
                        // else if(odom_z_save - g_rot_z[g_cmdvel_index] >= -M_PI 
                        // && odom_z_save - g_rot_z[g_cmdvel_index] < 0)
                        // {
                        //     linear_ = 0;
                        //     angular_ = -PLAYBACK_SPEED;
                        //     if(g_rot_z[g_cmdvel_index] >= 0)
                        //     {
                        //         if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                        //         {
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     else if (g_rot_z[g_cmdvel_index] < 0)
                        //     {
                        //         if (abs(odom_z - g_rot_z[g_cmdvel_index] - M_PI) <= M_PI/100)
                        //         {
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     if(dt < 100)
                        //     {
                        //         dt += 1;
                        //         angular_ = 0;
                        //     }
                        // }
                        // else if (odom_z_save - g_rot_z[g_cmdvel_index] < -M_PI)
                        // {
                        //     linear_ = 0;
                        //     angular_ = PLAYBACK_SPEED;
                        //     if(g_rot_z[g_cmdvel_index] >= 0)
                        //     {
                        //         if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                        //         {
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     else if (g_rot_z[g_cmdvel_index] < 0)
                        //     {
                        //         if (abs(odom_z - g_rot_z[g_cmdvel_index] - M_PI) <= M_PI/100)
                        //         {
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     if(dt < 100)
                        //     {
                        //         dt += 1;
                        //         angular_ = 0;
                        //     }
                        // }
                        // else // if odom_z and g_rot[i]'s difference is small. Move
                        // {
                        //     dt = 80;
                        //     if(g_pos_x[g_cmdvel_index] - (double)odom_x > 0)
                        //     {
                        //         angular_ = atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x)) - odom_z; // foot_data: without - odom_z; & odom_data: with - odom_z
                        //         if(angular_ < -M_PI_2)
                        //         {
                        //             angular_ = angular_ + M_PI;
                        //         }
                        //         else if (angular_ > M_PI_2)
                        //         {
                        //             angular_ = angular_ - M_PI;
                        //         }
                        //     }
                        //     else if (g_pos_x[g_cmdvel_index] - (double)odom_x  == 0)
                        //     {
                        //         if(g_pos_y[g_cmdvel_index] - (double)odom_y > 0) angular_ = -M_PI_2;
                        //         else if(g_pos_y[g_cmdvel_index] - (double)odom_y == 0) angular_ = 0;
                        //         else if(g_pos_y[g_cmdvel_index] - (double)odom_y <0) angular_ = M_PI_2;
                        //     }
                        //     else if (g_pos_x[g_cmdvel_index] - (double)odom_x  < 0)
                        //     {
                        //         if(g_pos_y[g_cmdvel_index] - (double)odom_y >= 0)
                        //         {
                        //             angular_ = -M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x)) - odom_z; // foot_data: without - odom_z; & odom_data: with - odom_z
                        //             if(angular_ < -M_PI_2)
                        //             {
                        //                 angular_ = angular_ + M_PI;
                        //             }
                        //             else if (angular_ > M_PI_2)
                        //             {
                        //                 angular_ = angular_ - M_PI;
                        //             }
                        //         }
                        //         else if(g_pos_y[g_cmdvel_index] - (double)odom_y <0)
                        //         {
                        //             angular_ = M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y),(g_pos_x[g_cmdvel_index] - odom_x)) - odom_z; // foot_data: without - odom_z; & odom_data: with - odom_z
                        //             if(angular_ < -M_PI_2)
                        //             {
                        //                 angular_ = angular_ + M_PI;
                        //             }
                        //             else if (angular_ > M_PI_2)
                        //             {
                        //                 angular_ = angular_ - M_PI;
                        //             }
                        //         }
                        //     }
                        //     if(angular_ > M_PI / 4.0 || angular_ < -M_PI / 4.0)
                        //     {
                        //         linear_ = 0.0;
                        //     }
                        //     else
                        //     {
                        //         if(distance_sub < 0.5)      linear_ = PLAYBACK_SPEED * 0.5;
                        //         else if(distance_sub < 1)   linear_ = PLAYBACK_SPEED * 0.7;
                        //         else                        linear_ = PLAYBACK_SPEED;
                        //     }
                        // }
                        
                        if(distance_sub >= STOP_DISTANCE)
                        {
                            dt = 80;
                            odom_z_save = odom_z;

                            if(g_pos_x[g_cmdvel_index] - (double)odom_x > 0)
                            {
                                angular_ = atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x)) - odom_z; // foot_data: without - odom_z; & odom_data: with - odom_z
                                if(angular_ < -M_PI_2)
                                {
                                    angular_ = angular_ + M_PI;
                                }
                                else if (angular_ > M_PI_2)
                                {
                                    angular_ = angular_ - M_PI;
                                }
                            }
                            else if (g_pos_x[g_cmdvel_index] - (double)odom_x  == 0)
                            {
                                if(g_pos_y[g_cmdvel_index] - (double)odom_y > 0) angular_ = -M_PI_2;
                                else if(g_pos_y[g_cmdvel_index] - (double)odom_y == 0) angular_ = 0;
                                else if(g_pos_y[g_cmdvel_index] - (double)odom_y <0) angular_ = M_PI_2;
                            }
                            else if (g_pos_x[g_cmdvel_index] - (double)odom_x  < 0)
                            {
                                if(g_pos_y[g_cmdvel_index] - (double)odom_y >= 0)
                                {
                                    angular_ = -M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x)) - odom_z; // foot_data: without - odom_z; & odom_data: with - odom_z
                                    if(angular_ < -M_PI_2)
                                    {
                                        angular_ = angular_ + M_PI;
                                    }
                                    else if (angular_ > M_PI_2)
                                    {
                                        angular_ = angular_ - M_PI;
                                    }
                                }
                                else if(g_pos_y[g_cmdvel_index] - (double)odom_y <0)
                                {
                                    angular_ = M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y),(g_pos_x[g_cmdvel_index] - odom_x)) - odom_z; // foot_data: without - odom_z; & odom_data: with - odom_z
                                    if(angular_ < -M_PI_2)
                                    {
                                        angular_ = angular_ + M_PI;
                                    }
                                    else if (angular_ > M_PI_2)
                                    {
                                        angular_ = angular_ - M_PI;
                                    }
                                }
                            }

                            if(angular_ > M_PI / 4.0 || angular_ < -M_PI / 4.0)
                            {
                                linear_ = 0.0;
                            }
                            else
                            {
                                if(distance_sub < 0.5)      linear_ = PLAYBACK_SPEED * 0.5;
                                else if(distance_sub < 1)   linear_ = PLAYBACK_SPEED * 0.7;
                                else                        linear_ = PLAYBACK_SPEED;
                            }
                        }
                        else // if odom_z and g_rot[i]'s difference is small. Move
                        {
                            if (odom_z_save - g_rot_z[g_cmdvel_index] >= M_PI) // if odom_z and g_rot[i]'s difference is big. Spin in a position
                            {
                                linear_ = 0;
                                angular_ = -PLAYBACK_SPEED;
                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] - M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }
                                

                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                            else if(odom_z_save - g_rot_z[g_cmdvel_index] >= 0 
                            && odom_z_save - g_rot_z[g_cmdvel_index] < M_PI)
                            {
                                linear_ = 0;
                                angular_ = PLAYBACK_SPEED;
                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] - M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }

                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                            else if(odom_z_save - g_rot_z[g_cmdvel_index] >= -M_PI 
                            && odom_z_save - g_rot_z[g_cmdvel_index] < 0)
                            {
                                linear_ = 0;
                                angular_ = -PLAYBACK_SPEED;
                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] - M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }
                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                            else if (odom_z_save - g_rot_z[g_cmdvel_index] < -M_PI)
                            {
                                linear_ = 0;
                                angular_ = PLAYBACK_SPEED;
                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] + M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index] - M_PI) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                        g_cmdvel_index--;
                                    }
                                }

                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                        }

                        // }
                        // else if (g_linear_data[g_cmdvel_index] == 0 && g_angular_data[g_cmdvel_index] == 0)
                        // {
                        //     g_cmdvel_index--;
                        // }
                        // else // g_linear_data[g_cmdvel_index] == 0 && g_angular_data[g_cmdvel_index] != 0
                        // {
                        //     dt_spin_end = 0;
                        //     linear_ = 0;
                        //     int i;
                        //     if(distance_sub < STOP_DISTANCE)
                        //     {
                        //         odom_z_save = odom_z;
                        //     }
                        //     while (sqrt(pow(g_pos_x[g_cmdvel_index - g_spin_index] - (double)odom_x, 2)
                        //     + pow(g_pos_y[g_cmdvel_index - g_spin_index] - (double)odom_y, 2)) < 0.1)
                        //     {
                        //         g_spin_index++;
                        //         dt_spin_start = dt;
                        //         std::cout << "while\n";
                        //     }     //     i = g_cmdvel_index - g_spin_index;
                        //     if(odom_z_save - g_rot_z[i] >= M_PI)
                        //     {
                        //         angular_ = -PLAYBACK_SPEED;
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] + M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z > 0\n";
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] - M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z < 0\n";
                        //         }
                        //         std::cout << "> M_PI\n";
                        //     }
                        //     else if (odom_z_save - g_rot_z[i] < -M_PI)
                        //     {
                        //         angular_ = PLAYBACK_SPEED;
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] + M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z > 0\n";
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] - M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z < 0\n";
                        //         }
                        //         std::cout << "< -M_PI\n";
                        //     }
                        //     else if(odom_z_save - g_rot_z[i] >= 0 && odom_z_save - g_rot_z[i] < M_PI)
                        //     {
                        //         angular_ = PLAYBACK_SPEED; 
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] + M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z > 0\n";
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] - M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z < 0\n";
                        //         }
                        //         std::cout << "0~M_PI\n"; // o
                        //     }
                        //     else if(odom_z_save - g_rot_z[i] >= -M_PI && odom_z_save - g_rot_z[i] < 0)
                        //     {
                        //         angular_ = -PLAYBACK_SPEED;
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] + M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z > 0\n";
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i] - M_PI) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //             std::cout << "rot_z < 0\n";
                        //         }
                        //         std::cout << "-M_PI~0\n"; // o
                        //     }
                        //     else
                        //     {
                        //         angular_ = -PLAYBACK_SPEED;
                        //         if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //         {
                        //             g_cmdvel_index = i;
                        //             g_spin_index = 0;
                        //             angular_ = 0;
                        //         }
                        //         std::cout << "elses\n";
                        //     }
                        //     if(angular_ == 0)
                        //     {
                        //         play_re_index++;
                        //         if(play_re_index > 1)
                        //         {
                        //             g_cmdvel_index--;
                        //             play_re_index = 0;
                        //         }
                        //     }
                        //     if(dt - dt_spin_start < 20)
                        //     {
                        //         dt++;
                        //         angular_ = 0;
                        //     }
                        //     // if(distance_sub < STOP_DISTANCE)
                        //     // {
                        //     //     g_spin_index++;
                        //     //     odom_z_save = odom_z;
                        //     //     std::cout << "spin index " << g_spin_index << std::endl;
                        //     // }
                        //     // else if(distance_sub >= STOP_DISTANCE)
                        //     // {
                        //     //     std::cout << "odom_z_save: " << odom_z_save << " g_rot_z[g_cmdvel_index - g_spin_index] " << g_rot_z[g_cmdvel_index - g_spin_index] << std::endl;
                        //     //     if(odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] > 2*M_PI)
                        //     //     {
                        //     //         spin_angle = odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] - M_PI;
                        //     //     }
                        //     //     else if (odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] < -2*M_PI)
                        //     //     {
                        //     //         spin_angle = odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] + M_PI;
                        //     //     }
                        //     //     else
                        //     //     {
                        //     //         spin_angle = odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //     }
                        //     //     std::cout << "spin angle: " << spin_angle << std::endl;
                        //     //     if(spin_angle < 0) // && spin_angle > -M_PI
                        //     //     {
                        //     //         angular_ = -PLAYBACK_SPEED;
                        //     //         // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //         if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] + M_PI)) < M_PI/100)
                        //     //         {
                        //     //             // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //             g_spin_index = 0;
                        //     //         }
                        //     //     }
                        //     //     // else if (spin_angle < -M_PI)
                        //     //     // {
                        //     //     //     angular_ = PLAYBACK_SPEED;
                        //     //     //     // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //     //     if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] + M_PI)) < M_PI/100)
                        //     //     //     {
                        //     //     //         // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //     //         g_spin_index = 0;
                        //     //     //     }
                        //     //     // }
                        //     //     else if (spin_angle > 0) // && spin_angle < M_PI
                        //     //     {
                        //     //         angular_ = PLAYBACK_SPEED;
                        //     //         // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //         if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] - M_PI)) <= M_PI/100)
                        //     //         {
                        //     //             // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //             g_spin_index = 0;
                        //     //         }
                        //     //     }
                        //     //     // else if (spin_angle > M_PI)
                        //     //     // {
                        //     //     //     angular_ = -PLAYBACK_SPEED;
                        //     //     //     // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //     //     if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] - M_PI)) <= M_PI/100)
                        //     //     //     {
                        //     //     //         // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //     //         g_spin_index = 0;
                        //     //     //     }
                        //     //     // }
                        //     //     else
                        //     //     {
                        //     //         std::cout << "else spin angle\n";
                        //     //     }
                        //     // }
                        // }
                    }
                    else
                    {
                        linear_ = 0;
                        angular_ = 0;
                        button_state[0] = 0;
                        button_state[1] = 0;
                        button_state[2] = 0;
                        ledcomm.led[1] = 0;                        
                    }
                }
            }
            twist.angular.z = angular_;
            twist.linear.x = linear_;
            ledcomm.led[0] = 0;
            ledcomm.led[2] = 0;
            is_motor_coast.iscoast = remote_controller;
            g_target_detecting_state = 0;
            determinant_value.determinant = 1;
        }
        else if (button_state[0] == 0 && button_state[1] == 0 && button_state[2] == 1) // go back to robot's target(robot owner) point
        {
            ////////////////////////////////////////////
            //  blink to ask playback to followed point
            ////////////////////////////////////////////

            // led part
            // button_led = 0 is off, button_led = 1 is blink, button_led = 2 is on
            if(blink == false)
            {
                blink = true;
                ledcomm.led[2] = 1;
            }
            else
            {
                blink = false;
                ledcomm.led[2] = 0;
            }

            twist.angular.z = 0;
            twist.linear.x = 0;
            ledcomm.led[0] = 0;
            ledcomm.led[1] = 0;
            is_motor_coast.iscoast = remote_controller;
            g_target_detecting_state = 0;
            determinant_value.determinant = 1;
        }
        else if (button_state[0] == 0 && button_state[1] == 0 && button_state[2] == 2) // go back to robot's target(robot owner) point
        {
            ///////////////////////////
            //  send original linear and angular data
            ///////////////////////////

            remote_controller = false;

            if(truefalse_replay == true) // valid value kill
            {
                g_cmdvel_index++;
                truefalse_replay = false;
            }
            
            // collision
            collision = collisionAvoidance(collision_obj_info_vector);
            distance_sub = sqrt(pow(g_pos_x[g_cmdvel_index] - (double)odom_x, 2)
            + pow(g_pos_y[g_cmdvel_index] - (double)odom_y, 2));

            if(collision == 1 || ((frontdist < ULTRASONIC_LIMIT_DISTANCE) && (frontdist > 0)))
            {
                angular_ = 0;
                linear_ = 0;
                ledcomm.led[2] = 1;
            }
            else if (g_pos_x.empty()|| g_pos_x[1] == odom_x)
            {
                linear_ = 0;
                angular_ = 0;
                button_state[0] = 0;
                button_state[1] = 0;
                button_state[2] = 0;
                ledcomm.led[2] = 0;
            }
            else
            {
                if(g_cmdvel_index >= 1 && (g_cmdvel_index <= g_pos_x.size()-1))
                {
                    ledcomm.led[2] = 1;

                    if(dt < 10)
                    {
                        angular_ = 0.0;
                        linear_ = 0.0;
                        dt += 1;
                    }
                    else if(dt < 50)
                    {
                        angular_ = -0.25 * M_PI;
                        linear_ = 0.0;
                        dt += 1;
                    }
                    else if(dt < 80)
                    {
                        angular_ = 0.0;
                        linear_ = 0.0;
                        dt += 1;
                    }
                    else if(dt >= 80)
                    {
                        // if(g_linear_data[g_cmdvel_index] != 0 || distance_sub >= STOP_DISTANCE)
                        // {
                            if(distance_sub < STOP_DISTANCE)
                            {
                                if(g_cmdvel_index < g_pos_x.size()-1)
                                {
                                    g_cmdvel_index++;
                                    odom_z_save = odom_z;
                                }
                                else
                                {
                                    button_state[0] = 0;
                                    button_state[1] = 0;
                                    button_state[2] = 0;
                                    ledcomm.led[2] = 0;                                    
                                }

                                linear_ = 0;
                                angular_ = 0;
                            }
                            else if (g_rot_z[g_cmdvel_index] - odom_z_save >= M_PI)
                            {
                                linear_ = 0;
                                angular_ = -PLAYBACK_SPEED;

                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }

                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                            else if(g_rot_z[g_cmdvel_index] - odom_z_save >= 0 
                            && g_rot_z[g_cmdvel_index] - odom_z_save < M_PI)
                            {
                                linear_ = 0;
                                angular_ = PLAYBACK_SPEED;
                                
                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }

                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                            else if(g_rot_z[g_cmdvel_index] - odom_z_save >= -M_PI 
                            && g_rot_z[g_cmdvel_index] - odom_z_save < 0)
                            {
                                linear_ = 0;
                                angular_ = -PLAYBACK_SPEED;

                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }

                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                            else if (g_rot_z[g_cmdvel_index] - odom_z_save < -M_PI)
                            {
                                linear_ = 0;
                                angular_ = PLAYBACK_SPEED;

                                if(g_rot_z[g_cmdvel_index] >= 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }
                                else if (g_rot_z[g_cmdvel_index] < 0)
                                {
                                    if (abs(odom_z - g_rot_z[g_cmdvel_index]) <= M_PI/100)
                                    {
                                        angular_ = 0;
                                    }
                                }

                                if(dt < 100)
                                {
                                    dt += 1;
                                    angular_ = 0;
                                }
                            }
                            else
                            {
                                dt = 80;

                                if(g_pos_x[g_cmdvel_index] - odom_x > 0)
                                {
                                    angular_ = atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x)) - (double)odom_z; 
                                    if(angular_ < -M_PI_2)
                                    {
                                        angular_ = angular_ + M_PI;
                                    }
                                    else if (angular_ > M_PI_2)
                                    {
                                        angular_ = angular_ - M_PI;
                                    }
                                }
                                else if (g_pos_x[g_cmdvel_index] - odom_x  == 0)
                                {
                                    if(g_pos_y[g_cmdvel_index] - odom_y > 0) angular_ = -M_PI_2;
                                    else if(g_pos_y[g_cmdvel_index] - odom_y == 0) angular_ = 0;
                                    else if(g_pos_y[g_cmdvel_index] - odom_y <0) angular_ = M_PI_2;
                                }
                                else if (g_pos_x[g_cmdvel_index] - odom_x  < 0)
                                {
                                    if(g_pos_y[g_cmdvel_index] - odom_y >= 0)
                                    {
                                        angular_ = -M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y), (g_pos_x[g_cmdvel_index] - odom_x)) - (double)odom_z; 
                                        if(angular_ < -M_PI_2)
                                        {
                                            angular_ = angular_ + M_PI;
                                        }
                                        else if (angular_ > M_PI_2)
                                        {
                                            angular_ = angular_ - M_PI;
                                        }
                                    }
                                    else if(g_pos_y[g_cmdvel_index] - odom_y <0)
                                    {
                                        angular_ = M_PI + atan2((g_pos_y[g_cmdvel_index] - odom_y),(g_pos_x[g_cmdvel_index] - odom_x)) - (double)odom_z; 
                                        if(angular_ < -M_PI_2)
                                        {
                                            angular_ = angular_ + M_PI;
                                        }
                                        else if (angular_ > M_PI_2)
                                        {
                                            angular_ = angular_ - M_PI;
                                        }
                                    }
                                }
                                
                                if(angular_ > M_PI / 4.0 || angular_ < -M_PI / 4.0)
                                {
                                    linear_ = 0.0;
                                }
                                else
                                {
                                    if(distance_sub < 0.5)      linear_ = PLAYBACK_SPEED * 0.5;
                                    else if(distance_sub < 1)   linear_ = PLAYBACK_SPEED * 0.7;
                                    else                        linear_ = PLAYBACK_SPEED;
                                }
                            }
                        // }
                        // if (g_linear_data[g_cmdvel_index] == 0 && g_angular_data[g_cmdvel_index] == 0)
                        // {
                        //     g_cmdvel_index++;
                        // }
                        // if(g_linear_data[g_cmdvel_index] == 0 && g_angular_data[g_cmdvel_index] != 0) // else // g_linear_data[g_cmdvel_index] == 0 && g_angular_data[g_cmdvel_index] != 0
                        // {
                        //     dt_spin_end = 0;
                        //     linear_ = 0;
                        //     int i;
                            
                        //     if(distance_sub < STOP_DISTANCE)
                        //     {
                        //         odom_z_save = odom_z;
                        //     }
                        //     while (sqrt(pow(g_pos_x[g_cmdvel_index + g_spin_index] - (double)odom_x, 2)
                        //     + pow(g_pos_y[g_cmdvel_index + g_spin_index] - (double)odom_y, 2)) < 0.1 )
                        //     // && g_linear_data[g_cmdvel_index + g_spin_index] == 0
                        //     {
                        //         g_spin_index++;
                        //         dt_spin_start = dt;
                        //         std::cout << "while\n";
                        //     }
                        //     i = g_cmdvel_index + g_spin_index;
                        //     if(g_rot_z[i] - odom_z_save >= M_PI)
                        //     {
                        //         angular_ = -PLAYBACK_SPEED;
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //                 dt++;
                        //             }
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //         }
                        //     }
                        //     else if (g_rot_z[i] - odom_z_save < -M_PI)
                        //     {
                        //         angular_ = +PLAYBACK_SPEED;
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }                            int i;
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //         }
                        //     }
                        //     else if(g_rot_z[i] - odom_z_save >= 0 && g_rot_z[i] - odom_z_save < M_PI)
                        //     {
                        //         angular_ = PLAYBACK_SPEED;
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //         }
                        //     }
                        //     else if(g_rot_z[i] - odom_z_save >= -M_PI && g_rot_z[i] - odom_z_save < 0)
                        //     {
                        //         angular_ = -PLAYBACK_SPEED;
                        //         if(g_rot_z[i] >= 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //         }
                        //         else if (g_rot_z[i] < 0)
                        //         {
                        //             if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //             {
                        //                 g_cmdvel_index = i;
                        //                 g_spin_index = 0;
                        //                 angular_ = 0;
                        //             }
                        //         }
                        //     }
                        //     else
                        //     {
                        //         angular_ = -PLAYBACK_SPEED;
                        //         if (abs(odom_z - g_rot_z[i]) <= M_PI/100)
                        //         {
                        //             g_cmdvel_index = i;
                        //             g_spin_index = 0;
                        //             angular_ = 0;
                        //         }
                        //     }
                        //     if(angular_ == 0)
                        //     {
                        //         play_re_index++;
                        //         if(play_re_index > 1)
                        //         {
                        //             g_cmdvel_index++;
                        //             play_re_index = 0;
                        //         }
                        //     }
                        //     if(dt - dt_spin_start < 20)
                        //     {
                        //         dt++;
                        //         angular_ = 0;
                        //         std::cout << "delay for spin" << std::endl;
                        //     }
                        //     // if(distance_sub < STOP_DISTANCE)
                        //     // {
                        //     //     g_spin_index++;
                        //     //     odom_z_save = odom_z;
                        //     //     std::cout << "spin index " << g_spin_index << std::endl;
                        //     // }
                        //     // else if(distance_sub >= STOP_DISTANCE)
                        //     // {
                        //     //     std::cout << "odom_z_save: " << odom_z_save << " g_rot_z[g_cmdvel_index - g_spin_index] " << g_rot_z[g_cmdvel_index - g_spin_index] << std::endl;
                        //     //     if(odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] > 2*M_PI)
                        //     //     {
                        //     //         spin_angle = odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] - M_PI;
                        //     //     }
                        //     //     else if (odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] < -2*M_PI)
                        //     //     {
                        //     //         spin_angle = odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index] + M_PI;
                        //     //     }
                        //     //     else
                        //     //     {
                        //     //         spin_angle = odom_z_save - g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //     }
                        //     //     std::cout << "spin angle: " << spin_angle << std::endl;
                        //     //     if(spin_angle < 0) // && spin_angle > -M_PI
                        //     //     {
                        //     //         angular_ = -PLAYBACK_SPEED;
                        //     //         // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //         if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] + M_PI)) < M_PI/100)
                        //     //         {
                        //     //             // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //             g_spin_index = 0;
                        //     //         }
                        //     //     }
                        //     //     // else if (spin_angle < -M_PI)
                        //     //     // {
                        //     //     //     angular_ = PLAYBACK_SPEED;
                        //     //     //     // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //     //     if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] + M_PI)) < M_PI/100)
                        //     //     //     {
                        //     //     //         // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //     //         g_spin_index = 0;
                        //     //     //     }
                        //     //     // }
                        //     //     else if (spin_angle > 0) // && spin_angle < M_PI
                        //     //     {
                        //     //         angular_ = PLAYBACK_SPEED;
                        //     //         // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //         if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] - M_PI)) <= M_PI/100)
                        //     //         {
                        //     //             // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //             g_spin_index = 0;
                        //     //         }
                        //     //     }
                        //     //     // else if (spin_angle > M_PI)
                        //     //     // {
                        //     //     //     angular_ = -PLAYBACK_SPEED;
                        //     //     //     // angular_ = -g_rot_z[g_cmdvel_index - g_spin_index];
                        //     //     //     if (abs(odom_z - (g_rot_z[g_cmdvel_index - g_spin_index] - M_PI)) <= M_PI/100)
                        //     //     //     {
                        //     //     //         // g_cmdvel_index = g_cmdvel_index - g_spin_index;
                        //     //     //         g_spin_index = 0;
                        //     //     //     }
                        //     //     // }
                        //     //     else
                        //     //     {
                        //     //         std::cout << "else spin angle\n";
                        //     //     }
                        //     // }
                        // }
                    }
                }
                else
                {
                    linear_ = 0.0;
                    angular_ = 0.0;
                    button_state[0] = 0;
                    button_state[1] = 0;
                    button_state[2] = 0;
                    ledcomm.led[2] = 0;
                    g_cmdvel_index = 0;
                    // if(g_cmdvel_index != 0)
                    // {
                    //     g_cmdvel_index--;
                    // }
                }
            }

            // std::cout << "replay cmdvel indexd: " << g_cmdvel_index << " distance sub: " << distance_sub << std::endl;

            twist.angular.z = angular_;
            twist.linear.x = linear_;
            ledcomm.led[0] = 0;
            ledcomm.led[1] = 0;
            is_motor_coast.iscoast = remote_controller;
            g_target_detecting_state = 0;
            determinant_value.determinant = 1;
        }
        else if (coast_state == 1)
        {
            determinant_value.determinant = 2;
        }
        else // buttons overlapping
        {
            linear_ = 0.0;
            angular_ = 0.0;
            remote_controller = true;
            button_state[0] = 0;
            button_state[1] = 0;
            button_state[2] = 0;

            twist.angular.z = angular_;
            twist.linear.x = linear_;
            ledcomm.led[0] = 0;
            ledcomm.led[1] = 0;
            ledcomm.led[2] = 0;
            is_motor_coast.iscoast = remote_controller;
            g_target_lost = 0;
            g_target_detect = 0;
            g_target_detecting_state = 0;
            determinant_value.determinant = 0;
        }
    }

    // firmware command (esp32 write control) : 1 = firmware communication on, 0 = firmware communication off
    ledcomm.firmwarecommand = 1;
    // ros2 driver transmit control : 1 = communication on, 0 = communication off
    ledcomm.transmitcommand = 1;

    if(angular_ > M_PI_4)
    {
        angular_ = M_PI_4;
    }
    else if(angular_ < -M_PI_4)
    {
        angular_ = -M_PI_4;
    }

    // determinant_value.modenum = 1;
    // std::cout << "mdvel indexd: " << g_cmdvel_index << " angular: " << angular_ 
    // << "buttons" << button_state[0] << button_state[1] << button_state[2] << " truefalse_playback " << truefalse_playback << "" << std::endl;

    if(g_rot_z.empty() != 1)
        std::cout << "index: " << g_cmdvel_index <<" odom z save" << odom_z_save << " g_rot_z[g_cmdvel_index] " << g_rot_z[g_cmdvel_index] << std::endl;

    cmd_vel_pub->publish(twist);
    controller_pub->publish(is_motor_coast);
    led_pub->publish(ledcomm);
    following_state_pub->publish(determinant_value);

    mapOutPub(target_obj_info_vector, target_id+1);
    footprintMarker();
    pubMarker(target_obj_info_vector);
    revMakeRefMap();
}

void clusteringNode::makeRefMap(void)
{
    // thread
    for (size_t i = 0; i < filtered_lidar_cloud->points.size(); i++)
    {
        map_cloud->points.push_back(filtered_lidar_cloud->points[i]);
    }
    map_cloud->width = map_cloud->points.size();
    map_cloud->height = 1;
    map_cloud->is_dense = true;
}

void clusteringNode::revMakeRefMap(void)
{
    for(size_t i = 0; i < filtered_lidar_cloud->points.size(); i++)
    {
        map_cloud->points.clear();
    }
    map_cloud->width = map_cloud->points.size();
    map_cloud->height = -1;
    map_cloud->is_dense = false;
}

void clusteringNode::mapDownsampling(void)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_lidar_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*map_cloud);
}

void clusteringNode::filteredEuclideanClustering(std::vector<ObjInfo>& info)
{
    ObjInfo filltered_clusterinfo;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    if(!filtered_lidar_cloud->empty())
        tree->setInputCloud (filtered_lidar_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (0.05);
    ec.setMinClusterSize (2);
    ec.setMaxClusterSize (50);
    ec.setSearchMethod (tree);
    ec.setInputCloud (filtered_lidar_cloud);
    ec.extract (cluster_indices);

    int cluster_index = 0;

    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices)
        {
            cloud_cluster->push_back((*filtered_lidar_cloud)[idx]);
        }

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        Eigen::Vector4f centroid;
        pcl::PointXYZ min_point, max_point;

        pcl::compute3DCentroid(*cloud_cluster, centroid);       // cluster center position
        pcl::getMinMax3D(*cloud_cluster, min_point, max_point); // cluster size

        filltered_clusterinfo.id =  cluster_index + 1;
        filltered_clusterinfo.width = max_point.y - min_point.y;
        filltered_clusterinfo.height = max_point.x - min_point.x;
        filltered_clusterinfo.position[0] = (max_point.x + min_point.x)/2;
        filltered_clusterinfo.position[1] = (max_point.y + min_point.y)/2;

        cluster_index++;

        info.push_back(filltered_clusterinfo);
    }
}

void clusteringNode::euclideanClustering(std::vector<ObjInfo>& info)
{
    ObjInfo clusterinfo;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    if(!map_cloud->empty())
        tree->setInputCloud (map_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (0.05);
    ec.setMinClusterSize (2);
    ec.setMaxClusterSize (50);
    ec.setSearchMethod (tree);
    ec.setInputCloud (map_cloud);
    ec.extract (cluster_indices);

    int cluster_index = 0;

    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices)
        {
            cloud_cluster->push_back((*map_cloud)[idx]);
        }

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        Eigen::Vector4f centroid;
        pcl::PointXYZ min_point, max_point;

        pcl::compute3DCentroid(*cloud_cluster, centroid);       // cluster center position
        pcl::getMinMax3D(*cloud_cluster, min_point, max_point); // cluster size

        clusterinfo.id =  cluster_index + 1;
        clusterinfo.width = max_point.y - min_point.y;
        clusterinfo.height = max_point.x - min_point.x;
        clusterinfo.position[0] = (max_point.x + min_point.x)/2;
        clusterinfo.position[1] = (max_point.y + min_point.y)/2;

        cluster_index++;
        info.push_back(clusterinfo);
    }
}

int clusteringNode::targetDetect(std::vector<ObjInfo> obj_info)
{
    double distance, min_distance = 0.5;
    int target_index = -1;
    int detect = -1;

    for(size_t i = 0; i < obj_info.size(); i++)
    {
        distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0] + obj_info[i].position[1] * obj_info[i].position[1]);

        if(min_distance > distance)
        {
            min_distance = distance;
            target_index = i;
        }
    }

    if(target_index >= 0)
    {
        detect = target_index;
        roi_center_x = obj_info[target_index].position[0];
        roi_center_y = obj_info[target_index].position[1];
    }
    else
    {
        detect = -1;
    }

    return detect;
}

int clusteringNode::roiDetect(std::vector<ObjInfo> obj_info)
{
    int target_lost = -1;

    for(size_t i = 0; i < obj_info.size(); i++)
    {
        if( sqrt(pow(obj_info[i].position[0] - roi_center_x, 2) + pow(obj_info[i].position[1] - roi_center_y, 2)) < ROI_RANGE
        && obj_info[i].position[0] != 0 && obj_info[i].position[1] != 0)
        {
            roi_center_x = obj_info[i].position[0];
            roi_center_y = obj_info[i].position[1];
            target_lost = i;

            return target_lost;
        }
        else
        {
            target_lost = -1;
        }
    }

    return target_lost;
}

int clusteringNode::collisionAvoidance(std::vector<ObjInfo> obj_info)
{
    int avoidance = 0;
    float distance, collision_distance = COLLISION_DISTANCE;

    for(size_t i = 0; i < obj_info.size(); i++)
    {
        distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0]
        + obj_info[i].position[1] * obj_info[i].position[1]);

        if(distance < collision_distance)
        {
            collision_distance = distance;
        }
    }

    if(collision_distance < COLLISION_DISTANCE || (min_laser > 0.05 && min_laser < 0.3)) avoidance = 1; //
    else avoidance = 0;

    return avoidance; // 1 means collision could happen by any other objects, 0 is good to move
}

void clusteringNode::mapOutPub(std::vector<ObjInfo> obj_info_vector, int id)
{
    sensor_msgs::msg::PointCloud out_msg;
    out_msg.header.frame_id = "base_scan";

    // thread
    if(id <= 0)
    {
        for (size_t i = 0; i < map_cloud->points.size(); i++)
        {
            geometry_msgs::msg::Point32 single_point;
            single_point.x = map_cloud->points[i].x;
            single_point.y = map_cloud->points[i].y;

            out_msg.points.push_back(single_point);
        }
    }
    else
    {
        for (size_t i = 0; i < map_cloud->points.size(); i++)
        {
            geometry_msgs::msg::Point32 single_point;
            single_point.x = map_cloud->points[i].x;
            single_point.y = map_cloud->points[i].y;

            if(single_point.x < obj_info_vector[id-1].position[0] - 0.3
            || single_point.x > obj_info_vector[id-1].position[0] + 0.3
            || single_point.y < obj_info_vector[id-1].position[1] - 0.3
            || single_point.y > obj_info_vector[id-1].position[1] + 0.3)
            {
                out_msg.points.push_back(single_point);
            }
            else
            {
                single_point.x = 0;
                single_point.y = 0;
            }
        }
    }

    if (out_msg.points.size() > 0)
    {
        sensor_msgs::msg::PointCloud2 out_msg2;
        sensor_msgs::convertPointCloudToPointCloud2(out_msg, out_msg2);
        out_msg2.header.stamp = now();
        map_out_pub->publish(out_msg2);
    }
}

void clusteringNode::footprintMarker()
{
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "base_scan";
    marker_msg.header.stamp = rclcpp::Clock().now();
    marker_msg.ns = "sphere";
    marker_msg.id = 0;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_array_msg.markers.push_back(marker_msg);
    footprint_pub->publish(marker_array_msg);
    marker_array_msg.markers.clear();

    for (size_t i = 1; i <= g_cmdvel_index; i++)
    {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "map";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "footprint";
        marker_msg.id = i;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = g_pos_x[i];
        marker_msg.pose.position.y = g_pos_y[i];
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.1;
        marker_msg.scale.y = 0.1;
        marker_msg.scale.z = 0.1;
        marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;
        marker_array_msg.markers.push_back(marker_msg);
    }

    for (size_t i = 0; i <= g_cmdvel_index; i++)
    {
        visualization_msgs::msg::Marker text_msg;
        text_msg.header.frame_id = "map";
        text_msg.header.stamp = rclcpp::Clock().now();
        text_msg.ns = "text";
        text_msg.id = i;
        text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_msg.action = visualization_msgs::msg::Marker::ADD;
        text_msg.pose.position.x = g_pos_x[i] + 0.2;
        text_msg.pose.position.y = g_pos_y[i] + 0.2;
        text_msg.scale.z = 0.2;
        text_msg.color.a = 1.0; // Don't forget to set the alpha!
        text_msg.color.r = 0.0;
        text_msg.color.g = 1.0;
        text_msg.color.b = 0.0;

        std::string text_string = "[" + std::to_string(i) + "]";
        text_msg.text = text_string;

        marker_array_msg.markers.push_back(text_msg);
    }

    footprint_pub->publish(marker_array_msg);
}

void clusteringNode::pubMarker(std::vector<ObjInfo> obj_info)
{
    double distance;

    //////////////////////////////////////////////////////////////
    // clear markers
    //////////////////////////////////////////////////////////////
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "base_scan";
    marker_msg.header.stamp = rclcpp::Clock().now();
    marker_msg.ns = "obj";
    marker_msg.id = 0;
    marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_msg.markers.push_back(marker_msg);
    rviz_pub->publish(marker_array_msg);
    marker_array_msg.markers.clear();

    //////////////////////////////////////////////////////////////
    // publish markers
    //////////////////////////////////////////////////////////////
    if (obj_info.size() == 0)
        return;

    for (size_t i = 0; i < obj_info.size(); i++)
    {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "base_scan";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "obj";
        marker_msg.id = obj_info[i].id;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = obj_info[i].position[0];
        marker_msg.pose.position.y = obj_info[i].position[1];
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.3;
        marker_msg.scale.y = 0.3;
        marker_msg.scale.z = 0.3;
        marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        marker_msg.color.r = 1.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;
        marker_array_msg.markers.push_back(marker_msg);
    }

    for (size_t i = 0; i < g_cmdvel_index; i++)
    {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "map"; //odom
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "footprint";
        marker_msg.id = i;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = g_pos_x[i];
        marker_msg.pose.position.y = g_pos_y[i];
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.1;
        marker_msg.scale.y = 0.1;
        marker_msg.scale.z = 0.1;
        marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;
        marker_array_msg.markers.push_back(marker_msg);
    }

    visualization_msgs::msg::Marker box_msg_roi;
    box_msg_roi.header.frame_id = "base_scan";
    box_msg_roi.header.stamp = rclcpp::Clock().now();
    box_msg_roi.ns = "roi";
    //box_msg_roi.id = obj_info_vector[i].id;
    box_msg_roi.type = visualization_msgs::msg::Marker::LINE_STRIP;
    box_msg_roi.action = visualization_msgs::msg::Marker::ADD;
    box_msg_roi.pose.orientation.w = 1.0;
    box_msg_roi.scale.x = 0.05;
    box_msg_roi.color.a = 1.0; // Don't forget to set the alpha!
    box_msg_roi.color.r = 0.0;
    box_msg_roi.color.g = 1.0;
    box_msg_roi.color.b = 1.0;

    geometry_msgs::msg::Point box_point_msg;

    box_point_msg.x = roi_center_x - ROI_WIDTH;
    box_point_msg.y = roi_center_y - ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);

    box_point_msg.x = roi_center_x + ROI_WIDTH;
    box_point_msg.y = roi_center_y - ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);

    box_point_msg.x = roi_center_x + ROI_WIDTH;
    box_point_msg.y = roi_center_y + ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);

    box_point_msg.x = roi_center_x - ROI_WIDTH;
    box_point_msg.y = roi_center_y + ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);

    box_point_msg.x = roi_center_x - ROI_WIDTH;
    box_point_msg.y = roi_center_y - ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);

    marker_array_msg.markers.push_back(box_msg_roi);

    //////////////////////////////////////////////////////////////
    // Text marker
    //////////////////////////////////////////////////////////////
    for (size_t i = 0; i < obj_info.size(); i++)
    {
        visualization_msgs::msg::Marker text_msg;
        text_msg.header.frame_id = "base_scan";
        text_msg.header.stamp = rclcpp::Clock().now();
        text_msg.ns = "text";
        text_msg.id = obj_info[i].id;
        text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_msg.action = visualization_msgs::msg::Marker::ADD;
        text_msg.pose.position.x = obj_info[i].position[0] - obj_info[i].width/2;
        text_msg.pose.position.y = obj_info[i].position[1] + obj_info[i].height/2 + 0.2;
        text_msg.scale.z = 0.2;
        text_msg.color.a = 1.0; // Don't forget to set the alpha!
        text_msg.color.r = 0.0;
        text_msg.color.g = 1.0;
        text_msg.color.b = 0.0;

        distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0] + obj_info[i].position[1] * obj_info[i].position[1]);

        std::string text_string = "[" + std::to_string(obj_info[i].id) + "]";
        text_msg.text = text_string;

        marker_array_msg.markers.push_back(text_msg);
    }
    rviz_pub->publish(marker_array_msg);
}

void followingSoundPlay(void) // (const char* filename)
{
    sf::SoundBuffer go_buffer, stop_buffer, shutdown_buffer;

    if (!go_buffer.loadFromFile("/home/robot/robot_ws/audio/Bolero.wav") || !stop_buffer.loadFromFile("/home/robot/robot_ws/audio/stopbeep.wav")
    || !shutdown_buffer.loadFromFile("/home/robot/robot_ws/audio/Turning_Off.wav")) // ~/Desktop/test/audio/beep.wav
    {
        std::cerr << "load failed!";
    }

    sf::Sound go_sound, stop_sound, shutdown_sound;

    go_sound.setBuffer(go_buffer);
    stop_sound.setBuffer(stop_buffer);
    shutdown_sound.setBuffer(shutdown_buffer);

    while(sound_play_thread_controller)
    {
        if(shutdown == 1) //&& shutdown_sound.getStatus() != sf::Sound::Playing
        {
            // if(shutdown_sound.getStatus() == 0)
            //     shutdown_sound.play();

            // std::cout << "play shutdown_sound.getStatus() " << shutdown_sound.getStatus() << std::endl;

            // while (shutdown_sound.getStatus() == 2)
            // {
            //     std::cout << "while shutdown_sound.getStatus() " << shutdown_sound.getStatus() << std::endl;

            //     if (shutdown_sound.getStatus() == 1)
            //     {
            //         std::cout << "if shutdown_sound.getStatus() " << shutdown_sound.getStatus() << std::endl;
            //         sync();
            //         reboot(LINUX_REBOOT_CMD_POWER_OFF);
            //         return;
            //     }
            // }
            // std::cout << "shutdown " << shutdown << std::endl;

            shutdown_sound.play();

            while (shutdown_sound.getStatus() == sf::Sound::Playing)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            system("sudo /sbin/shutdown -h now");
            // sync();

        }

        if(g_target_detecting_state != before_state)
        {
            if(g_target_detecting_state == 0)
            {
                go_sound.stop();
                stop_sound.stop();
            }
            else if(g_target_detecting_state == 1)
            {
                stop_sound.stop();
                go_sound.play();
            }
            else if(g_target_detecting_state == 2)
            {
                go_sound.stop();
                stop_sound.play();
            }
        }
        else
        {
            if(g_target_detecting_state == 0)
            {
                go_sound.stop();
                stop_sound.stop();
            }
            else if (g_target_detecting_state == 1 && go_sound.getStatus() != sf::Sound::Playing)
            {
                go_sound.play();
                stop_sound.stop();
            }
            else if (g_target_detecting_state == 2 && stop_sound.getStatus() != sf::Sound::Playing)
            {
                stop_sound.play();
                go_sound.stop();
            }
        }

        before_state = g_target_detecting_state;
    }
}

int main(int nArgc,const char* pszArgv[])
{
    sf::SoundBuffer booting_buffer;

    if (!booting_buffer.loadFromFile("/home/robot/robot_ws/audio/booting.wav")) // ~/Desktop/test/audio/beep.wav
    {
        std::cerr << "load failed!";
    }

    sf::Sound booting_sound;

    booting_sound.setBuffer(booting_buffer);

    booting_sound.play();

    rclcpp::init(nArgc, pszArgv);
    // rclcpp::spin(std::make_shared<clusteringNode>());

    auto node = std::make_shared<clusteringNode>();

    std::thread ros_thread([&](){
        rclcpp::spin(node);
    });
    std::thread play_thread(followingSoundPlay);

    if(play_thread.joinable())
        play_thread.detach();
    if(ros_thread.joinable())
        ros_thread.join();

    sound_play_thread_controller = 0;
    rclcpp::shutdown();

    return 0;
}
