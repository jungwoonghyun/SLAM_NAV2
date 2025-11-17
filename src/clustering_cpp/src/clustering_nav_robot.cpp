#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <ctime>
#include <thread>
#include <tuple>
#include <condition_variable>
#include <mutex>
#include <atomic>
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
#include <Eigen/Dense>
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
#include "message_interface/msg/navcancel.hpp"

using namespace std::chrono_literals;

#define ROI_WIDTH                   0.3
#define ROI_HEIGHT                  0.3
#define ROI_RANGE                   0.3
#define TARGET_EXCLUSIVE            0.4
#define COLLISION_DISTANCE          0.6
#define POINT_DISTANCE              0.3
#define ULTRASONIC_LIMIT_DISTANCE   500    // 500
#define STOP_DISTANCE               0.1
#define OBSTACLE_ANGLE_BOUNDARY     -90
#define SPIN_COUNTER                50

int g_target_detecting_state = 0;
int g_target_before_state = 0;
int g_sound_play_thread_controller = 1;
int g_shutdown = 0;

struct ObjInfo
{
    int id;
    std::vector<double> position{0.0,0.0};
    std::vector<double> leftbottom{0.0,0.0};
    std::vector<double> righttop{0.0,0.0};
    double width;
    double height;
    double angle;
};

class clusteringNode : public rclcpp::Node
{
  public:
    clusteringNode()
    : Node("clustering_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Qos depth = 10
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        this->declare_parameter<int>("max_iterations", 100);
        this->declare_parameter<double>("cluster_tolerance", 0.2);
        this->declare_parameter<int>("min_cluster_size", 2);
        this->declare_parameter<int>("max_cluster_size", 100);

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
        rclcpp::SensorDataQoS(), std::bind(&clusteringNode::msg_callback_lidar, this, std::placeholders::_1));
        // joystick_sub = this->create_subscription<message_interface::msg::Buttonstate>("joystick_status",
        // qos_profile, std::bind(&clusteringNode::msg_callback_joystick, this, std::placeholders::_1));
        button_sub_ = this->create_subscription<message_interface::msg::Buttonstate>("button_status",
        10, std::bind(&clusteringNode::msg_callback_button, this, std::placeholders::_1)); //
        ultrasonic_sub_ = this->create_subscription<message_interface::msg::Ulsonicdist>("ultrasonic_distance",
        qos_profile, std::bind(&clusteringNode::msg_callback_ultrasonic, this, std::placeholders::_1));
        odom_data_sub_ = this->create_subscription<message_interface::msg::Odomdata>("foot_data",
        qos_profile, std::bind(&clusteringNode::msg_callback_odom, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
        map_out_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("out_cloud", qos_profile);
        rviz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obj_detect_marker_array", 10);
        footprint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("base_footprint_marker_array", 10);
        led_pub_ = this->create_publisher<message_interface::msg::Ledcomm>("led_instructor", 10);
        controller_pub_ = this->create_publisher<message_interface::msg::Iscoast>("is_coast", 10);
        following_state_pub_ = this->create_publisher<message_interface::msg::Mode>("following_state", qos_profile);
        point_pub_ = this->create_publisher<message_interface::msg::Odomdata>("point_data", 10);
        nav_cancel_pub_ = this->create_publisher<message_interface::msg::Navcancel>("cancel_signal", qos_profile);

        timer_ = this->create_wall_timer(50ms, std::bind(&clusteringNode::timer_callback, this));
    }

    ~clusteringNode() {
    }

    int target_id_ = -2, obstacle_handle_mode_, avoidance_num_;
    double roi_center_x_ = 0.5, roi_center_y_ = 0.0;
    int frontdist_ = 0, backdist_ = 0;
    int move_bef_state_[3] = {0}, point_bef_state_ = 0, point_del_bef_state_ = 0; // move_bef_state_: buttons about follow, playback, replay
    int coast_state_ = 0, point_state_ = 0, point_del_state_ = 0;
    int joystick_state_[4] = {0};
    int button_state_[3] = {0};
    int shutdown_input_ = 0;
    int blink_ = 0;
    int target_lost_ = 0, target_detect_ = 0;
    bool remote_controller_ = false;
    double bef_angle_comm = 0;
    std::vector<double> point_pos_x_, point_pos_y_, point_rot_z_;
    float odom_x_, odom_y_, odom_z_;
    int cmdvel_index_ = 0;
    int end_point = 0;
    int trace_cnt_ = 0, spin_cnt_ = 0; // dt = 0,
    bool truefalse_playback_ = true, truefalse_replay_ = true; // , reset_data = true
    int cancel_state_ = -1; // if button state changed 'nav_cancel_data.nav_cancel' must send true one time to cancel.
    int play_back_point_pub_cnt_ = 1, replay_point_pub_cnt_ = 1; // 
    int play_back_control_ = 1, replay_control_ = 1;
    bool collision_back_ = true, collision_spin_ = false, cancel_ = true;
    bool nav_spin_ = true, odom_z_save_ = true;
    int nav_spin_cnt_ = 0;
    double odom_z_spin_save_val_, relative_x_=-1;
    bool follow_init_ = true;
    double obstacle_angle_;
    int avoidance_cnt_ = 0;
    int map_obstacle_idx_ = -1;
    message_interface::msg::Ledcomm ledcomm; // led is button & lgith is beacon

    private:

        std::chrono::steady_clock::time_point start_time;

        // variable
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_filtered{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // pcl::ExtractIndices<pcl::PointXYZ> extract;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        // rclcpp::Subscription<message_interface::msg::Buttonstate>::SharedPtr joystick_sub;
        rclcpp::Subscription<message_interface::msg::Buttonstate>::SharedPtr button_sub_;
        rclcpp::Subscription<message_interface::msg::Ulsonicdist>::SharedPtr ultrasonic_sub_;
        rclcpp::Subscription<message_interface::msg::Odomdata>::SharedPtr odom_data_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_out_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_pub_;
        rclcpp::Publisher<message_interface::msg::Ledcomm>::SharedPtr led_pub_;
        rclcpp::Publisher<message_interface::msg::Iscoast>::SharedPtr controller_pub_;
        rclcpp::Publisher<message_interface::msg::Mode>::SharedPtr following_state_pub_;
        rclcpp::Publisher<message_interface::msg::Odomdata>::SharedPtr point_pub_;
        rclcpp::Publisher<message_interface::msg::Navcancel>::SharedPtr nav_cancel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // function
        void msg_callback_lidar(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        // void msg_callback_joystick(const message_interface::msg::Buttonstate & msg);
        void msg_callback_button(const message_interface::msg::Buttonstate & msg);
        void msg_callback_ultrasonic(const message_interface::msg::Ulsonicdist msg);
        void msg_callback_odom(const message_interface::msg::Odomdata msg);
        void timer_callback();

        void make_ref_map(void);
        void rev_make_ref_map(void);
        void map_downsampling(void);
        void filtered_euclidean_clustering(std::vector<ObjInfo>& info);
        void euclidean_clustering(std::vector<ObjInfo>& info);
        int target_detect(std::vector<ObjInfo> obj_info);
        int roi_detect(std::vector<ObjInfo> obj_info);
        int following_collision_avoidance(std::vector<ObjInfo> obj_info);
        std::tuple<int, int> ptop_collision_avoidance(std::vector<ObjInfo> obj_info);
        void point_move(int ptop_collision, int obstacle_idx, std::vector<ObjInfo> collision_obj_info_vector);
        void map_out_pub(std::vector<ObjInfo> obj_info_vector, int id);
        void pub_marker(std::vector<ObjInfo> obj_info);
        void footprint_marker(std::vector<ObjInfo> obj_info, int map_obstacle_idx_);
};

void clusteringNode::msg_callback_lidar(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    sensor_msgs::msg::LaserScan scan_msg = *msg;
    filtered_lidar_cloud->points.clear();
    lidar_cloud->points.clear();

    //thread
    for (size_t i = 0; i < scan_msg.ranges.size(); i++) {
        double angle = (scan_msg.angle_increment) * i + scan_msg.angle_min;
        double range = scan_msg.ranges[i];

        if(std::isnan(range)) { //|| (range > ROI_RANGE && range < ROI_RANGE + TARGET_EXCLUSIVE)
            range = 0.0;
        }

        double lidar_x = range * cos(angle);
        double lidar_y = range * sin(angle);

        if(sqrt(pow(roi_center_x_ - lidar_x, 2) + pow(roi_center_y_ - lidar_y, 2)) < ROI_RANGE
        || sqrt(pow(roi_center_x_ - lidar_x, 2) + pow(roi_center_y_ - lidar_y, 2)) > ROI_RANGE + TARGET_EXCLUSIVE
        || sqrt(pow(lidar_x, 2) + pow(lidar_y, 2)) > 0.05) {
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

// void clusteringNode::msg_callback_joystick(const message_interface::msg::Buttonstate & msg) // joystick can be pushed one by one
// {
//     for(int i = 0; i < 4; i++) {
//         joystick_state_[i] = msg.button[i];
//     }
// }

void clusteringNode::msg_callback_button(const message_interface::msg::Buttonstate & msg)
{
    geometry_msgs::msg::Twist twist;

    int io_input = 0, point_input = 0;

    joystick_state_[0] = msg.button[0]; // angular
    joystick_state_[1] = msg.button[1]; // linear
    

    for(int i=0; i < 3; i++) {
        io_input = io_input + msg.button[i+4];
    }

    for(int i = 0; i < 2; i++) {
        point_input = point_input + msg.button[i+8];
    }
    
    // following
    if(io_input < 2) { // maximum one button clicked
        if(msg.button[4] == 1) {
            if(move_bef_state_[0] != msg.button[4]) {
                if(button_state_[0] == 1) {
                    end_point++;
                }
                button_state_[0]++;
                if(button_state_[0] > 1) { // button_state_[0]: 0 stop, 1 following
                    button_state_[0] = 0;
                }
                move_bef_state_[0] = msg.button[4];
            }
        } else {
            move_bef_state_[0] = msg.button[4];
        }
    } else { // more than one button clicked
        move_bef_state_[0] = 0;
        button_state_[0] = 0;
    }

    // playback & reverse playback
    for(int i = 1; i < 3; i++) { // io board
        if(io_input < 2) { // maximum one button clicked
            if(msg.button[i+4] == 1) {
                if(move_bef_state_[i] != msg.button[i+4]) {
                    button_state_[i]++;
                    if(button_state_[i] > 2) { // button_state_[1]: 0 stop, 1 blink, 2 playback
                                              // button_state_[2]: 0 stop, 1 blink, 2 reverse playback
                        button_state_[i] = 0;
                    }
                    move_bef_state_[i] = msg.button[i+4];

                    if(button_state_[1] == 2 && msg.button[5] == 1) {
                        cmdvel_index_--;
                        if(cmdvel_index_ < 1) {
                            cmdvel_index_ = 1;
                        }
                    } else if (button_state_[2] == 2 && msg.button[6] == 1) {
                        cmdvel_index_++;
                        if(cmdvel_index_ > (point_pos_x_.size() - 1)) {
                            cmdvel_index_ = (point_pos_x_.size() - 1);
                        }
                    }
                }
            } else {
                move_bef_state_[i] = msg.button[i+4];
            }
        } else { // more than one button clicked
            move_bef_state_[i] = 0;
            button_state_[i] = 0;
        }
    }

    io_input = 0;

    for(int i=0; i < 3; i++) {
        if(button_state_[i] > 0) {
            io_input++;
        }
    }

    if(io_input>=2) {
        move_bef_state_[0] = 0;
        move_bef_state_[1] = 0;
        move_bef_state_[2] = 0;
        button_state_[0] = 0;
        button_state_[1] = 0;
        button_state_[2] = 0;
    }

    if(msg.button[7] == 0) {
       shutdown_input_ = 0;
    } else {
        shutdown_input_++;
    }

    if(shutdown_input_ >= 40) { // shutdown button
        twist.angular.z = 0;
        twist.linear.x = 0;

        cmd_vel_pub_->publish(twist);

        g_shutdown = 1;
    }

    // if(point_input < 2)
    // {
        if(msg.button[9] == 1) { // msg.button[8] => msg.button[9] 
            if(point_bef_state_ != msg.button[9]) {
                point_state_++;
                if(point_state_ > 1) {
                    point_state_ = 0;
                }
                point_bef_state_ = msg.button[9]; // coast button
            }
        } else {
            point_bef_state_ = msg.button[9];
            point_state_ = 0;
        }
    // }
    // else
    // {
    //     point_bef_state_ = 0;
    //     point_state_ = 0;
    // }

    // if(msg.button[9] == 1)
    // {
    //     std::cout << "point down\n";
    //     if(point_del_bef_state_ != msg.button[9])
    //     {
    //         point_del_state_++;
    //         if(point_del_state_ > 1)
    //         {
    //             point_del_state_ = 0;
    //         }
    //         point_del_bef_state_ = msg.button[9]; // coast button
    //     }
    // }
    // else
    // {
    //     point_del_bef_state_ = msg.button[8];
    //     point_del_state_ = 0;
    // }
}

void clusteringNode::msg_callback_ultrasonic(const message_interface::msg::Ulsonicdist msg)
{
    // ultrasonic unit is mm
    frontdist_ = msg.frontdist;
    backdist_ = msg.backdist;
}

void clusteringNode::msg_callback_odom(const message_interface::msg::Odomdata msg)
{
    odom_x_ = msg.x_pos;
    odom_y_ = msg.y_pos;
    odom_z_ = msg.z_angle;
}

//////////////////////////////////////

void clusteringNode::timer_callback()
{
    double linear = 0.0, angular = 0.0;
    double target_distance = 0, target_angle = 0; // , target_vel = 0, robot_vel = 0, vel_dest = 0
    double velocity_comm = 0, bef_vel_comm = 0;
    int following_collision;
    double weight_linear = 0.9, weight_angular = 1;

    geometry_msgs::msg::Twist twist;
    message_interface::msg::Iscoast is_motor_coast;
    message_interface::msg::Ulsonicdist ultrasonic_distance_data;
    message_interface::msg::Mode determinant_value;
    message_interface::msg::Odomdata point_data;
    message_interface::msg::Navcancel nav_cancel_data;

    std::vector<ObjInfo> target_obj_info_vector;
    std::vector<ObjInfo> collision_obj_info_vector;

    if (filtered_lidar_cloud->points.size() == 0) {
        return;
    }

    make_ref_map();

    map_downsampling();

    filtered_euclidean_clustering(target_obj_info_vector);
    euclidean_clustering(collision_obj_info_vector); // make cluster and give info(like id, width...)
                                                    // to each cluster
    // target detecting or not
    if(target_id_ == -2) {
        target_id_ = target_detect(target_obj_info_vector); // target_id = -1 detect failed
        // std::cout << "Choose emergency stop case\n";
        // std::cout << "Avoidance mode: if sudden obstacle detected robot try to avoid\n";
        // std::cout << "Waiting mode: if sudden obstacle detected wait until it go away\n";
        // std::cout << "If you want Avoidance mode press 1, if you want Waiting mode press 2\n";
        // std::cin >> obstacle_handle_mode_;

        // if(obstacle_handle_mode_ == 1) {
        //     std::cout << "please enter the number that you want to avoid when obstacle suddenly pop out \n";
        //     std::cout << "when move point to point.\n";
        //     std::cout << "If you enter 0, robot will stop\n";
        //     std::cout << "If you enter over 3, robot will avoid infinitly\n";
        //     std::cout << "\nPlease input number : ";
        //     std::cin >> avoidance_num_;
        //     std::cout << "avoidance number: " << avoidance_num_ << "\n";
        
        //     if(avoidance_num_ < 0) {
        //         target_id_ = -2;
        //         std::cout << "\n\nInput value is wrong. Please input 0 or larger number\n\n";
        //     }
        // } else if (obstacle_handle_mode_ == 2) {
        //     std::cout << "Waiting mode\n";
        // } else {
        //     std::cout << "You enter wrong mode number please enter 1 or 2\n";
        //     target_id_ = -2;
        // }
        obstacle_handle_mode_ = 1;
        avoidance_num_ = 3;
        
        remote_controller_ = true;
    } else {
        target_id_ = roi_detect(target_obj_info_vector); // Get roi center position & get target id

        std::cout << "button 0 " << button_state_[0] << "button 1 " << button_state_[1] << "button 2 " << button_state_[2] << std::endl;

        if(button_state_[0] == 0 && button_state_[1] == 0 && button_state_[2] == 0) {// mode off
            roi_center_x_ = 0.5;
            roi_center_y_ = 0;
            target_lost_ = 0;
            target_detect_ = 0;
            // dt = 0;
            // reset_data = true;
            truefalse_playback_ = true;
            truefalse_replay_ = true;
            collision_back_ = true;
            avoidance_cnt_ = 0;
            nav_spin_cnt_ = 0;

            if(cancel_state_ != 0) {
                cancel_state_ = 0;
                nav_cancel_data.nav_cancel = true;
                nav_cancel_pub_->publish(nav_cancel_data);
            } else {
                nav_cancel_data.nav_cancel = false;
                nav_cancel_pub_->publish(nav_cancel_data);
            }

            if(coast_state_ != 1) { // joystick
                

                if(joystick_state_[1] > 155){
                    linear = (joystick_state_[1] - 155) * 0.0055;
                }
                else if(joystick_state_[1] < 100){
                    linear = (joystick_state_[1] - 100) * 0.004;
                }
                else{
                    linear = 0.0;
                }

                // angular
                if(joystick_state_[0] > 155){
                    angular = -(joystick_state_[0] - 155) * 0.004 ;
                }
                else if(joystick_state_[0] < 100){
                    angular = -(joystick_state_[0] - 100) * 0.004 ;
                }
                else{
                    angular = 0.0;
                }

                if(linear == 0 & angular == 0) {
                    ledcomm.light[0] = 0;
                    ledcomm.light[1] = 0;
                    ledcomm.light[2] = 0;
                } else {
                    ledcomm.light[0] = 1;
                    ledcomm.light[1] = 0;
                    ledcomm.light[2] = 0;
                }

                remote_controller_ = true;
                twist.linear.x = linear;
                twist.angular.z = angular;
                is_motor_coast.iscoast = remote_controller_;
                g_target_detecting_state = 0;
                determinant_value.determinant = 3;
            } else {
                remote_controller_ = true;
                twist.linear.x = linear;
                twist.angular.z = angular;
                ledcomm.led[0] = 0;
                ledcomm.led[1] = 0;
                ledcomm.led[2] = 0;
                ledcomm.light[0] = 0;
                ledcomm.light[1] = 0;
                ledcomm.light[2] = 0;
                is_motor_coast.iscoast = remote_controller_;
                g_target_detecting_state = 0;
                determinant_value.determinant = 2;
            }

            if (point_state_ == 1) {// mark point by button
                point_pos_x_.push_back(odom_x_);
                point_pos_y_.push_back(odom_y_);
                point_rot_z_.push_back(odom_z_);
                cmdvel_index_++;
                point_state_ = 0;
            }
            // else if (point_del_state_ == 1
            // && point_pos_x_[0] != 0 && point_pos_y_[0] != 0 && point_rot_z_[0] != 0)
            // {
            //     point_pos_x_.pop_back();
            //     point_pos_y_.pop_back();
            //     point_rot_z_.pop_back();
            // }

            if(follow_init_) { // init
                point_pos_x_.clear();
                point_pos_y_.clear();
                point_rot_z_.clear();
                point_pos_x_.shrink_to_fit();
                point_pos_y_.shrink_to_fit();
                point_rot_z_.shrink_to_fit();
                cmdvel_index_ = 0;
                follow_init_ = false;

                point_pos_x_.push_back(0.0);
                point_pos_y_.push_back(0.0);
                point_rot_z_.push_back(0.0);
                ledcomm.led[0] = 0;
                ledcomm.led[1] = 0;
                ledcomm.led[2] = 0;
                ledcomm.light[0] = 0;
                ledcomm.light[1] = 0;
                ledcomm.light[2] = 0;
            }

            ledcomm.led[0] = 0;
            ledcomm.led[1] = 0;
            ledcomm.led[2] = 0;

            cmd_vel_pub_->publish(twist);
            play_back_point_pub_cnt_ = 1;
            replay_point_pub_cnt_ = 1;
        } else if (button_state_[0] == 1 && button_state_[1] == 0 && button_state_[2] == 0) {
            /* following mode and memorize cmd_vel to go back starting point */
            if(truefalse_playback_ == false) { // valid value kill reset
                truefalse_playback_ = true;
            }

            if(truefalse_replay_ == false) {
                truefalse_replay_ = true;
            }

            if(cancel_state_ != 1) {
                cancel_state_ = 1;
                nav_cancel_data.nav_cancel = true;
                nav_cancel_pub_->publish(nav_cancel_data);
            } else {
                nav_cancel_data.nav_cancel = false;
                nav_cancel_pub_->publish(nav_cancel_data);
            }

            // start of following
            if(target_id_ != -1) {
                if(target_lost_ == 1) { // lost
                    linear = 0.0;
                    angular = 0.0;
                    roi_center_x_ = 0.5;
                    roi_center_y_ = 0;
                    remote_controller_ = true;

                    if(blink_ < 20) {
                        blink_++;
                        ledcomm.led[0] = 1;
                        ledcomm.led[1] = 0;
                        ledcomm.led[2] = 0;
                        ledcomm.light[0] = 1;
                        ledcomm.light[1] = 0;
                        ledcomm.light[2] = 0;
                    } else if(blink_ < 40) {
                        blink_++;
                        ledcomm.led[0] = 0;
                        ledcomm.led[1] = 0;
                        ledcomm.led[2] = 0;
                        ledcomm.light[0] = 0;
                        ledcomm.light[1] = 0;
                        ledcomm.light[2] = 0;
                    } else {
                        blink_ = 0;
                    }

                    if (point_state_ == 1) { // mark point by button
                        point_pos_x_.push_back(odom_x_);
                        point_pos_y_.push_back(odom_y_);
                        point_rot_z_.push_back(odom_z_);
                        cmdvel_index_++;
                        point_state_ = 0;
                    }

                    g_target_detecting_state = 2;
                } else { // move
                    target_distance = sqrt(pow(target_obj_info_vector[target_id_].position[0],2) + pow(target_obj_info_vector[target_id_].position[1],2));
                    target_angle = atan2(target_obj_info_vector[target_id_].position[1], target_obj_info_vector[target_id_].position[0]);

                    velocity_comm = 2.0 * (target_distance - COLLISION_DISTANCE);
                    if(velocity_comm < 0.0) {
                        velocity_comm = 0.0;
                    }
                    
                    linear = weight_linear*velocity_comm + (1 - weight_linear)*bef_vel_comm;
                    angular = weight_angular * target_angle; //+ (1 - weight_angular)*bef_angle_comm;

                    if((frontdist_ < ULTRASONIC_LIMIT_DISTANCE) && (frontdist_ > 0)) {
                        linear = 0.0;
                        angular = 0.0;
                    }

                    if(linear >= 1.3) {
                        linear = 1.3;
                    }

                    if(abs(target_obj_info_vector[target_id_].position[1]) < 0.25) {
                        angular = 0;
                    }

                    bef_angle_comm = angular;
                    bef_vel_comm = velocity_comm;

                    remote_controller_ = false;

                    target_detect_ = 1;
                    ledcomm.led[0] = 1;
                    ledcomm.led[1] = 0;
                    ledcomm.led[2] = 0;
                    ledcomm.light[0] = 1;
                    ledcomm.light[1] = 0;
                    ledcomm.light[2] = 0;

                    g_target_detecting_state = 1;
                }
            } else if(target_id_ == -1) {
                linear = 0.0;
                angular = 0.0;
                roi_center_x_ = 0.5;
                roi_center_y_ = 0;
                remote_controller_ = true;

                if(target_detect_ == 1) { // lost
                    target_lost_ = 1;

                    if(blink_ < 20) {
                        blink_++;
                        ledcomm.led[0] = 1;
                        ledcomm.led[1] = 0;
                        ledcomm.led[2] = 0;
                        ledcomm.light[0] = 1;
                        ledcomm.light[1] = 0;
                        ledcomm.light[2] = 0;
                    } else if(blink_ < 40) {
                        blink_++;
                        ledcomm.led[0] = 0;
                        ledcomm.led[1] = 0;
                        ledcomm.led[2] = 0;
                        ledcomm.light[0] = 0;
                        ledcomm.light[1] = 0;
                        ledcomm.light[2] = 0;
                    } else {
                        blink_ = 0;
                    }

                    g_target_detecting_state = 2;
                }
            } else {
                linear = 0.0;
                angular = 0.0;
                remote_controller_ = true;
                button_state_[0] = 0;
                button_state_[1] = 0;
                button_state_[2] = 0;
            }

            // collision
            following_collision = following_collision_avoidance(collision_obj_info_vector);

            if(following_collision == 1 || ((frontdist_ < ULTRASONIC_LIMIT_DISTANCE) && (frontdist_ > 0))) {
                linear = 0.0;
                angular = 0.0;
            }

            ///////////////////////////
            //  memorize linear and angular data if following on
            //  during following mode if lost happened, datas are most useless so let it know
            ///////////////////////////

            if(target_lost_ != 1) {
                remote_controller_ = false;

                if (point_state_ == 1) {// mark point by button
                    point_pos_x_.push_back(odom_x_);
                    point_pos_y_.push_back(odom_y_);
                    point_rot_z_.push_back(odom_z_);
                    cmdvel_index_++;
                    point_state_ = 0;
                }
                // else if (point_del_state_ == 1
                // && point_pos_x_[0] != 0 && point_pos_y_[0] != 0 && point_rot_z_[0] != 0)
                // {
                //     point_pos_x_.pop_back();
                //     point_pos_y_.pop_back();
                //     point_rot_z_.pop_back();
                // }
                
                // if(linear != 0.0 || angular != 0.0) // trace point
                // {                    
                //     trace_cnt_++; // processing point
                //     if(trace_cnt_ == 8)
                //     {
                //         point_pos_x_.push_back(odom_x_);
                //         point_pos_y_.push_back(odom_y_);
                //         point_rot_z_.push_back(odom_z_);
                //         cmdvel_index_++;
                //         trace_cnt_ = 0;
                //     }
                // }
            }  else if(target_lost_ == 1) { // collision
                linear = 0.0;
                angular = 0.0;
            }

            if(angular > M_PI/3) {
                angular = M_PI/3;
            } else if(angular < -M_PI/3) {
                angular = -M_PI/3;
            }

            map_obstacle_idx_ = -1;
            twist.linear.x = linear;
            twist.angular.z = angular;
            is_motor_coast.iscoast = remote_controller_;
            determinant_value.determinant = 0;

            cmd_vel_pub_->publish(twist);
        } else if(button_state_[0] == 0 && button_state_[1] == 1 && button_state_[2] == 0) { // go back to starting point
            ////////////////////////////////////////////
            //  blink to ask playback to starting point
            ////////////////////////////////////////////

            // led part
            // if(blink_ == false) {
            //     blink_ = true;
            //     ledcomm.led[1] = 1;
            // } else {
            //     blink_ = false;
            //     ledcomm.led[1] = 0;
            // }

            if(cancel_state_ != 2) {
                cancel_state_ = 2;
                nav_cancel_data.nav_cancel = true;
                nav_cancel_pub_->publish(nav_cancel_data);
            } else {
                nav_cancel_data.nav_cancel = false;
                nav_cancel_pub_->publish(nav_cancel_data);
            }

            // relative_x_ = cos(odom_z_) * (point_pos_x_[cmdvel_index_] - odom_x_) + sin(odom_z_) * (point_pos_y_[cmdvel_index_] - odom_y_);
            // relative_x_ = -1;

            map_obstacle_idx_ = -1;
            ledcomm.led[0] = 0;
            ledcomm.led[1] = 1;
            ledcomm.led[2] = 0;
            ledcomm.light[0] = 0;
            ledcomm.light[1] = 0;
            ledcomm.light[2] = 1;
            is_motor_coast.iscoast = remote_controller_;
            g_target_detecting_state = 0;
            determinant_value.determinant = 1;
            play_back_point_pub_cnt_ = 1;
            replay_point_pub_cnt_ = 1;
        } else if(button_state_[0] == 0 && button_state_[1] == 2 && button_state_[2] == 0) { // go back to starting point
            ///////////////////////////
            //  play memorized linear and angular data from last data to first data
            //  robot moves like rear wheel drive => might be need to change data + to - , - to +
            ///////////////////////////
            remote_controller_ = false;

            if(truefalse_playback_ == true) { // valid value kill
                if(cmdvel_index_ < 1) {
                    cmdvel_index_ = 1;
                }
                truefalse_playback_ = false;
            }

            // std::cout << "Please enter the point number you want to go: ";
            // std:cin >> cmdvel_index_;

            // if(cmdvel_index_ < 1) {
            //     std::cout << "You entered wrong number. Please enter 1 ~ " << point_pos_x_.size() - 1 << std::endl;
            //     button_state_[0] = 0;
            //     button_state_[1] = 0;
            //     button_state_[2] = 0;
            //     // cmdvel_index_ = cmdvel_index_bef_;
            // } else if (cmdvel_index_ > point_pos_x_.size() - 1) {
            //     std::cout << "You entered wrong number. Please enter 1 ~ " << point_pos_x_.size() - 1 << std::endl;
                // button_state_[0] = 0;
                // button_state_[1] = 1;
                // button_state_[2] = 0;
            //     // cmdvel_index_ = cmdvel_index_bef_;
            // } else {
                point_data.x_pos = point_pos_x_[cmdvel_index_];
                point_data.y_pos = point_pos_y_[cmdvel_index_];

                if(cancel_state_ != 3) {
                    cancel_state_ = 3;
                    nav_cancel_data.nav_cancel = true;
                    nav_cancel_pub_->publish(nav_cancel_data);
                    sleep(1);
                    point_pub_->publish(point_data);
                } else {
                    nav_cancel_data.nav_cancel = false;
                    nav_cancel_pub_->publish(nav_cancel_data);
                }

                ledcomm.led[0] = 0;
                ledcomm.led[1] = 1;
                ledcomm.led[2] = 0;

                // collision
                auto [ptop_collision, obstacle_idx] = ptop_collision_avoidance(collision_obj_info_vector);

                // if target is detected nearby the front 
                // 1.send cancel
                // 2.move back
                // 3.send point again       
                point_move(ptop_collision, obstacle_idx, collision_obj_info_vector);

                map_obstacle_idx_ = obstacle_idx;
                is_motor_coast.iscoast = remote_controller_;
                g_target_detecting_state = 0;
                determinant_value.determinant = 1;
            // }
        } else if (button_state_[0] == 0 && button_state_[1] == 0 && button_state_[2] == 1) { // go back to robot's target(robot owner) point
            ////////////////////////////////////////////
            //  blink to ask playback to followed point
            ////////////////////////////////////////////

            // led part
            // button_led = 0 is off, button_led = 1 is blink, button_led = 2 is on
            // if(blink_ == false) {
            //     blink_ = true;
            //     ledcomm.led[2] = 1;
            // } else {
            //     blink_ = false;
            //     ledcomm.led[2] = 0;
            // }

            if(cancel_state_ != 4) {
                cancel_state_ = 4;
                nav_cancel_data.nav_cancel = true;
                nav_cancel_pub_->publish(nav_cancel_data);
            } else {
                nav_cancel_data.nav_cancel = false;
                nav_cancel_pub_->publish(nav_cancel_data);
            }

            // relative_x_ = cos(odom_z_) * (point_pos_x_[cmdvel_index_] - odom_x_) + sin(odom_z_) * (point_pos_y_[cmdvel_index_] - odom_y_);
            // relative_x_ = -1;

            map_obstacle_idx_ = -1;
            ledcomm.led[0] = 0;
            ledcomm.led[1] = 0;
            ledcomm.led[2] = 1;
            ledcomm.light[0] = 0;
            ledcomm.light[1] = 0;
            ledcomm.light[2] = 1;
            is_motor_coast.iscoast = remote_controller_;
            g_target_detecting_state = 0;
            determinant_value.determinant = 1;
            replay_point_pub_cnt_ = 1;
        } else if (button_state_[0] == 0 && button_state_[1] == 0 && button_state_[2] == 2) { // go back to robot's target(robot owner) point
            ///////////////////////////
            //  send original linear and angular data
            ///////////////////////////

            remote_controller_ = false;

            if(truefalse_replay_ == true) {// valid value kill
                if(cmdvel_index_ > (point_pos_x_.size() - 1)) {
                    cmdvel_index_ = (point_pos_x_.size() - 1);
                }
                truefalse_replay_ = false;
            }

            // std::cout << "Please enter the point number you want to go: ";
            // std:cin >> cmdvel_index_;

            // if(cmdvel_index_ < 1) {
            //     std::cout << "You entered wrong number. Please enter 1 ~ " << point_pos_x_.size() - 1 << std::endl;
            //     button_staled_pub_te_[0] = 0;
            //     button_state_[1] = 0;
            //     button_state_[2] = 0;
            //     // cmdvel_index_ = cmdvel_index_bef_;
            // } else if (cmdvel_index_ > point_pos_x_.size() - 1) {
            //     std::cout << "You entered wrong number. Please enter 1 ~ " << point_pos_x_.size() - 1 << std::endl;
            //     button_state_[0] = 0;
            //     button_state_[1] = 0;
            //     button_state_[2] = 0;
            //     // cmdvel_index_ = cmdvel_index_bef_;
            // } else {
                point_data.x_pos = point_pos_x_[cmdvel_index_];
                point_data.y_pos = point_pos_y_[cmdvel_index_];

                if(cancel_state_ != 5) {
                    cancel_state_ = 5;
                    nav_cancel_data.nav_cancel = true;
                    nav_cancel_pub_->publish(nav_cancel_data);
                    sleep(1);
                    point_pub_->publish(point_data);
                } else {
                    nav_cancel_data.nav_cancel = false;
                    nav_cancel_pub_->publish(nav_cancel_data);
                }
                
                auto [ptop_collision, obstacle_idx] = ptop_collision_avoidance(collision_obj_info_vector);

                point_move(ptop_collision, obstacle_idx, collision_obj_info_vector);

                map_obstacle_idx_ = obstacle_idx;
                ledcomm.led[0] = 0;
                ledcomm.led[1] = 0;
                ledcomm.led[2] = 1;
                is_motor_coast.iscoast = remote_controller_;
                g_target_detecting_state = 0;
                determinant_value.determinant = 1;
            // }
        } else if (coast_state_ == 1) {
            determinant_value.determinant = 2;
            map_obstacle_idx_ = -1;
            ledcomm.led[0] = 0;
            ledcomm.led[1] = 0;
            ledcomm.led[2] = 0; // 1
            ledcomm.light[0] = 0;
            ledcomm.light[1] = 0;
            ledcomm.light[2] = 0;
        } else { // buttons overlapping
            linear = 0.0;
            angular = 0.0;
            remote_controller_ = true;
            button_state_[0] = 0;
            button_state_[1] = 0;
            button_state_[2] = 0;

            twist.linear.x = linear;
            twist.angular.z = angular;
            ledcomm.led[0] = 0;
            ledcomm.led[1] = 0;
            ledcomm.led[2] = 0;
            ledcomm.light[0] = 0;
            ledcomm.light[1] = 0;
            ledcomm.light[2] = 0;
            is_motor_coast.iscoast = remote_controller_;
            target_lost_ = 0;
            target_detect_ = 0;
            g_target_detecting_state = 0;
            determinant_value.determinant = 0;
            map_obstacle_idx_ = -1;
        }
    }

    // firmware command (esp32 write control) : 1 = firmware communication on, 0 = firmware communication off
    ledcomm.firmwarecommand = 1;
    // ros2 driver transmit control : 1 = communication on, 0 = communication off
    ledcomm.transmitcommand = 1;

    // determinant_value.modenum = 1;

    controller_pub_->publish(is_motor_coast);
    led_pub_->publish(ledcomm);
    following_state_pub_->publish(determinant_value);

    // map_out_pub(target_obj_info_vector, target_id_+1);
    footprint_marker(target_obj_info_vector, map_obstacle_idx_);
    // pub_marker(target_obj_info_vector);
    rev_make_ref_map();
}

void clusteringNode::make_ref_map(void)
{
    for (size_t i = 0; i < filtered_lidar_cloud->points.size(); i++) {
        map_cloud->points.push_back(filtered_lidar_cloud->points[i]);
    }
    map_cloud->width = map_cloud->points.size();
    map_cloud->height = 1;
    map_cloud->is_dense = true;
}

void clusteringNode::rev_make_ref_map(void)
{
    for(size_t i = 0; i < filtered_lidar_cloud->points.size(); i++) {
        map_cloud->points.clear();
    }
    map_cloud->width = map_cloud->points.size();
    map_cloud->height = -1;
    map_cloud->is_dense = false;
}

void clusteringNode::map_downsampling(void)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_lidar_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*map_cloud);
}

void clusteringNode::filtered_euclidean_clustering(std::vector<ObjInfo>& info)
{
    ObjInfo filltered_clusterinfo;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    int max_iterations = this->get_parameter("max_iterations").as_int();
    double cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    int min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    int max_cluster_size = this->get_parameter("max_cluster_size").as_int();

    // seg.setOptimizeCoefficients (true);
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (max_iterations);
    // seg.setDistanceThreshold (0.02);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    if(!filtered_lidar_cloud->empty()) {
        tree->setInputCloud (filtered_lidar_cloud);
    }

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (filtered_lidar_cloud);
    ec.extract (cluster_indices);

    int cluster_index = 0;

    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*filtered_lidar_cloud)[idx]);
        }

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Eigen::Vector4f centroid;
        pcl::PointXYZ min_point, max_point;

        // pcl::compute3DCentroid(*cloud_cluster, centroid);       // cluster center position
        pcl::getMinMax3D(*cloud_cluster, min_point, max_point); // cluster size

        Eigen::Vector2f centroid(0.0f, 0.0f);
        for(const auto& point : cloud_cluster->points) {
            centroid[0] += point.x;
            centroid[1] += point.y;
        }
        centroid /= static_cast<float>(cloud_cluster->size());

        Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
        for(const auto& point : cloud_cluster->points) {
            Eigen::Vector2f pt(point.x - centroid[0], point.y - centroid[1]);
            covariance += pt * pt.transpose();
        }
        covariance /= static_cast<float>(cloud_cluster->size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
        Eigen::Vector2f principal_direction = eigen_solver.eigenvectors().col(1);

        float angle = std::atan2(principal_direction.y(), principal_direction.x());
        float angle_degrees = angle * 180.0f / M_PI;

        filltered_clusterinfo.id =  cluster_index + 1;
        filltered_clusterinfo.width = max_point.y - min_point.y;
        filltered_clusterinfo.height = max_point.x - min_point.x;
        filltered_clusterinfo.position[0] = (max_point.x + min_point.x)/2;
        filltered_clusterinfo.position[1] = (max_point.y + min_point.y)/2;
        filltered_clusterinfo.angle = angle_degrees;

        cluster_index++;

        info.push_back(filltered_clusterinfo);
    }
}

void clusteringNode::euclidean_clustering(std::vector<ObjInfo>& info)
{
    ObjInfo clusterinfo;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // seg.setOptimizeCoefficients (true);
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (100);
    // seg.setDistanceThreshold (0.02);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    if(!map_cloud->empty()) {
        tree->setInputCloud (map_cloud);
    }
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (0.2);
    ec.setMinClusterSize (2);
    ec.setMaxClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (map_cloud);
    ec.extract (cluster_indices);

    int cluster_index = 0;

    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*map_cloud)[idx]);
        }

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Eigen::Vector4f centroid;
        pcl::PointXYZ min_point, max_point;

        // pcl::compute3DCentroid(*cloud_cluster, centroid);       // cluster center position
        pcl::getMinMax3D(*cloud_cluster, min_point, max_point); // cluster size

        Eigen::Vector2f centroid(0.0f, 0.0f);
        for(const auto& point : cloud_cluster->points) {
            centroid[0] += point.x;
            centroid[1] += point.y;
        }
        centroid /= static_cast<float>(cloud_cluster->size());

        Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
        for(const auto& point : cloud_cluster->points) {
            Eigen::Vector2f pt(point.x - centroid[0], point.y - centroid[1]);
            covariance += pt * pt.transpose();
        }
        covariance /= static_cast<float>(cloud_cluster->size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
        Eigen::Vector2f principal_direction = eigen_solver.eigenvectors().col(1);

        float angle = std::atan2(principal_direction.y(), principal_direction.x());
        float angle_degrees = angle * 180.0f / M_PI;

        clusterinfo.id =  cluster_index + 1;
        clusterinfo.width = max_point.y - min_point.y;
        clusterinfo.height = max_point.x - min_point.x;
        clusterinfo.position[0] = (max_point.x + min_point.x)/2;
        clusterinfo.position[1] = (max_point.y + min_point.y)/2;
        clusterinfo.angle = angle_degrees;

        cluster_index++;
        info.push_back(clusterinfo);
    }
}

int clusteringNode::target_detect(std::vector<ObjInfo> obj_info)
{
    double distance, min_distance = 0.5;
    int target_index = -1;
    int detect = -1;

    for(size_t i = 0; i < obj_info.size(); i++) {
        distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0] + obj_info[i].position[1] * obj_info[i].position[1]);

        if(min_distance > distance) {
            min_distance = distance;
            target_index = i;
        }
    }

    if(target_index >= 0) {
        detect = target_index;
        roi_center_x_ = obj_info[target_index].position[0];
        roi_center_y_ = obj_info[target_index].position[1];
    } else {
        detect = -1;
    }

    return detect;
}

int clusteringNode::roi_detect(std::vector<ObjInfo> obj_info)
{
    int target_lost = -1;

    for(size_t i = 0; i < obj_info.size(); i++) {
        if( sqrt(pow(obj_info[i].position[0] - roi_center_x_, 2) + pow(obj_info[i].position[1] - roi_center_y_, 2)) < ROI_RANGE
        && obj_info[i].position[0] != 0 && obj_info[i].position[1] != 0) {
            roi_center_x_ = obj_info[i].position[0];
            roi_center_y_ = obj_info[i].position[1];
            target_lost = i;

            return target_lost;
        } else {
            target_lost = -1;
        }
    }
    return target_lost;
}

int clusteringNode::following_collision_avoidance(std::vector<ObjInfo> obj_info)
{
    int avoidance = 0;
    float distance, collision_distance = COLLISION_DISTANCE;

    for(size_t i = 0; i < obj_info.size(); i++) {
        distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0]
        + obj_info[i].position[1] * obj_info[i].position[1]);

        if(distance < collision_distance)
        {
            collision_distance = distance;
        }
    }

    if(collision_distance < COLLISION_DISTANCE) {
        avoidance = 1;
    } else {
        avoidance = 0;
    }
    return avoidance; // 1 means collision could happen by any other objects, 0 is good to move
}

std::tuple<int, int> clusteringNode::ptop_collision_avoidance(std::vector<ObjInfo> obj_info)
{ // away area and distance redefine // 
    int obstacle_index = -1;
    int avoidance = 0;
    double obstacle_distance;

    for(size_t i = 0; i < obj_info.size(); i++) {
        if((obj_info[i].position[0] > abs(obj_info[i].position[1])*sqrt(1/3)
        && sqrt(pow(obj_info[i].position[0],2) + pow(obj_info[i].position[1],2)) < 0.7)
        || (abs(obj_info[i].position[1]) < 0.45 && obj_info[i].position[0] < 0.5 && obj_info[i].position[0] > 0.0)) {// area looks sector form + rectangular
            avoidance = 1;
            obstacle_index = i;
            obstacle_distance = sqrt(pow(obj_info[i].position[0],2) + pow(obj_info[i].position[1],2));

            if(sqrt(pow(obj_info[obstacle_index].position[0],2) + pow(obj_info[obstacle_index].position[1],2)) < 0.5) {
                avoidance = 2;
                obstacle_distance = sqrt(pow(obj_info[obstacle_index].position[0],2) + pow(obj_info[obstacle_index].position[1],2));
            }
            break;
        } else {
            avoidance = 0;
            obstacle_distance = -1;
        }
    }
    return std::make_tuple(avoidance, obstacle_index); // 0 == good to go, 1 == collision could happen by any other objects, 2 == move back until distance between obstacle over 0.5
}

void clusteringNode::point_move(int ptop_collision, int obstacle_idx, std::vector<ObjInfo> collision_obj_info_vector)
{
    // ptop_collision => 0: no obstacle, 1: grey area. if obstacle detected keep back  
    // if not ignore , 2: sudden obstacle detected
    // obstacle_idx is an index of obstacle suddenly came out

    geometry_msgs::msg::Twist twist;
    message_interface::msg::Odomdata point_data;
    message_interface::msg::Navcancel nav_cancel_data;
    // message_interface::msg::Ledcomm ledcomm;

    double linear, angular, distance_sub, calculated_angle;
    
    distance_sub = sqrt(pow(point_pos_x_[cmdvel_index_] - (double)odom_x_, 2) + pow(point_pos_y_[cmdvel_index_] - (double)odom_y_, 2));

    if(distance_sub < STOP_DISTANCE * 3) {
        play_back_point_pub_cnt_ = 1;
        replay_point_pub_cnt_ = 1;
        nav_cancel_data.nav_cancel = true;
        cancel_ = true;
        odom_z_save_ = true;
        nav_spin_ = true;
        nav_spin_cnt_ = 0;

        if(!play_back_control_) {
            play_back_control_ = 1;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            cmd_vel_pub_->publish(twist);
        }

        nav_cancel_pub_->publish(nav_cancel_data);

        button_state_[0] = 0;
        button_state_[1] = 0;
        button_state_[2] = 0;
        ledcomm.led[0] = 0;
        ledcomm.led[1] = 0;
        ledcomm.led[2] = 0;
        ledcomm.light[0] = 0;
        ledcomm.light[1] = 0;
        ledcomm.light[2] = 0;
        led_pub_->publish(ledcomm);
    } 
    // else if(distance_sub < STOP_DISTANCE * 5) {
    //     if(play_back_control_) {
    //         play_back_control_ = 0;
    //         nav_cancel_data.nav_cancel = true;
    //         nav_cancel_pub_->publish(nav_cancel_data);
    //     }
    //     angular = atan2((point_pos_y_[cmdvel_index_] - odom_y_), (point_pos_x_[cmdvel_index_] - odom_x_)) - odom_z_;
    //     if (angular >= M_PI/3.0 || angular <= -M_PI/3.0) {
    //         linear = 0.0;
    //     } else {odom_z_spin_save_val_
    //         linear = 0.1;
    //     }
    //     if(angular >= M_PI/3.0) {
    //         angular = 0.3;
    //     } else if (angular <= -M_PI/3.0) {
    //         angular = -0.3;
    //     }
    //     twist.linear.x = linear;
    //     twist.angular.z = -angular;
    //     cmd_vel_pub_->publish(twist);
    // } 
    // else if(init_spin_) {
    //     // calculated_angle = atan2(odom_y_ - point_pos_y_[cmdvel_index_], odom_x_ - point_pos_x_[cmdvel_index_]) - odom_z_;
    //     if(cancel_ == true) {
    //         cancel_ = false;
    //         g_odom = odom_z_;
    //         nav_cancel_data.nav_cancel = true;
    //         nav_cancel_pub_->publish(nav_cancel_data);
    //     }
    //     // if(calculated_angle < -M_PI) {
    //     //     calculated_angle = calculated_angle + 2*M_PI;
    //     // } else if(calculated_angle > M_PI) {
    //     //     calculated_angle = calculated_angle - 2*M_PI;
    //     // }
    //     // if(abs(calculated_angle) <= M_PI/18) {
    //     //     init_spin_ = false;
    //     //     twist.linear.x = 0.0;
    //     //     twist.angular.z = 0.0;
    //     //     cmd_vel_pub_->publish(twist);
    //     // } else if (calculated_angle > -M_PI && calculated_angle < -M_PI/18) {
    //     //     twist.linear.x = 0.0;
    //     //     twist.angular.z = -0.2;
    //     //     cmd_vel_pub_->publish(twist);
    //     // } else {
    //     //     twist.linear.x = 0.0;
    //     //     twist.angular.z = 0.2;
    //     //     cmd_vel_pub_->publish(twist);
    //     // }
    //     // std::cout << "calculatnav_spin_cnt_ed_angle " << calculated_angle << std::endl;
    //     calculated_angle = odom_z_ - g_odom;
    //     if(abs(calculated_angle) <= M_PI/18) {
    //         init_spin_ = false;
    //         twist.linear.x = 0.0;
    //         twist.angular.z = 0.0;
    //         cmd_vel_pub_->publish(twist);
    //         std::cout << "180 end\n";
    //     } else {
    //         twist.linear.x = 0.0;
    //         twist.angular.z = 0.2;
    //         cmd_vel_pub_->publish(twist);
    //     }
    // }
    else {
        point_data.x_pos = point_pos_x_[cmdvel_index_];
        point_data.y_pos = point_pos_y_[cmdvel_index_];

        // relative_x = cos(odom_z_) * point_pos_x_[cmdvel_index_] - sin(odom_z_) * point_pos_y_[cmdvel_index_];
        
        if(relative_x_ >= 0) {
            if(obstacle_handle_mode_ == 1) { // Avoidance mode 1
                if(avoidance_num_ == 0) { // stop until press button
                    if(frontdist_ < 500 && frontdist_ > 0) {
                        play_back_point_pub_cnt_ = 1;
                        replay_point_pub_cnt_ = 1;
                        nav_cancel_data.nav_cancel = true;
                        button_state_[0] = 0;
                        button_state_[1] = 0;
                        button_state_[2] = 0;
                        nav_cancel_pub_->publish(nav_cancel_data);
                    } else if(ptop_collision == 2) {
                        if(collision_back_) {
                            collision_back_ = false;
                            play_back_point_pub_cnt_ = 1;
                            replay_point_pub_cnt_ = 1;
                            nav_cancel_data.nav_cancel = true;
                            obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                            button_state_[0] = 0;
                            button_state_[1] = 0;
                            button_state_[2] = 0;
                            nav_cancel_pub_->publish(nav_cancel_data);
                        } else {
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;
                            button_state_[0] = 0;
                            button_state_[1] = 0;
                            button_state_[2] = 0;
                            cmd_vel_pub_->publish(twist);
                        }
                    } else if(ptop_collision == 1) {
                        if(!collision_back_) {
                            // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                twist.linear.x = -0.15;
                                twist.angular.z = 0.0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 1;
                            // } else {
                            //     twist.linear.x = 0.0;
                            //     twist.angular.z = 0.0; 
                            // }
                            led_pub_->publish(ledcomm);
                            cmd_vel_pub_->publish(twist);
                        }
                    } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && ptop_collision == 0) {
                        sleep(1);
                        collision_back_ = true;
                        play_back_point_pub_cnt_ = 0;
                        replay_point_pub_cnt_ = 0;
                        // ledcomm.led[0] = 0;
                        // ledcomm.led[1] = 0;
                        // ledcomm.led[2] = 1;
                        ledcomm.light[0] = 1;
                        ledcomm.light[1] = 0;
                        ledcomm.light[2] = 0;
                        led_pub_->publish(ledcomm);
                        point_pub_->publish(point_data);
                    }
                } else if(avoidance_num_ > 3) { // try to keep avoid
                    if(frontdist_ < 500 && frontdist_ > 0) {
                    //     if(collision_back_) {
                    //         collision_back_ = false;
                    //         collision_spin_ = true;
                    //         play_back_point_pub_cnt_ = 1;
                    //         replay_point_pub_cnt_ = 1;
                    //         nav_cancel_data.nav_cancel = true;
                    //         if(obstacle_idx != -1){
                    //             obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                    //         } else {
                    //             obstacle_angle_ = OBSTACLE_ANGLE_BOUNDARY;
                    //         }

                    //         nav_cancel_pub_->publish(nav_cancel_data);
                    //     } else {
                    //         if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                    collision_back_ = false;
                                twist.linear.x = -0.15;
                                twist.angular.z = 0.0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 1;
                    //         } else {
                    //             twist.linear.x = 0.0;
                    //             twist.angular.z = 0.0; 
                    //         }
                            led_pub_->publish(ledcomm);
                            cmd_vel_pub_->publish(twist);
                    //     }
                    } else if(ptop_collision == 2) {
                        if(collision_back_) {
                            collision_back_ = false;
                            play_back_point_pub_cnt_ = 1;
                            replay_point_pub_cnt_ = 1;
                            nav_cancel_data.nav_cancel = true;
                            obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                            // ledcomm.led[0] = 0;
                            // ledcomm.led[1] = 1;
                            // ledcomm.led[2] = 0;
                            ledcomm.light[0] = 0;
                            ledcomm.light[1] = 0;
                            ledcomm.light[2] = 1;
                            led_pub_->publish(ledcomm);
                            nav_cancel_pub_->publish(nav_cancel_data);
                        } else {
                            // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                twist.linear.x = -0.15;
                                twist.angular.z = 0.0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 1;
                            // } else {
                            //     twist.linear.x = 0.0;
                            //     twist.angular.z = 0.0; 
                            // }
                            led_pub_->publish(ledcomm);
                            cmd_vel_pub_->publish(twist);
                        }
                    } else if(ptop_collision == 1) {
                        if(!collision_back_) {
                            // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                twist.linear.x = -0.15;
                                twist.angular.z = 0.0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 1;
                            // } else {
                            //     twist.linear.x = 0.0;
                            //     twist.angular.z = 0.0; 
                            // }
                            collision_spin_ = true;
                            led_pub_->publish(ledcomm);
                            cmd_vel_pub_->publish(twist);
                        }
                    } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && ptop_collision == 0) {
                        if(collision_spin_ == true) {
                            collision_back_ = true;
                            twist.linear.x = 0.0;
                            if(obstacle_angle_ > OBSTACLE_ANGLE_BOUNDARY) {
                                twist.angular.z = 0.2;
                                spin_cnt_++;

                                if(spin_cnt_ > SPIN_COUNTER) {
                                    spin_cnt_ = 0;
                                    twist.angular.z = 0.0;
                                    collision_spin_ = false;
                                }
                            } else {
                                twist.angular.z = -0.2;
                                spin_cnt_++;

                                if (spin_cnt_ > SPIN_COUNTER) {
                                    spin_cnt_ = 0;
                                    twist.angular.z = 0.0;
                                    collision_spin_ = false;
                                }
                            }
                            cmd_vel_pub_->publish(twist);
                        }  else {
                            sleep(1);
                            play_back_point_pub_cnt_ = 0;
                            replay_point_pub_cnt_ = 0;
                            // ledcomm.led[0] = 0;
                            // ledcomm.led[1] = 0;
                            // ledcomm.led[2] = 1;
                            ledcomm.light[0] = 1;
                            ledcomm.light[1] = 0;
                            ledcomm.light[2] = 0;
                            led_pub_->publish(ledcomm);
                            point_pub_->publish(point_data);
                        }
                    }
                } else if(avoidance_num_ >= 1 && avoidance_num_ <= 3) { // try to avoid a few times and stop
                    if(frontdist_ < 500 && frontdist_ > 0) {
                    //     if(collision_back_) {
                    //         collision_back_ = false;
                    //         collision_spin_ = true;
                    //         play_back_point_pub_cnt_ = 1;
                    //         replay_point_pub_cnt_ = 1;
                    //         nav_cancel_data.nav_cancel = true;
                    //         if(obstacle_idx != -1){
                    //             obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                    //         } else {
                    //             obstacle_angle_ = OBSTACLE_ANGLE_BOUNDARY;
                    //         }
                    //         avoidance_cnt_++;

                    //         if(avoidance_cnt_ > avoidance_num_) {
                    //             button_state_[0] = 0;
                    //             button_state_[1] = 0;
                    //             button_state_[2] = 0;
                    //         }

                    //         nav_cancel_pub_->publish(nav_cancel_data);
                    //     } else {
                    //         if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                collision_back_ = false;
                                twist.linear.x = -0.15;
                                twist.angular.z = 0.0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 1;
                    //         } else {
                    //             twist.linear.x = 0.0;
                    //             twist.angular.z = 0.0; 
                    //         }
                            led_pub_->publish(ledcomm);
                            cmd_vel_pub_->publish(twist);
                    //     }
                    } else if(ptop_collision == 2) {
                        if(collision_back_) {
                            collision_back_ = false;
                            play_back_point_pub_cnt_ = 1;
                            replay_point_pub_cnt_ = 1;
                            nav_cancel_data.nav_cancel = true;
                            obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                            avoidance_cnt_++;

                            if(avoidance_cnt_ > avoidance_num_) {
                                button_state_[0] = 0;
                                button_state_[1] = 0;
                                button_state_[2] = 0;
                            }

                            nav_cancel_pub_->publish(nav_cancel_data);
                        } else {
                            // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                twist.linear.x = -0.15;
                                twist.angular.z = 0.0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 1;
                            // } else {
                            //     twist.linear.x = 0.0;
                            //     twist.angular.z = 0.0; 
                            // }
                            led_pub_->publish(ledcomm);
                            cmd_vel_pub_->publish(twist);
                        }
                    } else if(ptop_collision == 1) {
                        if(!collision_back_) {
                            // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                twist.linear.x = -0.15;
                                twist.angular.z = 0.0;
                            // } else {
                            //     twist.linear.x = 0.0;
                            //     twist.angular.z = 0.0;
                            // }
                            collision_spin_ = true;
                            // ledcomm.led[0] = 0;
                            // ledcomm.led[1] = 1;
                            // ledcomm.led[2] = 0;
                            ledcomm.light[0] = 0;
                            ledcomm.light[1] = 0;
                            ledcomm.light[2] = 1;
                            led_pub_->publish(ledcomm);
                            cmd_vel_pub_->publish(twist);
                        }
                    } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && ptop_collision == 0) {
                        if(collision_spin_ == true) {
                            collision_back_ = true;
                            twist.linear.x = 0.0;
                            if(obstacle_angle_ > OBSTACLE_ANGLE_BOUNDARY) {
                                twist.angular.z = 0.2;
                                spin_cnt_++;

                                if(spin_cnt_ > SPIN_COUNTER) {
                                    spin_cnt_ = 0;
                                    twist.angular.z = 0.0;
                                    collision_spin_ = false;
                                }
                            } else {
                                twist.angular.z = -0.2;
                                spin_cnt_++;

                                if (spin_cnt_ > SPIN_COUNTER) {
                                    spin_cnt_ = 0;
                                    twist.angular.z = 0.0;
                                    collision_spin_ = false;
                                }
                            }
                            cmd_vel_pub_->publish(twist);
                        } else {
                            // collision_spin_ = false;
                            sleep(1);
                            // ledcomm.led[0] = 0;
                            // ledcomm.led[1] = 0;
                            // ledcomm.led[2] = 1;
                            ledcomm.light[0] = 1;
                            ledcomm.light[1] = 0;
                            ledcomm.light[2] = 0;
                            play_back_point_pub_cnt_ = 0;
                            replay_point_pub_cnt_ = 0;
                            led_pub_->publish(ledcomm);
                            point_pub_->publish(point_data);
                        }
                    }
                }
            } else if (obstacle_handle_mode_ == 2) { // Waiting mode
                if(ptop_collision == 2) {
                    if(collision_back_) {
                        collision_back_ = false;
                        play_back_point_pub_cnt_ = 1;
                        replay_point_pub_cnt_ = 1;
                        nav_cancel_data.nav_cancel = true;                    
                        nav_cancel_pub_->publish(nav_cancel_data);
                    } else if(!collision_back_) {
                        twist.linear.x = 0.0;
                        twist.angular.z = 0.0;
                        // ledcomm.led[0] = 0;
                        // ledcomm.led[1] = 0;
                        // ledcomm.led[2] = 1;
                        ledcomm.light[0] = 0;
                        ledcomm.light[1] = 0;
                        ledcomm.light[2] = 0;
                        led_pub_->publish(ledcomm);
                        cmd_vel_pub_->publish(twist);
                    }
                } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && (ptop_collision == 0 || ptop_collision == 1)) {
                    collision_back_ = true;
                    play_back_point_pub_cnt_ = 0;
                    replay_point_pub_cnt_ = 0;
                    // ledcomm.led[0] = 0;
                    // ledcomm.led[1] = 0;
                    // ledcomm.led[2] = 1;
                    ledcomm.light[0] = 1;
                    ledcomm.light[1] = 0;
                    ledcomm.light[2] = 0;
                    led_pub_->publish(ledcomm);
                    point_pub_->publish(point_data);
                }
            }
        } else {
            // std::cout << "backward\n";
            if(nav_spin_ == true) {
                // if(odom_z_save_ == true) {
                //     odom_z_save_ = false;
                //     odom_z_spin_save_val_ = odom_z_;
                // }

                // if (odom_z_ - odom_z_spin_save_val_ > M_PI/10) {
                //     twist.linear.x = 0.0;
                //     twist.angular.z = - 0.2;
                // } else if ((odom_z_ - odom_z_spin_save_val_ < -M_PI/10)) {
                //     twist.linear.x = 0.0;
                //     twist.angular.z = 0.2;
                // } else {
                //     if(nav_spin_cnt_ < 30) {
                //         nav_spin_cnt_++;
                //         twist.linear.x = 0.0;
                //         twist.angular.z = 0.0;       
                //     } else {
                //         nav_spin_ = false;
                //     }
                // }
                // std::cout << "nav_spin true";

                if (nav_spin_cnt_ == 0) {
                    nav_spin_cnt_++;
                    nav_cancel_data.nav_cancel = true;
                    nav_cancel_pub_->publish(nav_cancel_data);
                } else if(nav_spin_cnt_ < 80) {
                    nav_spin_cnt_++;
                    twist.linear.x = 0.0;
                    // led_pub_->publish(ledcomm);         
                    twist.angular.z = 0.8; 
                    std::cout << nav_spin_cnt_ << "spin\n";      
                } else if (nav_spin_cnt_ < 90) {
                    nav_spin_cnt_++;
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    std::cout << nav_spin_cnt_ << "stop\n"; 
                } else {
                    nav_spin_ = false;
                }
                // ledcomm.led[0] = 0;
                // ledcomm.led[1] = 1;
                // ledcomm.led[2] = 0;
                ledcomm.light[0] = 0;
                ledcomm.light[1] = 0;
                ledcomm.light[2] = 1;
                led_pub_->publish(ledcomm);
                cmd_vel_pub_->publish(twist);
            } else {
                if(obstacle_handle_mode_ == 1) { // Avoidance mode 1
                    if(avoidance_num_ == 0) { // stop until press button
                        if(frontdist_ < 500 && frontdist_ > 0) {
                            play_back_point_pub_cnt_ = 1;
                            replay_point_pub_cnt_ = 1;
                            nav_cancel_data.nav_cancel = true;
                            button_state_[0] = 0;
                            button_state_[1] = 0;
                            button_state_[2] = 0;
                            nav_cancel_pub_->publish(nav_cancel_data);
                        } else if(ptop_collision == 2) {
                            if(collision_back_) {
                                collision_back_ = false;
                                play_back_point_pub_cnt_ = 1;
                                replay_point_pub_cnt_ = 1;
                                nav_cancel_data.nav_cancel = true;
                                obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                                button_state_[0] = 0;
                                button_state_[1] = 0;
                                button_state_[2] = 0;
                                nav_cancel_pub_->publish(nav_cancel_data);
                            } else {
                                twist.linear.x = 0.0;
                                twist.angular.z = 0.0;
                                button_state_[0] = 0;
                                button_state_[1] = 0;
                                button_state_[2] = 0;
                                cmd_vel_pub_->publish(twist);
                            }
                        } else if(ptop_collision == 1) {
                            if(!collision_back_) {
                                // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                    twist.linear.x = -0.15;
                                    twist.angular.z = 0.0;
                                    // ledcomm.led[0] = 0;
                                    // ledcomm.led[1] = 1;
                                    // ledcomm.led[2] = 0;
                                    ledcomm.light[0] = 0;
                                    ledcomm.light[1] = 0;
                                    ledcomm.light[2] = 1;
                                // } else {
                                //     twist.linear.x = 0.0;
                                //     twist.angular.z = 0.0; 
                                // }
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                            }
                        } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && ptop_collision == 0) {
                            sleep(1);
                            collision_back_ = true;
                            play_back_point_pub_cnt_ = 0;
                            replay_point_pub_cnt_ = 0;
                            // ledcomm.led[0] = 0;
                            // ledcomm.led[1] = 0;
                            // ledcomm.led[2] = 1;
                            ledcomm.light[0] = 1;
                            ledcomm.light[1] = 0;
                            ledcomm.light[2] = 0;
                            led_pub_->publish(ledcomm);
                            point_pub_->publish(point_data);
                        }
                    } else if(avoidance_num_ > 3) { // try to keep avoid
                        if(frontdist_ < 500 && frontdist_ > 0) {
                        //     if(collision_back_) {
                        //         collision_back_ = false;
                        //         collision_spin_ = true;
                        //         play_back_point_pub_cnt_ = 1;
                        //         replay_point_pub_cnt_ = 1;
                        //         nav_cancel_data.nav_cancel = true;
                        //         if(obstacle_idx != -1){
                        //             obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                        //         } else {
                        //             obstacle_angle_ = OBSTACLE_ANGLE_BOUNDARY;
                        //         }

                        //         nav_cancel_pub_->publish(nav_cancel_data);
                        //     } else {
                        //         if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                        collision_back_ = false;
                                    twist.linear.x = -0.15;
                                    twist.angular.z = 0.0;
                                    // ledcomm.led[0] = 0;
                                    // ledcomm.led[1] = 1;
                                    // ledcomm.led[2] = 0;
                                    ledcomm.light[0] = 0;
                                    ledcomm.light[1] = 0;
                                    ledcomm.light[2] = 1;
                        //         } else {
                        //             twist.linear.x = 0.0;
                        //             twist.angular.z = 0.0; 
                        //         }
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                        //     }
                        } else if(ptop_collision == 2) {
                            if(collision_back_) {
                                collision_back_ = false;
                                play_back_point_pub_cnt_ = 1;
                                replay_point_pub_cnt_ = 1;
                                nav_cancel_data.nav_cancel = true;
                                obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                                nav_cancel_pub_->publish(nav_cancel_data);
                            } else {
                                // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                    twist.linear.x = -0.15;
                                    twist.angular.z = 0.0;
                                    // ledcomm.led[0] = 0;
                                    // ledcomm.led[1] = 0;
                                    // ledcomm.led[2] = 1;
                                    ledcomm.light[0] = 0;
                                    ledcomm.light[1] = 0;
                                    ledcomm.light[2] = 1;
                                // } else {
                                //     twist.linear.x = 0.0;
                                //     twist.angular.z = 0.0; 
                                // }
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                            }
                        } else if(ptop_collision == 1) {
                            if(!collision_back_) {
                                // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                    twist.linear.x = -0.15;
                                    twist.angular.z = 0.0;
                                    // ledcomm.led[0] = 0;
                                    // ledcomm.led[1] = 1;
                                    // ledcomm.led[2] = 0;
                                    ledcomm.light[0] = 0;
                                    ledcomm.light[1] = 0;
                                    ledcomm.light[2] = 1;
                                // } else {
                                //     twist.linear.x = 0.0;
                                //     twist.angular.z = 0.0; 
                                // }
                                collision_spin_ = true;
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                            }
                        } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && ptop_collision == 0) {
                            if(collision_spin_ == true) {
                                collision_back_ = true;
                                twist.linear.x = 0.0;
                                if(obstacle_angle_ > OBSTACLE_ANGLE_BOUNDARY) {
                                    twist.angular.z = 0.2;
                                    spin_cnt_++;

                                    if(spin_cnt_ > SPIN_COUNTER) {
                                        spin_cnt_ = 0;
                                        twist.angular.z = 0.0;
                                        collision_spin_ = false;
                                    }
                                } else {
                                    twist.angular.z = -0.2;
                                    spin_cnt_++;

                                    if (spin_cnt_ > SPIN_COUNTER) {
                                        spin_cnt_ = 0;
                                        twist.angular.z = 0.0;
                                        collision_spin_ = false;
                                    }
                                }
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 1;
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                            }  else {
                                sleep(1);
                                play_back_point_pub_cnt_ = 0;
                                replay_point_pub_cnt_ = 0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 0;
                                // ledcomm.led[2] = 1;
                                ledcomm.light[0] = 1;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 0;
                                led_pub_->publish(ledcomm);
                                point_pub_->publish(point_data);
                            }
                        }
                    } else if(avoidance_num_ >= 1 && avoidance_num_ <= 3) { // try to avoid a few times and stop
                        if(frontdist_ < 500 && frontdist_ > 0) {
                        //     if(collision_back_) {
                        //         collision_back_ = false;
                        //         collision_spin_ = true;
                        //         play_back_point_pub_cnt_ = 1;
                        //         replay_point_pub_cnt_ = 1;
                        //         nav_cancel_data.nav_cancel = true;
                        //         if(obstacle_idx != -1){
                        //             obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                        //         } else {
                        //             obstacle_angle_ = OBSTACLE_ANGLE_BOUNDARY;
                        //         }
                        //         avoidance_cnt_++;

                        //         if(avoidance_cnt_ > avoidance_num_) {
                        //             button_state_[0] = 0;
                        //             button_state_[1] = 0;
                        //             button_state_[2] = 0;
                        //         }

                        //         nav_cancel_pub_->publish(nav_cancel_data);
                        //     } else {
                        //         if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                        collision_back_ = false;
                                    twist.linear.x = -0.15;
                                    twist.angular.z = 0.0;
                                    // ledcomm.led[0] = 0;
                                    // ledcomm.led[1] = 1;
                                    // ledcomm.led[2] = 0;
                                    ledcomm.light[0] = 0;
                                    ledcomm.light[1] = 0;
                                    ledcomm.light[2] = 1;
                        //         } else {
                        //             twist.linear.x = 0.0;
                        //             twist.angular.z = 0.0; 
                        //         }
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                        //     }
                        } else if(ptop_collision == 2) {
                            if(collision_back_) {
                                collision_back_ = false;
                                play_back_point_pub_cnt_ = 1;
                                replay_point_pub_cnt_ = 1;
                                nav_cancel_data.nav_cancel = true;
                                obstacle_angle_ = collision_obj_info_vector[obstacle_idx].angle;
                                avoidance_cnt_++;

                                if(avoidance_cnt_ > avoidance_num_) {
                                    button_state_[0] = 0;
                                    button_state_[1] = 0;
                                    button_state_[2] = 0;
                                }

                                nav_cancel_pub_->publish(nav_cancel_data);
                            } else {
                                // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                    twist.linear.x = -0.15;
                                    twist.angular.z = 0.0;
                                    // ledcomm.led[0] = 0;
                                    // ledcomm.led[1] = 1;
                                    // ledcomm.led[2] = 0;
                                    ledcomm.light[0] = 0;
                                    ledcomm.light[1] = 0;
                                    ledcomm.light[2] = 1;
                                // } else {
                                //     twist.linear.x = 0.0;
                                //     twist.angular.z = 0.0; 
                                // }
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                            }
                        } else if(ptop_collision == 1) {
                            if(!collision_back_) {
                                // if(backdist_ > ULTRASONIC_LIMIT_DISTANCE) { sensor is unrelialbe
                                    twist.linear.x = -0.15;
                                    twist.angular.z = 0.0;
                                    // ledcomm.led[0] = 0;
                                    // ledcomm.led[1] = 1;
                                    // ledcomm.led[2] = 0;
                                    ledcomm.light[0] = 0;
                                    ledcomm.light[1] = 0;
                                    ledcomm.light[2] = 1;
                                // } else {
                                //     twist.linear.x = 0.0;
                                //     twist.angular.z = 0.0; 
                                // }
                                collision_spin_ = true;
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                            }
                        } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && ptop_collision == 0) {
                            if(collision_spin_ == true) {
                                collision_back_ = true;
                                twist.linear.x = 0.0;
                                if(obstacle_angle_ > OBSTACLE_ANGLE_BOUNDARY) {
                                    twist.angular.z = 0.2;
                                    spin_cnt_++;

                                    if(spin_cnt_ > SPIN_COUNTER) {
                                        spin_cnt_ = 0;
                                        twist.angular.z = 0.0;
                                        collision_spin_ = false;
                                    }
                                } else {
                                    twist.angular.z = -0.2;
                                    spin_cnt_++;

                                    if (spin_cnt_ > SPIN_COUNTER) {led_pub_->publish(ledcomm);
                                        spin_cnt_ = 0;
                                        twist.angular.z = 0.0;
                                        collision_spin_ = false;
                                    }
                                }
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 1;
                                // ledcomm.led[2] = 0;
                                ledcomm.light[0] = 0;
                                ledcomm.light[1] = 1;
                                ledcomm.light[2] = 0;
                                led_pub_->publish(ledcomm);
                                cmd_vel_pub_->publish(twist);
                            } else {
                                // collision_spin_ = false;
                                sleep(1);
                                play_back_point_pub_cnt_ = 0;
                                replay_point_pub_cnt_ = 0;
                                // ledcomm.led[0] = 0;
                                // ledcomm.led[1] = 0;
                                // ledcomm.led[2] = 1;
                                ledcomm.light[0] = 1;
                                ledcomm.light[1] = 0;
                                ledcomm.light[2] = 0;
                                led_pub_->publish(ledcomm);
                                point_pub_->publish(point_data);
                            }
                        }
                    }
                } else if (obstacle_handle_mode_ == 2) { // Waiting mode
                    if(ptop_collision == 2) {
                        if(collision_back_) {
                            collision_back_ = false;
                            play_back_point_pub_cnt_ = 1;
                            replay_point_pub_cnt_ = 1;
                            nav_cancel_data.nav_cancel = true;                    
                            nav_cancel_pub_->publish(nav_cancel_data);
                        } else if(!collision_back_) {
                            twist.linear.x = 0.0;
                            twist.angular.z = 0.0;
                            // ledcomm.led[0] = 1;
                            // ledcomm.led[1] = 0;
                            // ledcomm.led[2] = 0;
                            ledcomm.light[0] = 0;
                            ledcomm.light[1] = 1;
                            ledcomm.light[2] = 0;
                            cmd_vel_pub_->publish(twist);
                        }
                    } else if((play_back_point_pub_cnt_ || replay_point_pub_cnt_) && (ptop_collision == 0 || ptop_collision == 1)) {
                        collision_back_ = true;
                        play_back_point_pub_cnt_ = 0;
                        replay_point_pub_cnt_ = 0;
                        // ledcomm.led[0] = 0;
                        // ledcomm.led[1] = 0;
                        // ledcomm.led[2] = 1;
                        ledcomm.light[0] = 1;
                        ledcomm.light[1] = 0;
                        ledcomm.light[2] = 0;
                        point_pub_->publish(point_data);
                        led_pub_->publish(ledcomm);
                    }
                }
            }
        }
    }
    led_pub_->publish(ledcomm);
}

// void clusteringNode::map_out_pub(std::vector<ObjInfo> obj_info_vector, int id)
// {
//     sensor_msgs::msg::PointCloud out_msg;
//     out_msg.header.frame_id = "base_scan";
//     // thread
//     if(id <= 0)
//     {
//         for (size_t i = 0; i < map_cloud->points.size(); i++)
//         {
//             geometry_msgs::msg::Point32 single_point;
//             single_point.x = map_cloud->points[i].x;
//             single_point.y = map_cloud->points[i].y;
//             out_msg.points.push_back(single_point);
//         }
//     }
//     else
//     {
//         for (size_t i = 0; i < map_cloud->points.size(); i++)
//         {
//             geometry_msgs::msg::Point32 single_point;
//             single_point.x = map_cloud->points[i].x;
//             single_point.y = map_cloud->points[i].y;
//             if(single_point.x < obj_info_vector[id-1].position[0] - 0.3
//             || single_point.x > obj_info_vector[id-1].position[0] + 0.3
//             || single_point.y < obj_info_vector[id-1].position[1] - 0.3
//             || single_point.y > obj_info_vector[id-1].position[1] + 0.3)
//             {
//                 out_msg.points.push_back(single_point);
//             }
//             else
//             {
//                 single_point.x = 0;
//                 single_point.y = 0;
//             }
//         }
//     }
//     if (out_msg.points.size() > 0)
//     {
//         sensor_msgs::msg::PointCloud2 out_msg2;
//         sensor_msgs::convertPointCloudToPointCloud2(out_msg, out_msg2);
//         out_msg2.header.stamp = now();
//         map_out_pub->publish(out_msg2);
//     }
// }

void clusteringNode::footprint_marker(std::vector<ObjInfo> obj_info, int map_obstacle_idx_)
{
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "base_scan";
    marker_msg.header.stamp = rclcpp::Clock().now();
    marker_msg.ns = "sphere";
    marker_msg.id = 0;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_array_msg.markers.push_back(marker_msg);
    footprint_pub_->publish(marker_array_msg);
    marker_array_msg.markers.clear();

    for (size_t i = 1; i <= cmdvel_index_; i++) {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "map";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "footprint";
        marker_msg.id = i;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = point_pos_x_[i];
        marker_msg.pose.position.y = point_pos_y_[i];
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.4;
        marker_msg.scale.y = 0.4;
        marker_msg.scale.z = 0.4;
        marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        marker_msg.color.r = 1.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 0.0;
        marker_array_msg.markers.push_back(marker_msg);
    }

    for (size_t i = 1; i <= cmdvel_index_; i++) {
        visualization_msgs::msg::Marker text_msg;
        text_msg.header.frame_id = "map";
        text_msg.header.stamp = rclcpp::Clock().now();
        text_msg.ns = "text";
        text_msg.id = i;
        text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_msg.action = visualization_msgs::msg::Marker::ADD;
        text_msg.pose.position.x = point_pos_x_[i] + 0.6;
        text_msg.pose.position.y = point_pos_y_[i];
        text_msg.scale.z = 0.2;
        text_msg.color.a = 1.0; // Don't forget to set the alpha!
        text_msg.color.r = 0.0;
        text_msg.color.g = 1.0;
        text_msg.color.b = 0.0;

        std::string text_string = "[" + std::to_string(i) + "]";
        text_msg.text = text_string;

        marker_array_msg.markers.push_back(text_msg);
    }

    for (size_t i = 0; i < obj_info.size(); i++) {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "map";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "obj";
        marker_msg.id = obj_info[i].id;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position.x = obj_info[i].position[0]*cos(odom_z_) - obj_info[i].position[1]*sin(odom_z_) + odom_x_; // obj_info[i].position[0] + odom_x_; // point_pos_x_ => obj_info
        marker_msg.pose.position.y = obj_info[i].position[0]*sin(odom_z_) + obj_info[i].position[1]*cos(odom_z_) + odom_y_; //obj_info[i].position[1] + odom_y_; // point_pos_x_ => obj_info
        marker_msg.pose.position.z = 0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.3;
        marker_msg.scale.y = 0.3;
        marker_msg.scale.z = 0.3;
        marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        if(i == map_obstacle_idx_) {
            marker_msg.color.r = 1.0;
            marker_msg.color.g = 0.0;
            marker_msg.color.b = 0.0;
        } else {
            marker_msg.color.r = 1.0;
            marker_msg.color.g = 1.0;
            marker_msg.color.b = 0.0;
        }
        marker_array_msg.markers.push_back(marker_msg);
    }

    for (size_t i = 0; i < obj_info.size(); i++) {
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

        std::string text_string = "[" + std::to_string(obj_info[i].id) + "]";
        text_msg.text = text_string;marker_array_msg;

        marker_array_msg.markers.push_back(text_msg);
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
    box_point_msg.x = roi_center_x_ - ROI_WIDTH;
    box_point_msg.y = roi_center_y_ - ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);
    box_point_msg.x = roi_center_x_ + ROI_WIDTH;
    box_point_msg.y = roi_center_y_ - ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);
    box_point_msg.x = roi_center_x_ + ROI_WIDTH;
    box_point_msg.y = roi_center_y_ + ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);
    box_point_msg.x = roi_center_x_ - ROI_WIDTH;
    box_point_msg.y = roi_center_y_ + ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);
    box_point_msg.x = roi_center_x_ - ROI_WIDTH;
    box_point_msg.y = roi_center_y_ - ROI_HEIGHT;
    box_msg_roi.points.push_back(box_point_msg);
    marker_array_msg.markers.push_back(box_msg_roi);

    footprint_pub_->publish(marker_array_msg);
    marker_array_msg.markers.clear();
}

// void clusteringNode::pub_marker(std::vector<ObjInfo> obj_info)
// {
//     double distance;
//     //////////////////////////////////////////////////////////////
//     // clear markers
//     //////////////////////////////////////////////////////////////
//     visualization_msgs::msg::MarkerArray marker_array_msg;
//     visualization_msgs::msg::Marker marker_msg;
//     marker_msg.header.frame_id = "base_scan";
//     marker_msg.header.stamp = rclcpp::Clock().now();
//     marker_msg.ns = "obj";
//     marker_msg.id = 0;
//     marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;
//     marker_array_msg.markers.push_back(marker_msg);
//     rviz_pub->publish(marker_array_msg);
//     marker_array_msg.markers.clear();
//     //////////////////////////////////////////////////////////////
//     // publish markers
//     //////////////////////////////////////////////////////////////
//     if (obj_info.size() == 0)
//         return;
//     for (size_t i = 0; i < obj_info.size(); i++)
//     {
//         visualization_msgs::msg::Marker marker_msg;
//         marker_msg.header.frame_id = "base_scan";
//         marker_msg.header.stamp = rclcpp::Clock().now();
//         marker_msg.ns = "obj";
//         marker_msg.id = obj_info[i].id;
//         marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
//         marker_msg.action = visualization_msgs::msg::Marker::ADD;
//         marker_msg.pose.position.x = obj_info[i].position[0];
//         marker_msg.pose.position.y = obj_info[i].position[1];
//         marker_msg.pose.position.z = 0;
//         marker_msg.pose.orientation.w = 1.0;
//         marker_msg.scale.x = 0.3;
//         marker_msg.scale.y = 0.3;
//         marker_msg.scale.z = 0.3;
//         marker_msg.color.a = 1.0; // Don't forget to set the alpharoi_center_x_!
//         marker_msg.color.r = 1.0;
//         marker_msg.color.g = 1.0;
//         marker_msg.color.b = 0.0;
//         marker_array_msg.markers.push_back(marker_msg);
//     }
//     for (size_t i = 0; i < cmdvel_index_; i++)
//     {
//         visualization_msgs::msg::Marker marker_msg;
//         marker_msg.header.frame_id = "map"; //odom
//         marker_msg.header.stamp = rclcpp::Clock().now();
//         marker_msg.ns = "footprint";
//         marker_msg.id = i;
//         marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
//         marker_msg.action = visualization_msgs::msg::Marker::ADD;
//         marker_msg.pose.position.x = point_pos_x_[i];
//         marker_msg.pose.position.y = point_pos_y_[i];
//         marker_msg.pose.position.z = 0;
//         marker_msg.pose.orientation.w = 1.0;
//         marker_msg.scale.x = 0.1;
//         marker_msg.scale.y = 0.1;
//         marker_msg.scale.z = 0.1;
//         marker_msg.color.a = 1.0; // Don't forget to set the alpha!
//         marker_msg.color.r = 0.0;
//         marker_msg.color.g = 1.0;
//         marker_msg.color.b = 0.0;
//         marker_array_msg.markers.push_back(marker_msg);
//     }
//     visualization_msgs::msg::Marker box_msg_roi;
//     box_msg_roi.header.frame_id = "base_scan";
//     box_msg_roi.header.stamp = rclcpp::Clock().now();
//     box_msg_roi.ns = "roi";
//     //box_msg_roi.id = obj_info_vector[i].id;
//     box_msg_roi.type = visualization_msgs::msg::Marker::LINE_STRIP;
//     box_msg_roi.action = visualization_msgs::msg::Marker::ADD;
//     box_msg_roi.pose.orientation.w = 1.0;
//     box_msg_roi.scale.x = 0.05;
//     box_msg_roi.color.a = 1.0; // Don't forget to set the alpha!
//     box_msg_roi.color.r = 0.0;
//     box_msg_roi.color.g = 1.0;
//     box_msg_roi.color.b = 1.0;
//     geometry_msgs::msg::Point box_point_msg;
//     box_point_msg.x = roi_center_x_ - ROI_WIDTH;
//     box_point_msg.y = roi_center_y_ - ROI_HEIGHT;
//     box_msg_roi.points.push_back(box_point_msg);
//     box_point_msg.x = roi_center_x_ + ROI_WIDTH;
//     box_point_msg.y = roi_center_y_ - ROI_HEIGHT;
//     box_msg_roi.points.push_back(box_point_msg);
//     box_point_msg.x = roi_center_x_ + ROI_WIDTH;
//     box_point_msg.y = roi_center_y_ + ROI_HEIGHT;
//     box_msg_roi.points.push_back(box_point_msg);
//     box_point_msg.x = roi_center_x_ - ROI_WIDTH;
//     box_point_msg.y = roi_center_y_ + ROI_HEIGHT;
//     box_msg_roi.points.push_back(box_point_msg);
//     box_point_msg.x = roi_center_x_ - ROI_WIDTH;
//     box_point_msg.y = roi_center_y_ - ROI_HEIGHT;
//     box_msg_roi.points.push_back(box_point_msg);
//     marker_array_msg.markers.push_back(box_msg_roi);
//     //////////////////////////////////////////////////////////////
//     // Text marker
//     //////////////////////////////////////////////////////////////
//     for (size_t i = 0; i < obj_info.size(); i++)
//     {
//         visualization_msgs::msg::Marker text_msg;
//         text_msg.header.frame_id = "base_scan";
//         text_msg.header.stamp = rclcpp::Clock().now();
//         text_msg.ns = "text";
//         text_msg.id = obj_info[i].id;
//         text_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         text_msg.action = visualization_msgs::msg::Marker::ADD;
//         text_msg.pose.position.x = obj_info[i].position[0] - obj_info[i].width/2;
//         text_msg.pose.position.y = obj_info[i].position[1] + obj_info[i].height/2 + 0.2;
//         text_msg.scale.z = 0.2;
//         text_msg.color.a = 1.0; // Don't forget to set the alpha!
//         text_msg.color.r = 0.0;
//         text_msg.color.g = 1.0;
//         text_msg.color.b = 0.0;
//         distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0] + obj_info[i].position[1] * obj_info[i].position[1]);
//         std::string text_string = "[" + std::to_string(obj_info[i].id) + "]";
//         text_msg.text = text_string;
//         marker_array_msg.markers.push_back(text_msg);
//     }
//     rviz_pub->publish(marker_array_msg);
// }

void followingSoundPlay(void) // (const char* filename)
{
    sf::SoundBuffer go_buffer, stop_buffer, shutdown_buffer;

    if (!go_buffer.loadFromFile("/home/robot/robot_ws/audio/Bolero.wav") || !stop_buffer.loadFromFile("/home/robot/robot_ws/audio/stopbeep.wav")
    || !shutdown_buffer.loadFromFile("/home/robot/robot_ws/audio/Turning_Off.wav")) { // ~/Desktop/test/audio/beep.wav
        std::cerr << "load failed!";
        return;
    }

    sf::Sound go_sound, stop_sound, shutdown_sound;

    go_sound.setBuffer(go_buffer);
    stop_sound.setBuffer(stop_buffer);
    shutdown_sound.setBuffer(shutdown_buffer);

    while(g_sound_play_thread_controller) {
        if(g_shutdown == 1) {
            shutdown_sound.play();

            while (shutdown_sound.getStatus() == sf::Sound::Playing) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            system("sudo /sbin/shutdown -h now");
        }

        if(g_target_detecting_state != g_target_before_state) {
            if(g_target_detecting_state == 0) {
                go_sound.stop();
                stop_sound.stop();
            } else if(g_target_detecting_state == 1) {
                stop_sound.stop();
                go_sound.play();
            } else if(g_target_detecting_state == 2) {
                go_sound.stop();
                stop_sound.play();
            }
        } else {
            if(g_target_detecting_state == 0) {
                go_sound.stop();
                stop_sound.stop();
            } else if (g_target_detecting_state == 1 && go_sound.getStatus() != sf::Sound::Playing) {
                go_sound.play();
                stop_sound.stop();
            } else if (g_target_detecting_state == 2 && stop_sound.getStatus() != sf::Sound::Playing) {
                stop_sound.play();
                go_sound.stop();
            }
        }
        g_target_before_state = g_target_detecting_state;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 반복 체크 간격
    }
}

int main(int nArgc,const char* pszArgv[])
{
    sf::SoundBuffer booting_buffer;

    if (!booting_buffer.loadFromFile("/home/robot/robot_ws/audio/booting.wav")) { // ~/Desktop/test/audio/beep.wav
        std::cerr << "load failed!";
    }

    sf::Sound booting_sound;
    booting_sound.setBuffer(booting_buffer);
    booting_sound.play();

    rclcpp::init(nArgc, pszArgv);
    // rclcpp::spin(std::make_shared<clusteringNode>());

    auto node = std::make_shared<clusteringNode>();

    std::thread ros_thread([&](){ rclcpp::spin(node); });
    std::thread play_thread(followingSoundPlay);

    if(play_thread.joinable()) {
        play_thread.detach();
    }

    if(ros_thread.joinable()) {
        ros_thread.join();
    }
    g_sound_play_thread_controller = 0;

    rclcpp::shutdown();

    return 0;
}