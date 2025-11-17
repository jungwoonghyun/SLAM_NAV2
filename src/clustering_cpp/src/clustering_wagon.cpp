#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <ctime>
#include <thread>
// #include <pcl/ModelCoefficients.h>
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

using namespace std::chrono_literals;

#define ROI_WIDTH                   0.3
#define ROI_HEIGHT                  0.3
#define ROI_RANGE                   0.25
// #define STARTING_DISTANCE           0.7
#define COLLISION_DISTANCE          0.8

int g_target_detecting_state = 0;
int before_state = 0;
int sound_play_thread_controller = 1;

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
        : Node("clustering_node")
        {
            // Qos depth = 10
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

            laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 
            rclcpp::SensorDataQoS(), std::bind(&clusteringNode::msgCallback_lidar, this, std::placeholders::_1));
            controller_state_sub = this->create_subscription<message_interface::msg::Mode>("mode_selector_pub", 
            rclcpp::SensorDataQoS(), std::bind(&clusteringNode::msgCallback_state, this, std::placeholders::_1));//#

            cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
            map_out_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("out_cloud", qos_profile);
            rviz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("obj_detect_marker_array", 10);
            following_state_pub = this->create_publisher<message_interface::msg::Mode>("following_state", qos_profile);
            // state_pub = this->create_publisher<message_interface::msg::Mode>("mode_selector_sub", 10);//#
            // controller_pub = this->create_publisher<message_interface::msg::Iscoast>("is_coast", 10);

            timer = this->create_wall_timer(100ms, std::bind(&clusteringNode::timer_callback, this));
        }

        ~clusteringNode()
        {

        }

        int target_id = -2;
        double roi_center_x = 0.5, roi_center_y = 0;
        int controller_state = 0;
        int count = 0;
        int target_detecting_state = 0, target_detect_counter = 0;
        // bool remote_controller=false;
        double prev_target_distance = 0, prev_target_angle = 0;
        bool following_start = false;

    private:

        std::chrono::steady_clock::time_point start_time;

        // variable
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
        rclcpp::Subscription<message_interface::msg::Mode>::SharedPtr controller_state_sub;//#
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_out_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub;
        rclcpp::Publisher<message_interface::msg::Mode>::SharedPtr following_state_pub;
        // rclcpp::Publisher<message_interface::msg::Mode>::SharedPtr state_pub;//#
        // rclcpp::Publisher<message_interface::msg::Iscoast>::SharedPtr controller_pub;
        rclcpp::TimerBase::SharedPtr timer;

        // function
        void msgCallback_lidar(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void msgCallback_state(const message_interface::msg::Mode & msg);//#
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

        if(std::isnan(range)) range = 0.0;
        
        double lidar_x = range * cos(angle);
        double lidar_y = range * sin(angle);

        if( sqrt(pow(roi_center_x - lidar_x, 2) + pow(roi_center_y - lidar_y, 2)) < ROI_RANGE
        || sqrt(pow(roi_center_x - lidar_x, 2) + pow(roi_center_y - lidar_y, 2)) > ROI_RANGE + 0.3)
            filtered_lidar_cloud->points.push_back(pcl::PointXYZ(lidar_x, lidar_y, 0.0));

        lidar_cloud->points.push_back(pcl::PointXYZ(lidar_x, lidar_y, 0.0));

    }
    filtered_lidar_cloud->width = filtered_lidar_cloud->points.size();
    filtered_lidar_cloud->height = 1;
    filtered_lidar_cloud->is_dense = true;

    lidar_cloud->width = lidar_cloud->points.size();
    lidar_cloud->height = 1;
    lidar_cloud->is_dense = true;
}

void clusteringNode::msgCallback_state(const message_interface::msg::Mode & msg)
{
    controller_state = msg.modenum;
    // RCLCPP_INFO(this->get_logger(), "Receive %d mode num from esp, Change\r\n", state);
}

//////////////////////////////////////

void clusteringNode::timer_callback()
{
    double linear_ = 0.0, angular_ = 0.0;
    double target_distance = 0, target_angle = 0;
    int lost, collision = 1;
    double weight_linear = 0.95, weight_angular = 0.95;
    int target_lost;

    geometry_msgs::msg::Twist twist;
    message_interface::msg::Mode c_mode;
    message_interface::msg::Mode target_state;
    message_interface::msg::Iscoast is_motor_coast;

    std::vector<ObjInfo> obj_info_vector;
    std::vector<ObjInfo> collision_obj_info_vector;

    if (filtered_lidar_cloud->points.size() == 0)
    {
        return;
    }
    makeRefMap();

    mapDownsampling();

    filteredEuclideanClustering(obj_info_vector); // make cluster and give info(like id, width...)
                                                  // to each cluster
    euclideanClustering(collision_obj_info_vector);

    // target detecting or not
    if(target_id == -2)
    {
        target_id = targetDetect(obj_info_vector); // target_id = -1 detect failed
        // remote_controller = true;

        // std::cout << "remote_controller:" << remote_controller << "id" << target_id << std::endl;
    }
    else
    {
        if(controller_state == 0) //&& target_detect_counter == 0
        {
            linear_ = 0.0;
            angular_ = 0.0;
            roi_center_x = 0.5;
            roi_center_y = 0;
            target_detecting_state = 0;
            target_detect_counter = 0;
            target_id = -2;
            following_start = false;
            std::cout << "state 0   " << g_target_detecting_state << std::endl;
        }
        else if(controller_state == 1)
        {
            target_id = roiDetect(obj_info_vector); // detect target and get id

            if (target_id != -1 && target_detecting_state != 2)// target_id != -1           && target_detecting_state == 0
            {
                target_distance = sqrt(pow(obj_info_vector[target_id].position[0],2) + pow(obj_info_vector[target_id].position[1],2));
                target_angle = atan2(obj_info_vector[target_id].position[1], obj_info_vector[target_id].position[0]);
                
                linear_ = 2.5 * (target_distance - COLLISION_DISTANCE); // (2.5 ~ 2.0)
                angular_ = target_angle;
                std::cout << linear_ <<"\n";
                if (linear_ > 1.3)
                {
                    linear_ = 1.3;
                }
                else if (linear_ <= 0)
                {
                    linear_ = 0;
                }
                
                if(linear_ == 0)
                {
                    angular_ = angular_/1.25;
                }
                else
                {
                    if (abs(angular_) < M_PI/150)
                    {
                        angular_ = 0.0;
                    }
                    else if (angular_ > M_PI/3)
                    {
                        angular_ = M_PI/3;
                    }
                    else if (angular_ < -M_PI/3)
                    {
                        angular_ = -M_PI/3;
                    }                
                    else
                    {
                        angular_ = angular_; // (compensatation)
                    }
                }

                prev_target_distance = linear_;
                prev_target_angle = angular_;

                target_detecting_state = 1;
                target_detect_counter == 1; // target detect counter
                std::cout << "state 1  " << g_target_detecting_state << std::endl;
            }
            else if(target_id == -1 && target_detecting_state == 1)
            {
                linear_ = 0.0;
                angular_ = 0.0;
                roi_center_x = 0.5;
                roi_center_y = 0;
                target_detecting_state = 2;
                std::cout << "state 2 start\n";
            }
            else if(target_id == -1 && target_detecting_state == 2) 
            {
                linear_ = 0.0;
                angular_ = 0.0;
                roi_center_x = 0.5;
                roi_center_y = 0;
                std::cout << "state 2  " << g_target_detecting_state << std::endl;
            }
            else
            {
                std::cout << "counter state 1, target detect state else\n";
                std::cout << "id " << target_id << " detect state " << target_detecting_state << "\n";
            }
        }

        g_target_detecting_state = target_detecting_state;

        // collision = collisionAvoidance(collision_obj_info_vector);

        // if(collision == 1)
        // {
        //     linear_ = 0.0;
        //     angular_ = 0.0;
        // }
        // else
        // {
        //     linear_ = linear_;
        //     angular_ = angular_;
        // }
    }

    if(controller_state == 1)
    {
        twist.angular.z = angular_;
        twist.linear.x = linear_;
        cmd_vel_pub->publish(twist);
        target_state.modenum = target_detecting_state;
        count = 0;
    }
    else
    {
        angular_ = 0;
        linear_ = 0;
        twist.angular.z = angular_;
        twist.linear.x = linear_;
        cmd_vel_pub->publish(twist);

        target_state.modenum = target_detecting_state;
    }

    following_state_pub->publish(target_state);

    // auto now = std::chrono::system_clock::now();
    // std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    // auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    // std::tm now_tm = *std::localtime(&now_time_t);

    // std::cout << "target distance, " << target_distance << "," << std::put_time(&now_tm, "%Y-%m-%d %H %M %S,         ") 
    // << std::setw(3) << std::setfill('0') << now_ms.count() << "\n";
    // std::cout << "distance " << target_distance << " ,linear_ " << linear_  << " ,id" << target_id << std::endl;

    mapOutPub(obj_info_vector, target_id+1);
    pubMarker(obj_info_vector);
    revMakeRefMap();
}

void clusteringNode::makeRefMap(void)
{
    // thread
    for (size_t i = 0; i < filtered_lidar_cloud->points.size(); i++)
    {
        filtered_map_cloud->points.push_back(filtered_lidar_cloud->points[i]);
    }
    filtered_map_cloud->width = filtered_map_cloud->points.size();
    filtered_map_cloud->height = 1;
    filtered_map_cloud->is_dense = true;

    for (size_t i = 0; i < lidar_cloud->points.size(); i++)
    {
        map_cloud->points.push_back(lidar_cloud->points[i]);
    }
    map_cloud->width = map_cloud->points.size();
    map_cloud->height = 1;
    map_cloud->is_dense = true;
}

void clusteringNode::revMakeRefMap(void)
{
    for(size_t i = 0; i < filtered_lidar_cloud->points.size(); i++)
    {
        filtered_map_cloud->points.clear();
    }
    filtered_map_cloud->width = filtered_map_cloud->points.size();
    filtered_map_cloud->height = -1;
    filtered_map_cloud->is_dense = false;

    for(size_t i = 0; i < lidar_cloud->points.size(); i++)
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
    sor.setInputCloud(lidar_cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*filtered_map_cloud);
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

    if(!filtered_map_cloud->empty())
        tree->setInputCloud (filtered_map_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (0.3);
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (filtered_map_cloud);
    ec.extract (cluster_indices);

    int cluster_index = 0;

    for (const auto& cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) 
        {
            cloud_cluster->push_back((*filtered_map_cloud)[idx]);
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

    ec.setClusterTolerance (0.2);
    ec.setMinClusterSize (2);
    ec.setMaxClusterSize (100);
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

    if(target_detecting_state == 0)
    {
        for(size_t i = 0; i < obj_info.size(); i++)
        {
            if(sqrt(pow(obj_info[i].position[0] - roi_center_x, 2) + pow(obj_info[i].position[1] - roi_center_y, 2)) < ROI_RANGE
            && obj_info[i].position[0] >= abs(sqrt(3)*obj_info[i].position[1]))
            {
                roi_center_x = obj_info[i].position[0];
                roi_center_y = obj_info[i].position[1];
                target_lost = i;

                if(target_detecting_state == 2)
                {
                    roi_center_x = 0.5;
                    roi_center_y = 0;
                }

                return target_lost;
            }
            else
            {
                target_lost = -1;
            }
        }
    }
    else
    {
        for(size_t i = 0; i < obj_info.size(); i++)
        {
            if(sqrt(pow(obj_info[i].position[0] - roi_center_x, 2) + pow(obj_info[i].position[1] - roi_center_y, 2)) < ROI_RANGE)
            {
                roi_center_x = obj_info[i].position[0];
                roi_center_y = obj_info[i].position[1];
                target_lost = i;

                if(target_detecting_state == 2)
                {
                    roi_center_x = 0.5;
                    roi_center_y = 0;
                }

                return target_lost;
            }
            else
            {
                target_lost = -1;
            }
        }
    }

    return target_lost;
}

int clusteringNode::collisionAvoidance(std::vector<ObjInfo> obj_info)
{
    int avoidance = 0, id =- 1;
    double distance, collision_distance_following = COLLISION_DISTANCE;

    for(size_t i = 0; i < obj_info.size(); i++)
    {
        // if (following_start == false)
        // {
        //     distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0]
        //     + obj_info[i].position[1] * obj_info[i].position[1]);

        //     if(distance < collision_distance_starting
        //     && sqrt(1/2) * obj_info[i].position[0] > obj_info[i].position[1])
        //     {
        //         collision_distance_starting = distance;
        //         following_start = true;
        //     }
        // }
        // else if (following_start == true)
        // {
            distance = sqrt(obj_info[i].position[0] * obj_info[i].position[0]
            + obj_info[i].position[1] * obj_info[i].position[1]);

            if(distance < collision_distance_following) // && sqrt(1/2) * obj_info[i].position[0] > abs(obj_info[i].position[1])
            {
                collision_distance_following = distance;
            }
        // }
    }

    if(collision_distance_following < COLLISION_DISTANCE+0.1) avoidance = 1; // collision_distance_starting < STARTING_DISTANCE+0.01 || 
    else avoidance = 0;
    
    std::cout << "avoidance " << avoidance << "\n";


    return avoidance; // 1 means collision could happen by any other objects, 0 is good to move
}

void clusteringNode::mapOutPub(std::vector<ObjInfo> obj_info_vector, int id)
{
    sensor_msgs::msg::PointCloud out_msg;
    out_msg.header.frame_id = "base_scan";

    // thread
    if(id <= 0)
    {
        for (size_t i = 0; i < filtered_map_cloud->points.size(); i++)
        {
            geometry_msgs::msg::Point32 single_point;
            single_point.x = filtered_map_cloud->points[i].x;
            single_point.y = filtered_map_cloud->points[i].y;

            out_msg.points.push_back(single_point);
        }
    }
    else
    {
        for (size_t i = 0; i < filtered_map_cloud->points.size(); i++)
        {
            geometry_msgs::msg::Point32 single_point;
            single_point.x = filtered_map_cloud->points[i].x;
            single_point.y = filtered_map_cloud->points[i].y;

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
    // 0,0
    //////////////////////////////////////////////////////////////
    // visualization_msgs::msg::Marker marker_zero;
    // marker_zero.header.frame_id = "base_scan";
    // marker_zero.header.stamp = rclcpp::Clock().now();
    // marker_zero.ns = "obj";
    // marker_zero.type = visualization_msgs::msg::Marker::SPHERE;
    // marker_zero.action = visualization_msgs::msg::Marker::ADD;
    // marker_zero.pose.position.x = 0;
    // marker_zero.pose.position.y = 0;
    // marker_zero.pose.position.z = 0;
    // marker_zero.pose.orientation.w = 1.0;
    // marker_zero.scale.x = 0.3;
    // marker_zero.scale.y = 0.3;
    // marker_zero.scale.z = 0.3;
    // marker_zero.color.a = 1.0; // Don't forget to set the alpha!
    // marker_zero.color.r = 1.0;
    // marker_zero.color.g = 1.0;
    // marker_zero.color.b = 0.0;

    // marker_array_msg.markers.push_back(marker_zero);

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
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE; //SPHERE
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
    // Cluster Size marker
    //////////////////////////////////////////////////////////////

    // for (size_t i = 0; i < obj_info.size(); i++)
    // {
    //     visualization_msgs::msg::Marker cluster_size;
    //     cluster_size.header.frame_id = "base_scan";
    //     cluster_size.header.stamp = rclcpp::Clock().now();
    //     cluster_size.ns = "size";
    //     cluster_size.id = obj_info[i].id;
    //     cluster_size.type = visualization_msgs::msg::Marker::LINE_STRIP;
    //     cluster_size.action = visualization_msgs::msg::Marker::ADD;
    //     cluster_size.pose.position.x = obj_info[i].position[0];
    //     cluster_size.pose.position.y = obj_info[i].position[1];
    //     cluster_size.pose.position.z = 0;
    //     cluster_size.pose.orientation.w = 1.0;
    //     cluster_size.scale.x = 0.01;
    //     cluster_size.color.a = 1.0; // Don't forget to set the alpha!
    //     cluster_size.color.r = 0.0;
    //     cluster_size.color.g = 1.0;
    //     cluster_size.color.b = 0.0;

    //     geometry_msgs::msg::Point cluster_point_msg;

    //     cluster_point_msg.x = obj_info[i].leftbottom[0];
    //     cluster_point_msg.y = obj_info[i].leftbottom[1];
    //     cluster_size.points.push_back(cluster_point_msg);

    //     cluster_point_msg.x = obj_info[i].righttop[0];
    //     cluster_point_msg.y = obj_info[i].leftbottom[1];
    //     cluster_size.points.push_back(cluster_point_msg);

    //     cluster_point_msg.x = obj_info[i].righttop[0];
    //     cluster_point_msg.y = obj_info[i].righttop[1];
    //     cluster_size.points.push_back(cluster_point_msg);

    //     cluster_point_msg.x = obj_info[i].leftbottom[0];
    //     cluster_point_msg.y = obj_info[i].righttop[1];
    //     cluster_size.points.push_back(cluster_point_msg);

    //     cluster_point_msg.x = obj_info[i].leftbottom[0];
    //     cluster_point_msg.y = obj_info[i].leftbottom[1];
    //     cluster_size.points.push_back(cluster_point_msg);

    //     marker_array_msg.markers.push_back(cluster_size);
    // }

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
    sf::SoundBuffer go_buffer, stop_buffer;

    if (!go_buffer.loadFromFile("/home/robot/robot_ws/audio/Bolero.wav") || !stop_buffer.loadFromFile("/home/robot/robot_ws/audio/stopbeep.wav")) // ~/Desktop/test/audio/beep.wav 
    {
        std::cerr << "load failed!";
    }

    sf::Sound go_sound, stop_sound;

    go_sound.setBuffer(go_buffer);
    stop_sound.setBuffer(stop_buffer);

    while(sound_play_thread_controller)
    {
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
                // stop_sound.setVolume();
            }
        }

        before_state = g_target_detecting_state;
    }
}

int main(int nArgc,const char* pszArgv[])
{
    rclcpp::init(nArgc, pszArgv);

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