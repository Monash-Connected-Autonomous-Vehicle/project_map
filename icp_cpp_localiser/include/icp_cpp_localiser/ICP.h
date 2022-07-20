#ifndef ICP3D_ICP_H
#define ICP3D_ICP_H
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <string>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

class ICP3D : public rclcpp::Node
{
    public:
         ICP3D();

    private:
        //using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg); //point cloud callback
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg); //imu data callback
        void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg); //imu data callback
        
        void cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //crops cloud using box filter
        void removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //removes noise using Statistical outlier removal
        void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //downsampling the point cloud using Voxelgrid
        void removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr); //ground removal using RANSAC
        void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //filtering the point cloud

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
  
        pcl::PointCloud<pcl::PointXYZ> _map_cloud; //point cloud of the pcd map
        Eigen::Matrix4f prev_transformation; //cumulative transformation until the previous time instance
        bool is_initial, is_imu_start; //boolean to tell if this is 1st iteration of the algo and start of imu reading
        double _prev_acc, _curr_acc; //accleration in consecutive time stamps
        double _prev_imu_time, _curr_imu_time; //consecutive time stamps of imu data 
        double _prev_time_stamp; //time stamp of the previous point cloud

        /*---------sub-pub parameters----------*/
        std::string _point_cloud_topic; //point cloud ros topic to subscribe to
        std::string _imu_topic; //imu ros topic to subscribe to
        std::string _gnss_topic; //imu ros topic to subscribe to
        std::string _pose_topic; //pose ros topic to which to publish
        
        /*----------ICP parameters------------*/
        double _leaf_size; //leaf size for voxel grid
        double _minX, _maxX, _minY, _maxY, _minZ, _maxZ; //min and max pts for box filter
        double _mean_k; //number of neighbors to analyze for each point for noise removal'_pose_topic',1
        double _std_mul; //standard deviation multiplication threshold for noise removal

        double _dist_threshold; //distance threshold for RASNSAC to consider a point as inlier (for ground removal)
        double _eps_angle; //allowed difference of angles in degrees for perpendicular plane model

        double _transformation_epsilon; //minimum transformation difference for termination condition
        double _max_iters; //max number of registration iterations
        double _euclidean_fitness_epsilon; //maximum allowed Euclidean error between two consecutive steps in the ICP loop
        double _max_correspondence_distance; //correspondences with higher distances will be ignored
        double _speed; //speed for initial guess
        double _yaw_rate; //change in yaw for initial guess

        std::string _map_path; //Path for PCD map

        // Had to change max iter, meak k and eps angle from int to double
};

#endif