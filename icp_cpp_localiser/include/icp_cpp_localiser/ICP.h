#ifndef ICP3D_ICP_H
#define ICP3D_ICP_H
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <string>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class ICP3D : public rclcpp::Node
{
    public:
         ICP3D();

    private:
        void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg); //point cloud callback
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg); //imu data callback

        void tf_broadcast(Eigen::Matrix4f trans); // Transform Function
        void tf_listener();
        
        void cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //crops cloud using box filter
        void removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //removes noise using Statistical outlier removal
        void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //downsampling the point cloud using Voxelgrid
        void removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr); //ground removal using RANSAC
        void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //filtering the point cloud

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  
        pcl::PointCloud<pcl::PointXYZ> _map_cloud; //point cloud of the pcd map
        pcl::PointCloud<pcl::PointXYZ> _prev_cloud; //point cloud of the pcd map
        Eigen::Matrix4f _prev_transformation; //cumulative transformation until the previous time instance
        bool _is_initial, _is_imu_start; //boolean to tell if this is 1st iteration of the algo and start of imu reading
        double _prev_acc, _curr_acc; //accleration in consecutive time stamps
        double _prev_imu_time, _curr_imu_time; //consecutive time stamps of imu data 
        double _prev_time_stamp; //time stamp of the previous point cloud

        /*---------sub-pub parameters----------*/
        std::string _point_cloud_topic; //point cloud ros topic to subscribe to
        std::string _imu_topic; //imu ros topic to subscribe to
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
        //geometry_msgs::msg::PoseWithCovarianceStamped _curr_pose;

        double _curr_pose_x;
        double _curr_pose_y;
        double _curr_pose_z;
        double _curr_rot_x;
        double _curr_rot_y;
        double _curr_rot_z;
        double _curr_rot_w;

        // Had to change max iter, mean k and eps angle from int to double
};

#endif