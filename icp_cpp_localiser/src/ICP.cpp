#include "../include/icp_cpp_localiser/ICP.h"
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic pop
#include "std_msgs/msg/string.hpp"


#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>


using std::placeholders::_1;

using namespace std;

/* @brief Constructor */
ICP3D::ICP3D()
: Node("icp_localiser_node")
{
    /* Declare parameters */
    this->declare_parameter("leaf_size",0.1);
    this->declare_parameter("dist_threshold",0.1);
    this->declare_parameter("eps_angle",15.0); ////
    this->declare_parameter("minX",0.0);
    this->declare_parameter("minY",-25.0);
    this->declare_parameter("minZ",-3.0);
    this->declare_parameter("maxX",-40.0);
    this->declare_parameter("maxY",25.0);
    this->declare_parameter("maxZ",3.0);
    this->declare_parameter("mean_k",50.0); //// 
    this->declare_parameter("std_mul",1.0);
    this->declare_parameter("transformation_epsilon",0.01);
    this->declare_parameter("max_iters",75.0); ///
    this->declare_parameter("euclidean_fitness_epsilon",0.1);
    this->declare_parameter("max_correspondence_distance",1.0);
    this->declare_parameter("point_cloud_topic","velodyne_points");
    this->declare_parameter("imu_topic","imu/data");
    this->declare_parameter("pose_topic","icp_pose");

    /* Loading parameters */
    this->get_parameter("leaf_size", _leaf_size);
    RCLCPP_INFO(this->get_logger(),"leaf_size: %f", _leaf_size);
    this->get_parameter("dist_threshold", _dist_threshold);
    RCLCPP_INFO(this->get_logger(),"dist_threshold: %f", _dist_threshold);
    this->get_parameter("eps_angle", _eps_angle);
    RCLCPP_INFO(this->get_logger(),"eps_angle: %f", _eps_angle);
    this->get_parameter("minX", _minX);
    RCLCPP_INFO(this->get_logger(),"minX: %f", _minX);
    this->get_parameter("minY", _minY);
    RCLCPP_INFO(this->get_logger(),"minY: %f", _minY);
    this->get_parameter("minZ", _minZ);
    RCLCPP_INFO(this->get_logger(),"minZ: %f", _minZ);
    this->get_parameter("maxX", _maxX);
    RCLCPP_INFO(this->get_logger(),"maxX: %f", _maxX);
    this->get_parameter("maxY", _maxY);
    RCLCPP_INFO(this->get_logger(),"maxY: %f", _maxY);
    this->get_parameter("maxZ", _maxZ);
    RCLCPP_INFO(this->get_logger(),"maxZ: %f", _maxZ);
    this->get_parameter("mean_k", _mean_k);
    RCLCPP_INFO(this->get_logger(),"mean_k: %f", _mean_k);
    this->get_parameter("std_mul", _std_mul);
    RCLCPP_INFO(this->get_logger(),"std_mul: %f", _std_mul);

    this->get_parameter("transformation_epsilon", _transformation_epsilon);
    RCLCPP_INFO(this->get_logger(),"transformation_epsilon: %f", _transformation_epsilon);    
    this->get_parameter("max_iters", _max_iters);
    RCLCPP_INFO(this->get_logger(),"max_iters: %f", _max_iters);   
    this->get_parameter("euclidean_fitness_epsilon", _euclidean_fitness_epsilon);
    RCLCPP_INFO(this->get_logger(),"euclidean_fitness_epsilon: %f", _euclidean_fitness_epsilon);  
    this->get_parameter("max_correspondence_distance", _max_correspondence_distance);
    RCLCPP_INFO(this->get_logger(),"max_correspondence_distance: %f", _max_correspondence_distance);  
 
    this->get_parameter("point_cloud_topic", _point_cloud_topic);
    RCLCPP_INFO(this->get_logger(),"point_cloud_topic: %s", _point_cloud_topic.c_str());  
    // private_nh.param<std::string>("point_cloud_topic", _point_cloud_topic, "velodyne_points");
    // RCLCPP_INFO(node->get.logger(),"point_cloud_topic: %s", _point_cloud_topic.c_str());
    this->get_parameter("imu_topic", _imu_topic);
    RCLCPP_INFO(this->get_logger(),"imu_topic: %s", _imu_topic.c_str()); 
    // private_nh.param<std::string>("imu_topic", _imu_topic, "imu/data");
    // RCLCPP_INFO(node->get.logger(),"imu_topic: %s", _imu_topic.c_str());
    this->get_parameter("pose_topic", _pose_topic);
    RCLCPP_INFO(this->get_logger(),"pose_topic: %s", _pose_topic.c_str()); 
    // private_nh.param<std::string>("pose_topic", _pose_topic, "icp_pose");
    // RCLCPP_INFO(node->get.logger(),"pose_topic: %s", _pose_topic.c_str());

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(_point_cloud_topic, 1, std::bind(&ICP3D::cloudCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(_imu_topic, 1, std::bind(&ICP3D::imuCallback, this, _1));
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(_pose_topic,1);

    //initialising values
    _prev_acc = 0.0;
    _curr_acc = 0.0;
    _yaw_rate = 0.0;
    _speed = 0.0;

    is_initial = true;
    is_imu_start = true;

}

/* @brief Cropping the cloud using Box filter */
void ICP3D::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(_minX, _minY, _minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(_maxX, _maxY, _maxZ, 1.0));
    boxFilter.setInputCloud(in_cloud_ptr);
    boxFilter.filter(*out_cloud_ptr);

    //cout<<"Crop Input: "<<in_cloud_ptr->size()<<" pts, Crop output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

/* @brief Removing Noise using Statistical outlier method */
void ICP3D::removeNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setMeanK(_mean_k);
    sor.setStddevMulThresh(_std_mul);
    sor.filter(*out_cloud_ptr);

    //cout<<"Noise Input: "<<in_cloud_ptr->size()<<" pts, Noise output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

/* @brief Downsampling using Aprroximate Voxel grid filter */
void ICP3D::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    //cout<<"-------Downsampling cloud---------"<<endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    approx_vg.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    approx_vg.setInputCloud(in_cloud_ptr);
    approx_vg.filter(*out_cloud_ptr);

    //cout<<"DS Input: "<<in_cloud_ptr->size()<<" pts, DS output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

/* @brief Removes ground plane using perpendicular plane model with RANSAC */
void ICP3D::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr)
{
    //cout<<"-------Removing ground---------"<<endl;
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //Creating the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true); //optional
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_dist_threshold);
    seg.setAxis(Eigen::Vector3f(0,0,1)); //z-axis
    seg.setEpsAngle(_eps_angle);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout<<"Could not estimate the plane"<<endl;
    }

    //Remove ground from the cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true); //true removes the indices
    extract.filter(*out_cloud_ptr);

    //Extract ground from the cloud
    extract.setNegative(false); //false leaves only the indices
    extract.filter(*ground_plane_ptr);

    //cout<<"GR Input: "<<in_cloud_ptr->size()<<" pts, GR output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}

/* @brief Filters the point cloud using cropping, ground and noise removal filters and then downsamples */
void ICP3D::filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr only_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_noise_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cropCloud(in_cloud_ptr, cropped_cloud_ptr);
    removeGround(cropped_cloud_ptr, no_ground_cloud_ptr, only_ground_cloud_ptr);
    removeNoise(no_ground_cloud_ptr, no_noise_cloud_ptr);
    //removeNoise(no_ground_cloud_ptr, out_cloud_ptr);
    downsampleCloud(no_noise_cloud_ptr, out_cloud_ptr);

    return;
}

/* @brief Callback function to fetch IMU data, calculates speed and change in yaw */
void ICP3D::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if(is_imu_start)
    {
        _prev_acc = msg->linear_acceleration.x;
        _prev_imu_time = msg->header.stamp.sec;

        is_imu_start = false;
    }
    else
    {
        _curr_acc = msg->linear_acceleration.x;
        _curr_imu_time = msg->header.stamp.sec;

        double del_time = _curr_imu_time - _prev_imu_time;
        double avg_acc = 0.5*(_prev_acc + _curr_acc);

        _speed = avg_acc*del_time;
        _yaw_rate = msg->angular_velocity.z;

        _prev_acc = _curr_acc;
        _prev_imu_time = _curr_imu_time;
    }

    return;
}

/* @brief Callback function for pointcloud, implements the ICP algo */
void ICP3D::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    //cout<<"-------Entered callback---------"<<endl;
    
    if(is_initial)
    {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        //pcl::fromPCLPointCloud2(*msg, *prev_cloud_ptr);
        //pcl_conversions::fromROSMsg(*msg, *prev_cloud_ptr);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *prev_cloud_ptr);
        
        filterCloud(prev_cloud_ptr, filtered_cloud_ptr);
        
        _prev_cloud = *filtered_cloud_ptr;
        _prev_time_stamp = msg->header.stamp.sec;

        //initialising the previous transformation
        prev_transformation = Eigen::Matrix4f::Identity();

        is_initial = false;
    }
    else
    {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(_prev_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        chrono::time_point<chrono::system_clock> start, end;

        start = chrono::system_clock::now(); 
        //pcl::fromPCLPointCloud2(*msg, *current_cloud_ptr);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *current_cloud_ptr);
        
        filterCloud(current_cloud_ptr, filtered_cloud_ptr);
        
        //cout<<"Filtered cloud has "<<filtered_cloud_ptr->size()<<"points"<<endl;
        //cout<<"Current cloud has "<<current_cloud_ptr->size()<<"points"<<endl;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setTransformationEpsilon(_transformation_epsilon);
        icp.setMaximumIterations(_max_iters);
        icp.setMaxCorrespondenceDistance(_max_correspondence_distance);
        icp.setEuclideanFitnessEpsilon(_euclidean_fitness_epsilon);
        //icp.setInputSource(_prev_cloud_ptr);
        //icp.setInputTarget(_downsampled_cloud_ptr);
        icp.setInputSource(filtered_cloud_ptr);
        icp.setInputTarget(prev_cloud_ptr);

        double diff_time = msg->header.stamp.sec - _prev_time_stamp; //calculating time btw the matching pointclouds

        double diff_yaw = diff_time*_yaw_rate;
        Eigen::AngleAxisf init_rotation (diff_yaw, Eigen::Vector3f::UnitZ ());
        double del_x = diff_time*_speed;
        Eigen::Translation3f init_translation (del_x, 0.0, 0.0);
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

        //cout<<"-------Matching clouds---------"<<endl;
        //start = chrono::system_clock::now(); 
        icp.align(*transformed_cloud_ptr, init_guess);
        //icp.align(*transformed_cloud_ptr);
        end = chrono::system_clock::now();

        //cout<<"-------Matching done---------"<<endl;

        cout << "ICP has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << endl;
        RCLCPP_INFO(this->get_logger(),"ICP has converged");

        chrono::duration<double> elapsed_seconds = end - start;
        //cout<<"elapsed time: " << elapsed_seconds.count() << "s\n"; 

        Eigen::Matrix4f t = icp.getFinalTransformation();
        Eigen::Matrix4f curr_transformation = prev_transformation*t; //final transformation matrix

        Eigen::Matrix3f mat; //rotation matrix
        Eigen::Vector3f trans; //translation vector
        
        trans << curr_transformation(0,3), curr_transformation(1,3), curr_transformation(2,3);
        mat << curr_transformation(0,0), curr_transformation(0,1), curr_transformation(0,2),
               curr_transformation(1,0), curr_transformation(1,1), curr_transformation(1,2),
               curr_transformation(2,0), curr_transformation(2,1), curr_transformation(2,2);

        Eigen::Quaternionf quat(mat); //rotation matrix stored as a quaternion

        geometry_msgs::msg::PoseWithCovarianceStamped curr_pose;
        curr_pose.header.stamp = rclcpp::Clock().now();
        curr_pose.header.frame_id = "icp";
        curr_pose.pose.pose.position.x = trans[0];
        curr_pose.pose.pose.position.y = trans[1];        
        curr_pose.pose.pose.position.z = trans[2];        
        curr_pose.pose.pose.orientation.x = quat.x();
        curr_pose.pose.pose.orientation.y = quat.y();        
        curr_pose.pose.pose.orientation.z = quat.z();        
        curr_pose.pose.pose.orientation.w = quat.w();

        //----------Note: for now the covariance is left at default values-----------
        //---------later covariance values will also be used,------------------------
        //---------so this can used as input to probabilistic filter like EKF/UKF----

        pose_pub->publish(curr_pose); //publishing the current pose

        _prev_cloud = *filtered_cloud_ptr;
        prev_transformation = curr_transformation;
        _prev_time_stamp = msg->header.stamp.sec;
    }

    return;
}