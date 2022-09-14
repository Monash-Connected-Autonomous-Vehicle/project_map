#include "icp_cpp_localiser/ICP.h"
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

#define BOOST_BIND_NO_PLACEHOLDERS // this must go before pcl imports to prevent "ambiguous placeholder" error: https://answers.ros.org/question/379224/error-reference-to-_1-is-ambiguous/
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
#include <pcl/filters/voxel_grid.h>
/*#include <pcl_ros/transforms.hpp>*/


// #include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <boost/filesystem.hpp>

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
    // Had to change while using ros2 run instead of Launch ///////////////
    this->declare_parameter("point_cloud_topic","/carla/ego_vehicle/lidar"); //velodyne_points
    this->declare_parameter("imu_topic","/carla/ego_vehicle/imu");
    this->declare_parameter("pose_topic","/my_icp_pose");
    this->declare_parameter("map_path","/home/mcav/liam_ws/localisation/pointclouds/less_points_good_map.pcd");

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
    this->get_parameter("imu_topic", _imu_topic);
    RCLCPP_INFO(this->get_logger(),"imu_topic: %s", _imu_topic.c_str()); 
    this->get_parameter("pose_topic", _pose_topic);
    RCLCPP_INFO(this->get_logger(),"pose_topic: %s", _pose_topic.c_str()); 

    this->get_parameter("map_path", _map_path);
    RCLCPP_INFO(this->get_logger(),"map_path: %s", _map_path.c_str());

    // Subscribers and Publishers
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(_point_cloud_topic, 1, std::bind(&ICP3D::cloudCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(_imu_topic, 1, std::bind(&ICP3D::imuCallback, this, _1));
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(_pose_topic,1);
    map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("the_map",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //initialising values
    _prev_acc = 0.0;
    _curr_acc = 0.0;
    _yaw_rate = 0.0;
    _speed = 0.0;

    // Pose Variables - NEED TO CHANGE TO AN ARRAY
    _curr_pose_x = 0.0;
    _curr_pose_y = 0.0;
    _curr_pose_z = 0.0;
    _curr_rot_x = 0.0;
    _curr_rot_y = 0.0;
    _curr_rot_z = 0.0;
    _curr_rot_w = 0.0;


    is_initial = true;
    is_imu_start = true;

    // Check for path to find pcd map
    boost::filesystem::path full_path(boost::filesystem::current_path());
    std::cout << "Current path is : " << full_path << std::endl;
}

/* @brief Cropping the cloud using Box filter */
void ICP3D::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(_minX, _minY, _minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(_maxX, _maxY, _maxZ, 1.0));
    boxFilter.setInputCloud(in_cloud_ptr);
    // This filter is not working ////// Please fix
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

    // Error coming from filter in cropCloud for incoming scans ////////
    // cropCloud(in_cloud_ptr, cropped_cloud_ptr);

    // Errors for map coming from
    removeGround(in_cloud_ptr, no_ground_cloud_ptr, only_ground_cloud_ptr);
    // RCLCPP_INFO(this->get_logger(),to_string(no_ground_cloud_ptr->size())); 
    // RCLCPP_INFO(this->get_logger(),"No Ground Loaded \n"); 
    removeNoise(no_ground_cloud_ptr, no_noise_cloud_ptr);
    //removeNoise(no_ground_cloud_ptr, out_cloud_ptr);
    downsampleCloud(no_noise_cloud_ptr, out_cloud_ptr);

    return;
}

/* @brief Callback function to fetch IMU data, calculates speed and change in yaw */
void ICP3D::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),"IMU Callback \n"); 
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
    if(is_initial)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile(_map_path, *incoming_cloud_ptr) == -1) //* load the file
        {
            RCLCPP_INFO(this->get_logger(),"Couldn't read file map.pcd \n"); 
        }
        else {
            //filterCloud(incoming_cloud_ptr, filtered_map_ptr);
            sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*incoming_cloud_ptr, *map_msg_ptr);
            map_msg_ptr->header.frame_id = "the_map";
            map_pub->publish(*map_msg_ptr);
            _map_cloud = *incoming_cloud_ptr;
        }

        _prev_time_stamp = msg->header.stamp.sec;

        //Initialising the previous transformation
        // This would be good for initial pose estimation (if car doesn't start at 0 position)
        prev_transformation = Eigen::Matrix4f::Identity();

        is_initial = false;
    }
    else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(_map_cloud));
        chrono::time_point<chrono::system_clock> start, end;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *current_cloud_ptr);
        filterCloud(current_cloud_ptr, filtered_cloud_ptr);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setTransformationEpsilon(_transformation_epsilon);
        icp.setMaximumIterations(_max_iters);
        icp.setMaxCorrespondenceDistance(_max_correspondence_distance);
        icp.setEuclideanFitnessEpsilon(_euclidean_fitness_epsilon);

        // Match to the Map Cloud
        icp.setInputTarget(map_cloud_ptr);

        // Set Incoming Clouud as Source
        icp.setInputSource(filtered_cloud_ptr);


        ///////////////////////// USING IMU ////////////////////////////////////
        double diff_time = msg->header.stamp.sec - _prev_time_stamp; //calculating time btw the matching pointclouds
        double diff_yaw = diff_time*_yaw_rate;
        //Eigen::AngleAxisf init_rotation (diff_yaw, Eigen::Vector3f::UnitZ ());
        Eigen::AngleAxisf init_rotation (diff_yaw, Eigen::Vector3f::UnitZ ());
        double del_x = diff_time*_speed;
        Eigen::Translation3f init_translation (del_x, 0.0, 0.0);
        Eigen::Matrix4f init_guess_imu = (init_translation * init_rotation).matrix ();
        Eigen::Matrix4f init_guess = init_guess_imu*prev_transformation;
        /////////////////////////////////////////////////////////////////////////////////        

        // Use this instead if you want to use IMu corrections
        //Eigen::Matrix4f init_guess = prev_transformation;

        // Matching CLouds
        start = chrono::system_clock::now(); 
        icp.align(*transformed_cloud_ptr, init_guess);
        end = chrono::system_clock::now();
        chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this->get_logger(),"ICP has converged");// in: " + to_string(elapsed_seconds) + " with Fitness score: " + to_string(icp.getFitnessScore()));

        Eigen::Matrix4f t = icp.getFinalTransformation();

        // Broadcast Transformation Matrix
        tf_broadcast(t);
 
        //tf_listener();
        geometry_msgs::msg::PoseWithCovarianceStamped curr_pose;
        curr_pose.header.stamp = rclcpp::Clock().now();
        curr_pose.header.frame_id = "the_map";
        curr_pose.pose.pose.position.x = _curr_pose_x;
        curr_pose.pose.pose.position.y = _curr_pose_y;        
        curr_pose.pose.pose.position.z = _curr_pose_z;        
        curr_pose.pose.pose.orientation.x = _curr_rot_x;
        curr_pose.pose.pose.orientation.y = _curr_rot_y;        
        curr_pose.pose.pose.orientation.z = _curr_rot_z;        
        curr_pose.pose.pose.orientation.w = _curr_rot_w;


        //----------Note: for now the covariance is left at default values-----------
        //---------later covariance values will also be used,------------------------
        //---------so this can used as input to probabilistic filter like EKF/UKF----
        pose_pub->publish(curr_pose); //publishing the current pose
        RCLCPP_INFO(this->get_logger(),"Publishing Pose");
        auto translation_string = "Trans XYZ:" + to_string(_curr_pose_x) + ", "  + to_string(_curr_pose_y) + ", "  + to_string(_curr_pose_z);
        RCLCPP_INFO(this->get_logger(), translation_string.c_str());
        auto rotation_string = "ROT XYZW:" + to_string(_curr_rot_x)  + ", "  + to_string(_curr_rot_y) + ", "  + to_string(_curr_rot_z) + ", "  + to_string(_curr_rot_w);
        RCLCPP_INFO(this->get_logger(), rotation_string.c_str()); 

        prev_transformation = t;
        _prev_time_stamp = msg->header.stamp.sec;
    }
    return;
}

void ICP3D::tf_broadcast(Eigen::Matrix4f trans){
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    Eigen::Quaternionf q(trans.topLeftCorner<3, 3>());
    Eigen::Vector3f v = trans.topRightCorner<3, 1>();

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = now;
    t.header.frame_id = "the_map";
    t.child_frame_id = "ego_vehicle/lidar";
    t.transform.translation.x = v(0);
    t.transform.translation.y = v(1);
    t.transform.translation.z = v(2);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    _curr_pose_x = t.transform.translation.x;
    _curr_pose_y = t.transform.translation.y;        
    _curr_pose_z = t.transform.translation.z;        
    _curr_rot_x = t.transform.rotation.x;
    _curr_rot_y = t.transform.rotation.y;        
    _curr_rot_z = t.transform.rotation.z;       
    _curr_rot_w = t.transform.rotation.w;
    

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(),"Broadcasted TF\n"); 
    return;
}


// CURRENTLY NOT IN USE
void ICP3D::tf_listener(){
    //RCLCPP_INFO(this->get_logger(),"Getting Pose\n"); 
    geometry_msgs::msg::TransformStamped transformStamped;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
    try {
        transformStamped = tf_buffer_->lookupTransform(
        "map", "ego_vehicle/lidar",
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s","map", "ego_vehicle/lidar", ex.what());
        return;
    }
    //RCLCPP_INFO(this->get_logger(),to_string(transformStamped.transform.translation.x)); 
    _curr_pose_x = transformStamped.transform.translation.x;
    _curr_pose_y = transformStamped.transform.translation.y;        
    _curr_pose_z = transformStamped.transform.translation.z;        
    _curr_rot_x = transformStamped.transform.rotation.x;
    _curr_rot_y = transformStamped.transform.rotation.y;        
    _curr_rot_z = transformStamped.transform.rotation.z;       
    _curr_rot_w = transformStamped.transform.rotation.w;
    //RCLCPP_INFO(this->get_logger(),to_string(_curr_pose_x)); 
    
    return;
}