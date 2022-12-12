#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class PcdMapPublisher : public rclcpp::Node
{
  public:
    PcdMapPublisher()
    : Node("pcd_map_publisher")
    {
        // Loading and publish map
        rclcpp::QoS durable_qos{1};
        durable_qos.transient_local();
        auto map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", durable_qos);

        const auto map_path = declare_parameter<std::string>("pcd_path", "map.pcd");
        sensor_msgs::msg::PointCloud2 incoming_map;
        
        int load_result = pcl::io::loadPCDFile(map_path, incoming_map);
        if (load_result == -1)
        {
            RCLCPP_INFO(this->get_logger(), "Couldn't read file %s\n", map_path.c_str()); 
        } 
        else 
        {
            incoming_map.header.frame_id = "map";
            map_publisher_->publish(incoming_map);
            RCLCPP_INFO(this->get_logger(),"Loaded and published map.\n"); 
        }
    }

    /* @brief Downsampling using Approximate Voxel grid filter */
    // void ICP3D::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
    // {
    //     //cout<<"-------Downsampling cloud---------"<<endl;
    //     pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    //     approx_vg.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    //     approx_vg.setInputCloud(in_cloud_ptr);
    //     approx_vg.filter(*out_cloud_ptr);

    //     return;
    // }

    // /* @brief Removes ground plane using perpendicular plane model with RANSAC */
    // void ICP3D::removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
    //                             pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_ptr)
    // {
    //     //cout<<"-------Removing ground---------"<<endl;
    //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //     //Creating the segmentation object
    //     pcl::SACSegmentation<pcl::PointXYZ> seg;
    //     seg.setOptimizeCoefficients(true); //optional
    //     seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    //     seg.setMethodType(pcl::SAC_RANSAC);
    //     seg.setDistanceThreshold(_dist_threshold);
    //     seg.setAxis(Eigen::Vector3f(0,0,1)); //z-axis
    //     seg.setEpsAngle(_eps_angle);
    //     seg.setInputCloud(in_cloud_ptr);
    //     seg.segment(*inliers, *coefficients);
    //     if(inliers->indices.size() == 0)
    //     {
    //         RCLCPP_INFO(this->get_logger(),"Could not estimate the plane");
    //     }

    //     //Remove ground from the cloud
    //     pcl::ExtractIndices<pcl::PointXYZ> extract;
    //     extract.setInputCloud(in_cloud_ptr);
    //     extract.setIndices(inliers);
    //     extract.setNegative(true); //true removes the indices
    //     extract.filter(*out_cloud_ptr);

    //     return;
    // }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdMapPublisher>());
  rclcpp::shutdown();
  return 0;
}

