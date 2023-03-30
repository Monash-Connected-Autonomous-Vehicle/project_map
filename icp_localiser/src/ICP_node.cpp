// #include "rclcpp/rclcpp.hpp"
// /*#include <pcl/common/common_headers.h>
// #include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/conversions.h>
// //#include <pcl/visualization/cloud_viewer.h>
// //#include <pcl/common/transforms.h>
// #include <pcl/registration/icp.h>
// //#include "icp_localiser/cpp_header.hpp"*/
// #include "std_msgs/msg/string.hpp"

#include "../include/icp_localiser/ICP.h" // SURELY we can source better than this
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    //RCLCPP_INFO(node->get_logger(), "Herer")
    rclcpp::spin(std::make_shared<ICP3D>());
    rclcpp::shutdown();
    return 0;
}