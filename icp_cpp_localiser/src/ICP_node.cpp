#include "icp_cpp_localiser/ICP.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ICP3D>());
    rclcpp::shutdown();
    return 0;
}