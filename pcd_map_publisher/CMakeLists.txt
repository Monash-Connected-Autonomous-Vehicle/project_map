cmake_minimum_required(VERSION 3.8)
project(pcd_map_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

add_executable(pcd_map_publisher src/pcd_map_publisher.cpp)
ament_target_dependencies(pcd_map_publisher rclcpp sensor_msgs)

target_link_libraries(pcd_map_publisher ${PCL_LIBRARIES})

target_include_directories(pcd_map_publisher
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
)

install(TARGETS
  pcd_map_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
