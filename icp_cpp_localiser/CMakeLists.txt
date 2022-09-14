cmake_minimum_required(VERSION 3.5)
project(icp_cpp_localiser)

# Default to C++14
# HAD TO CHANGE TO C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_executable(icp_localiser src/icp_cpp_node.cpp)
# target_link_libraries(icp_localiser ${PCL_Libraries})
add_executable(icp_localiser_node src/ICP_node.cpp src/ICP.cpp) # Add executable needs a main function - just add these two files into executable

# ament_target_dependencies(icp_localiser rclcpp std_msgs PCL)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# install(TARGETS 
#   icp_localiser
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rospy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.8 REQUIRED)
#find_package(perception_pcl REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(pcl_ros REQUIRED)

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_include_directories( icp_localiser_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries( icp_localiser_node ${PCL_LIBRARIES})


# TEST - For finding PCD File
find_package(Boost
  )                      # e.g. "date_time" for "libboost_date_time"
################################

ament_target_dependencies(icp_localiser_node
  "rclcpp"
  "tf2"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "PCL"
  "tf2_ros"
  #"pcl_ros"
  "pcl_conversions"
  )

install(TARGETS
  icp_localiser_node
  DESTINATION lib/${PROJECT_NAME})  

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add_executable(icp_node src/ICP_node.cpp src/ICP.cpp)
# target_link_libraries(icp_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(icp_node ${catkin_EXPORTED_TARGETS})
