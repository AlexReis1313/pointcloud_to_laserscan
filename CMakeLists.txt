cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

find_package(ament_cmake_auto REQUIRED)
find_package(pcl_ros REQUIRED)
#find_package(small_gicp REQUIRED)
#include_directories(${CMAKE_SOURCE_DIR}/src/small_gicp/include)
#target_link_libraries(pointcloud_to_laserscan small_gicp)
#add_dependencies(pointcloud_to_laserscan small_gicp)

#add_library(  src/vgicp_registration.cpp ) # Add this if missing

ament_auto_find_build_dependencies()

ament_auto_add_library(laserscan_to_pointcloud SHARED
  src/laserscan_to_pointcloud_node.cpp)



ament_auto_add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
