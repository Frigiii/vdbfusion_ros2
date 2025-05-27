cmake_minimum_required(VERSION 3.8)
project(vdbfusion_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(openvdb_vendor REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(OpenVDB REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# builds the vdbfusion library from source
add_library(
  vdbfusion_library
  src/vdbfusion/VDBVolume.cpp
  src/vdbfusion/MarchingCubes.cpp
)
target_include_directories(
  vdbfusion_library
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenVDB_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(
  vdbfusion_library
  ${OpenVDB_LIBRARIES}
)

install (
  TARGETS vdbfusion_library
  EXPORT export_vdbfusion_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# build the vdbfusion node as a shared library 
add_library(
  vdbfusion_node
  SHARED
  src/vdbfusion_ros2/vdbfusion_node.cpp
)

target_include_directories(
  vdbfusion_node
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(
  vdbfusion_node
  vdbfusion_library
)

ament_target_dependencies(
  vdbfusion_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs)

# install the vdbfusion node as a shared library
rclcpp_components_register_nodes(vdbfusion_node "vdbfusion::vdbfusion_node")

install(
  TARGETS vdbfusion_node
  EXPORT export_vdbfusion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
    FILES plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
