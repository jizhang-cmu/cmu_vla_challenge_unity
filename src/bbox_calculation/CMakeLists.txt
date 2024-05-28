cmake_minimum_required(VERSION 3.5)
project(bbox_calculation)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(bboxCalculation src/bboxCalculation.cpp)
add_executable(bboxBroadcast src/bboxBroadcast.cpp)

## Specify libraries to link a library or executable target against
ament_target_dependencies(bboxCalculation rclcpp std_msgs nav_msgs pcl_ros message_filters)
ament_target_dependencies(bboxBroadcast rclcpp std_msgs nav_msgs pcl_ros tf2 tf2_ros tf2_geometry_msgs message_filters visualization_msgs)
target_link_libraries(bboxCalculation ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(bboxBroadcast ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS bboxCalculation bboxBroadcast
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()