cmake_minimum_required(VERSION 3.0.2)
project(unitree_cam_publisher)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

## Declare the ROS package dependencies
catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs std_msgs
)

find_package(OpenCV 4 REQUIRED)

if(OpenCV_FOUND)
  message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
else()
  message(FATAL_ERROR "OpenCV Library Not Found")
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
	link_directories(${CMAKE_CURRENT_SOURCE_DIR}/UnitreecameraSDK/lib/amd64/)
else()
	link_directories(${CMAKE_CURRENT_SOURCE_DIR}/UnitreecameraSDK/lib/arm64/)
endif()

set(SDKLIBS unitree_camera tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/UnitreecameraSDK/include
)

add_executable(camera_node src/camera_node.cpp)
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_node
  ${SDKLIBS}
  ${catkin_LIBRARIES}
)
