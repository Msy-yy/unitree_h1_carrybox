cmake_minimum_required(VERSION 3.5)
project(qr_code_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies (replacing ROS2 dependencies with ROS1 equivalents)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  message_filters
  tf
)

find_package(OpenCV REQUIRED)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport pcl_ros message_filters tf
  DEPENDS OpenCV
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable and link libraries
add_executable(qr_code_detector_node src/qr_code_detector.cpp)
target_link_libraries(qr_code_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install targets
install(TARGETS qr_code_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Testing and linting options (if applicable)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add tests here if needed
endif()
