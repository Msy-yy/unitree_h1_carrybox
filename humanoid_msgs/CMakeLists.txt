cmake_minimum_required(VERSION 3.0.2)
project(humanoid_msgs)

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

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# Add service files
add_service_files(
  FILES
  MovePose.srv
  SavePose.srv
)

# Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# Testing section (if needed)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

# Installation instructions (if needed)
install(DIRECTORY srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
