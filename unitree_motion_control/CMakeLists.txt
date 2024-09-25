cmake_minimum_required(VERSION 3.5)
project(unitree_motion_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# Find the catkin package and required dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_srvs
  humanoid_msgs
  tf
)

add_subdirectory(thirdparty)

message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

set(UNITREE_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
find_library(UNITREE_SDK_LIB unitree_sdk2 PATHS ${UNITREE_SDK_PATH} NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else ()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif ()

message(STATUS "Importing: ${UNITREE_SDK_LIB}")

# Declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unitree_sdk2
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_srvs humanoid_msgs tf
  DEPENDS system_lib
)

# Add the unitree_sdk2 library
add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unitree_sdk2 PROPERTIES
        IMPORTED_LOCATION ${UNITREE_SDK_LIB})
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
target_include_directories(unitree_sdk2 INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Define the executables and link them to the necessary libraries
add_executable(leg_move src/leg_move.cpp)
target_link_libraries(leg_move unitree_sdk2 ${catkin_LIBRARIES})

add_executable(arm_control src/arm_control.cpp)
target_link_libraries(arm_control unitree_sdk2 ${catkin_LIBRARIES})

add_executable(joint_states_pub src/joint_states_pub.cpp)
target_link_libraries(joint_states_pub unitree_sdk2 ${catkin_LIBRARIES})

add_executable(motion_controller src/motion_controller.cpp)
target_link_libraries(motion_controller unitree_sdk2 ${catkin_LIBRARIES})

add_executable(odom_tf_pub src/odom_tf_pub.cpp)
target_link_libraries(odom_tf_pub ${catkin_LIBRARIES})

# Add include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Install the executables
install(TARGETS  arm_control joint_states_pub leg_move motion_controller odom_tf_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add testing (if needed)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # add_rostest(test/motion_controller.test)
endif()

