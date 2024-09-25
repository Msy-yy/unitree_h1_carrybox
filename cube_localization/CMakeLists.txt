CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(cube_localization)

ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()
FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(OpenMP)
FIND_PACKAGE(
        catkin REQUIRED
        roscpp
        std_msgs
        nav_msgs
        tf
        tf2_ros
        pcl_ros
        sensor_msgs
        pcl_conversions
        tf_conversions
)

IF (OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF ()

CATKIN_PACKAGE()

INCLUDE_DIRECTORIES(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

ADD_LIBRARY(
        lo_ndt_omp
        src/pclomp/voxel_grid_covariance_omp.cpp
        src/pclomp/ndt_omp.cpp
        src/pclomp/gicp_omp.cpp
)

ADD_EXECUTABLE(cube_localization src/localization.cpp)
ADD_DEPENDENCIES(cube_localization lo_ndt_omp)
TARGET_LINK_LIBRARIES(cube_localization ${PCL_LIBRARIES} ${catkin_LIBRARIES} lo_ndt_omp)


install(TARGETS
    cube_localization
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    lo_ndt_omp
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config  launch rviz urdf maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
