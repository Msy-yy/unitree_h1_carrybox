# 第一阶段：编译Livox-SDK2, 编译livox_ros_driver2
FROM ros:melodic-ros-core-bionic AS build_livox_ros_driver2

#COPY sources.list /etc/apt

RUN apt-get update &&\
  apt-get install -y --no-install-recommends --fix-missing \
  build-essential ros-melodic-pcl-ros &&\
  apt-get clean

# build livox-sdk2
COPY  ./Livox-SDK2  /Livox-SDK2
RUN cd /Livox-SDK2 &&\
  mkdir build && cd build &&\
  cmake .. &&\
  make &&\
  make install  &&\
  rm -rf /Livox-SDK2 &&\
  rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/lidar_driver2/src
COPY  ./livox_ros_driver2  /root/lidar_driver2/src/livox_ros_driver2

WORKDIR /root/lidar_driver2
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

RUN  rm -rf /root/lidar_driver2/src/livox_ros_driver2/src/

# 第二阶段：最终镜像, 只包含构建好的livox_ros_driver2 ROS文件
FROM ros:melodic-ros-core-bionic
COPY --from=build_livox_ros_driver2 /root/lidar_driver2/ /root/lidar_driver2/
RUN sed -i "6i source \"/root/lidar_driver2/devel/setup.bash\"" /ros_entrypoint.sh

CMD roslaunch livox_ros_driver2 sator_lidar.launch
