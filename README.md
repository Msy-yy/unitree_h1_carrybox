# humanoid-demo

##Base on unitree H1 robot
##Base on unitree sdk2

## install ros1
   https://wiki.ros.org/noetic/Installation/Ubuntu

## install realsense2-ros
    sudo apt-get install ros-noetic-realsense2-camera

## build this package
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws/src
    git clone -b ros1 git@github.com:embodyx/humanoid-demo.git
    cd ~/ros_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make_isolated --install


## ros master
    export ROS_HOSTNAME=192.168.217.213
    export ROS_MASTER_URI=http://192.168.217.213:11311
    export ROS_IP=192.168.217.213
## servant
    export ROS_HOSTNAME=192.168.217.48 #servant ip
    export ROS_MASTER_URI=http://192.168.217.213:11311
    export ROS_IP=192.168.217.213


## run demo
    source install_isolated/setup.bash
    roslaunch humanoid_bringup bringup.launch

## run mapping
    roslaunch h1_description robot_model.launch
    roslaunch fast_lio_sam mapping_mid360.launch
## save map
    mkdir ~/maps
    rosservice call /save_map "resolution: 0.0
        destination: '/maps/pcd'" 
        success: True

## save arm position
    rosservice call /save_arm_position "pose_index: 0"
## move arm to saved position
    rosservice call /move_to_saved_position "pose_index: 0"
