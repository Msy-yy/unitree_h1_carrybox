//
// Created by dongfang on 2023/8/25.
//
#ifndef CONFIG_
#define CONFIG_
#include <ros/ros.h>
using namespace std;
class Config {
 public:
  string world_frame = "world";
  string map_frame = "map";
  string odom_frame = "odom";
  string base_frame = "base_link";
  string front_lidar_frame = "front_livox";
  string rear_lidar_frame = "rear_livox";
  string scan_topic_front;
  string scan_topic_rear;
  string map_load_file;
  string traj_load_file;
  std::vector<double> local_map_size;
  int sub_map_scan_buf_size;
  string scan_matcher_type = "ndt";
  bool debug = false;
  float init_pose_x, init_pose_y, init_pose_z, init_pose_roll, init_pose_pitch, init_pose_yaw;

  struct {
    int num_threads = 4;
    int maximum_iterations = 20;
    float voxel_leaf_size = 0.1;
    float resolution = 1.0;
    double transformation_epsilon = 0.01;
    double step_size = 0.1;
    double thresh_shift = 2;
    double thresh_rot = M_PI / 12;

  } ndt;
  struct {
    double max_correspondence_distance = 0.1;
    double transformation_epsilon = 1e-6;
    double euclidean_fitness_epsilon = 1e-6;
    double maximum_iterations = 20;
  } icp;

  explicit Config(ros::NodeHandle &nh) : nh_(nh) {
    nh_.getParam("/world_frame", world_frame);
    nh_.getParam("/map_frame", map_frame);
    nh_.getParam("/odom_frame", odom_frame);
    nh_.getParam("/base_frame", base_frame);
    nh_.getParam("/front_lidar_frame", front_lidar_frame);
    nh_.getParam("/rear_lidar_frame", rear_lidar_frame);
    nh_.getParam("/scan_topic_front", scan_topic_front);
    nh_.getParam("/scan_topic_rear", scan_topic_rear);
    nh_.getParam("/traj_load_file", traj_load_file);
    nh_.getParam("/map_load_file", map_load_file);

    nh_.getParam("/debug", debug);
    nh_.getParam("/ndt/num_threads", ndt.num_threads);
    nh_.getParam("/ndt/maximum_iterations", ndt.maximum_iterations);
    nh_.getParam("/ndt/voxel_leaf_size", ndt.voxel_leaf_size);
    nh_.getParam("/ndt/transformation_epsilon", ndt.transformation_epsilon);
    nh_.getParam("/ndt/step_size", ndt.step_size);
    nh_.getParam("/ndt/resolution", ndt.resolution);
    nh_.getParam("/ndt/thresh_shift", ndt.thresh_shift);
    nh_.getParam("/ndt/thresh_rot", ndt.thresh_rot);
    nh_.getParam("/sub_map_scan_buf_size", sub_map_scan_buf_size);
    nh_.getParam("/scan_matcher_type", scan_matcher_type);
    nh_.param<std::vector<double>>("/local_map_size", local_map_size,
                                   std::vector<double>());

    nh_.getParam("/icp/max_correspondence_distance",
                 icp.max_correspondence_distance);
    nh_.getParam("/icp/transformation_epsilon", icp.transformation_epsilon);
    nh_.getParam("/icp/euclidean_fitness_epsilon",
                 icp.euclidean_fitness_epsilon);
    nh_.getParam("/icp/maximum_iterations", icp.maximum_iterations);

    
    nh_.param<float>("/init_pose_x", init_pose_x, 1.0);
    nh_.param<float>("/init_pose_y", init_pose_y, 0.0);
    nh_.param<float>("/init_pose_z", init_pose_z, 0.0);
    nh_.param<float>("/init_pose_roll", init_pose_roll, 1.0);
    nh_.param<float>("/init_pose_pitch", init_pose_pitch, 1.0);
    nh_.param<float>("/init_pose_yaw", init_pose_yaw, 1.0);
  }

 private:
  ros::NodeHandle &nh_;
};

#endif