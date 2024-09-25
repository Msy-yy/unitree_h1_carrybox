//
// Created by dongfang on 2023/8/25.
//

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Eigen>
#include <chrono>
#include <deque>

#include "config/config.h"
#include "pclomp/gicp_omp.h"
#include "pclomp/ndt_omp.h"

using namespace std;

typedef pcl::PointXYZI Point;
typedef pclomp::NormalDistributionsTransform<Point, Point> NDT;
// typedef pclomp::GeneralizedIterativeClosestPoint < Point,
// Point> ICP;
typedef pcl::GeneralizedIterativeClosestPoint<Point, Point> ICP;
typedef pcl::PointCloud<Point> Cloud;

class Localization {
 public:
  explicit Localization(ros::NodeHandle &nh)
      : nh_(nh),
        cfg_(nh),
        global_map_ptr_(new Cloud),
        global_map_filtered_ptr_(new Cloud),
        trajectory_ptr_(new Cloud),
        sub_map_ptr_(new Cloud),
        local_map_ptr_(new Cloud) {
    map_sub_ =
        nh_.subscribe("/map_cloud", 10, &Localization::MapCallback, this);
    init_pose_sub_ = nh_.subscribe(
        "/initialpose", 10, &Localization::InitPoseWithNDTCallback, this);
    map_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>("global_map_cloud", 1);
    sub_map_publisher_ =
        nh.advertise<sensor_msgs::PointCloud2>("sub_map_cloud", 1);
    localization_pose_publisher_ =
        nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
    localization_odom_publisher_ =
        nh.advertise<nav_msgs::Odometry>("/lidar_odom_r", 10);
    localization_path_publisher_ =
        nh.advertise<nav_msgs::Path>("/ndt_trajectory", 10);
    localization_path_.header.frame_id = "/map";

    laser_cloud_front_sub_ = nh_.subscribe(
        cfg_.scan_topic_front, 10, &Localization::FrontLaserCallback, this);
    laser_cloud_rear_sub_ = nh_.subscribe(
        cfg_.scan_topic_rear, 10, &Localization::RearLaserCallback, this);
    voxel_grid_filter_.setLeafSize(cfg_.ndt.voxel_leaf_size,
                                   cfg_.ndt.voxel_leaf_size,
                                   cfg_.ndt.voxel_leaf_size);
    if (cfg_.scan_matcher_type == "ndt") {
      InitNDT();
    } else {
      InitICP();
    }
    if_init_pose = true;

    map_to_odom_.setIdentity();
    odom_to_base_.setIdentity();
    InitPoseByParam();

    // GetTransformWithTF(cfg_.odom_frame, cfg_.base_frame, ros::Time(0),
    //                    odom_to_base_);
  }
  void RunLocalization();

 private:
  ros::NodeHandle &nh_;
  ros::Publisher map_publisher_;
  ros::Publisher sub_map_publisher_;
  ros::Publisher localization_pose_publisher_;
  ros::Publisher localization_odom_publisher_;
  ros::Publisher localization_path_publisher_;
  nav_msgs::Path localization_path_;
  double path_threshold_ = 0.005;

  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber laser_cloud_front_sub_;
  ros::Subscriber laser_cloud_rear_sub_;
  tf2_ros::TransformBroadcaster br_;

  // cloud time
  ros::Time cloud_time_front_;

  NDT ndt_;
  ICP icp_;

  pcl::VoxelGrid<Point> voxel_grid_filter_;
  Config cfg_;
  Cloud::Ptr global_map_ptr_, global_map_filtered_ptr_, trajectory_ptr_;
  tf::Pose odom_to_base_, map_to_odom_;
  sensor_msgs::PointCloud2::ConstPtr laser_point_cloud_ptr_ = nullptr;
  bool load_map_success_;
  bool load_traj_success_;
  bool if_init_pose;
  tf2_ros::Buffer tf_buffer_;
  std::deque<sensor_msgs::PointCloud2> front_lidar_data_buf_;
  std::deque<sensor_msgs::PointCloud2> rear_lidar_data_buf_;
  std::deque<Cloud> sub_map_buf_;
  Cloud::Ptr sub_map_ptr_, local_map_ptr_;
  tf::Pose pose_inc_;

  void InitNDT() {
    ndt_.setNumThreads(cfg_.ndt.num_threads);
    ndt_.setTransformationEpsilon(cfg_.ndt.transformation_epsilon);
    ndt_.setStepSize(cfg_.ndt.step_size);
    ndt_.setResolution(cfg_.ndt.resolution);
    ndt_.setMaximumIterations(cfg_.ndt.maximum_iterations);
    ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT1);
    // ndt_.setMaxCorrespondenceDistance(2);
    // ndt_.setRANSACIterations(10);
    // ndt_.setRANSACOutlierRejectionThreshold(cfg_.ndt.resolution*3);
  }
  void InitICP() {
    icp_.setMaxCorrespondenceDistance(cfg_.icp.max_correspondence_distance);
    icp_.setTransformationEpsilon(cfg_.icp.transformation_epsilon);
    icp_.setEuclideanFitnessEpsilon(cfg_.icp.euclidean_fitness_epsilon);
    icp_.setMaximumIterations(cfg_.icp.maximum_iterations);
  }

  void transPointCloud2ToPCL(sensor_msgs::PointCloud2 &msg, Cloud &out) {
    Cloud::Ptr tmpCloudPtr(new Cloud);
    pcl::fromROSMsg(msg, *tmpCloudPtr);
    voxel_grid_filter_.setInputCloud(tmpCloudPtr);
    voxel_grid_filter_.filter(out);
  }
  double GetInitPoseHigh(double in_x, double in_y) {
    double min_dis = 10000000000;
    double high = 0;
    if (load_traj_success_ == true) {
      for (size_t i = 0; i < trajectory_ptr_->points.size(); ++i) {
        Point p = trajectory_ptr_->points[i];
        double x = in_x - p.x;
        double y = in_y - p.y;
        double dist = hypot(x, y);
        if (dist < min_dis) {
          min_dis = dist;
          high = p.z;
        }
      }
    }
    std::cout << "init high:" << high << std::endl;

    return high;
  }

  void CreateSubmap() {
    tf::Pose T = pose_inc_;
    if (hypot(T.getOrigin().x(), T.getOrigin().y()) > cfg_.ndt.thresh_shift or
        tf::getYaw(T.getRotation()) > cfg_.ndt.thresh_rot or
        (sub_map_buf_.size() < cfg_.sub_map_scan_buf_size)) {
      if (front_lidar_data_buf_.size() > 0) {
        sensor_msgs::PointCloud2 current_front_msg =
            front_lidar_data_buf_.front();
        Cloud odom_cp;
        transPointCloud2ToPCL(current_front_msg, odom_cp);
        odom_cp =
            TransformCloud(odom_cp, cfg_.odom_frame, cfg_.front_lidar_frame,
                           current_front_msg.header.stamp);
        sub_map_buf_.push_back(odom_cp);
        if (sub_map_buf_.size() > cfg_.sub_map_scan_buf_size) {
          sub_map_buf_.pop_front();
        }
        front_lidar_data_buf_.pop_front();
      }
      if (rear_lidar_data_buf_.size() > 0) {
        sensor_msgs::PointCloud2 current_rear_msg =
            rear_lidar_data_buf_.front();
        Cloud odom_cp;
        transPointCloud2ToPCL(current_rear_msg, odom_cp);
        odom_cp =
            TransformCloud(odom_cp, cfg_.odom_frame, cfg_.rear_lidar_frame,
                           current_rear_msg.header.stamp);
        sub_map_buf_.push_back(odom_cp);
        if (sub_map_buf_.size() > cfg_.sub_map_scan_buf_size) {
          sub_map_buf_.pop_front();
        }
        rear_lidar_data_buf_.pop_front();
      }
      if (sub_map_buf_.size() == 0) return;

      sub_map_ptr_->clear();

      for (size_t i = 0; i < sub_map_buf_.size(); ++i) {
        *sub_map_ptr_ += sub_map_buf_.at(i);
      }
      voxel_grid_filter_.setInputCloud(sub_map_ptr_);
      voxel_grid_filter_.filter(*sub_map_ptr_);
      if (if_init_pose == true) RemovePlan(sub_map_ptr_);
      if (cfg_.debug)
        SubMapCloudPublish(*sub_map_ptr_, ros::Time::now(), cfg_.odom_frame);
    }
  }
  void MapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    ROS_INFO("Get map");
    pcl::fromROSMsg<Point>(*msg, *global_map_ptr_);
    LoadMapToScanMatcher();
  }

  void CropMap(const Eigen::Vector3f &origin) {
    static pcl::CropBox<Point> crop_box;
    Eigen::Matrix<float, 6, 1> edge;

    for (int i = 0; i < 3; ++i) {
      edge[2 * i] = cfg_.local_map_size[2 * i] + origin[i];
      edge[2 * i + 1] = cfg_.local_map_size[2 * i + 1] + origin[i];
    }

    crop_box.setMin(Eigen::Vector4f(edge[0], edge[2], edge[4], 1.0f));
    crop_box.setMax(Eigen::Vector4f(edge[1], edge[3], edge[5], 1.0f));
    crop_box.setInputCloud(global_map_ptr_);
    crop_box.filter(*local_map_ptr_);
  }

  void RemovePlan(Cloud::Ptr &cloud) {
    if (cloud->size() == 0) return;

    pcl::SampleConsensusModelPlane<Point>::Ptr model_plane(
        new pcl::SampleConsensusModelPlane<Point>(cloud));
    pcl::RandomSampleConsensus<Point> ransac(model_plane);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointCloud<Point>::Ptr cloud_no_plane(new pcl::PointCloud<Point>);
    vector<int> inliers;
    ransac.getInliers(inliers);

    Cloud::Ptr cloud_filtered(new Cloud);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
      bool remove = false;
      // 检查当前点是否在需要移除的索引列表中
      for (size_t j = 0; j < inliers.size(); ++j) {
        if (i == inliers[j]) {
          remove = true;
          inliers.erase(inliers.begin() + j);
          break;
        }
      }
      // 如果当前点不在需要移除的索引列表中，则添加到新的点云中
      if (!remove) {
        cloud_filtered->points.push_back(cloud->points[i]);
      }
    }
    *cloud = *cloud_filtered;
  }

  void InitPoseWithNDTCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    ROS_INFO("Initial pose set");
    ROS_WARN("Init pose from topic: x: %f, y: %f, z: %f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_WARN("Quaternion: x: %f, y: %f, z: %f, w: %f", 
             msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    double yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_WARN("Yaw: %f", yaw);

    auto &q = msg->pose.pose.orientation;
    auto &p = msg->pose.pose.position;
    double z = GetInitPoseHigh(p.x, p.y);
    tf::Pose map_to_base(tf::Quaternion(q.x, q.y, q.z, q.w),
                         tf::Vector3(p.x, p.y, z));
    map_to_odom_ = map_to_base * odom_to_base_.inverse();
    if_init_pose = true;
    ndt_.setStepSize(cfg_.ndt.step_size * 10);
    // ndt_.setStepSize(cfg_.ndt.resolution * 10);
  }

  void InitPoseByParam() {
    ROS_INFO("Initial pose from param");

    tf2::Quaternion quat;
    ROS_INFO("Init pose x y z: %f, %f, %f", cfg_.init_pose_x,
             cfg_.init_pose_x, cfg_.init_pose_x);
    ROS_INFO("Init pose roll pitch yaw: %f, %f, %f", cfg_.init_pose_roll,
             cfg_.init_pose_pitch, cfg_.init_pose_yaw);
    quat.setRPY(cfg_.init_pose_roll, cfg_.init_pose_pitch, cfg_.init_pose_yaw);
    quat.normalize();

    ROS_INFO("Quaternion (x, y, z, w): (%f, %f, %f, %f)", quat.x(), quat.y(),
             quat.z(), quat.w());

    tf::Pose map_to_base(
        tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()),
        tf::Vector3(cfg_.init_pose_x, cfg_.init_pose_y, cfg_.init_pose_z));
    map_to_odom_ = map_to_base;
    PublishMapToOdomTF();
    PublishWorldToMapTF();
  }

  void ScanMatchProcess() {
    static chrono::steady_clock::time_point t0, t1;
    if (cfg_.debug) t0 = chrono::steady_clock::now();
    tf::Pose now_transform;
    GetTransformWithTF(cfg_.odom_frame, cfg_.base_frame, ros::Time(0),
                       odom_to_base_);
    now_transform = map_to_odom_ * odom_to_base_;

    static tf::Pose lastNDTPose = now_transform;
    auto T = now_transform * lastNDTPose.inverse();
    pose_inc_ = T;
    CreateSubmap();

    // Eigen::Vector3d baseMapMat;
    // tf::vectorTFToEigen(now_transform.getOrigin(), baseMapMat);
    // CropMap(baseMapMat.cast<float>());

    if ((hypot(T.getOrigin().x(), T.getOrigin().y()) > cfg_.ndt.thresh_shift or
         tf::getYaw(T.getRotation()) > cfg_.ndt.thresh_rot) &&
        if_init_pose == false) {
      if (cfg_.scan_matcher_type == "ndt") {
        ScanMatchNdt(sub_map_ptr_, map_to_odom_);
      } else {
        ScanMatchICP(sub_map_ptr_, map_to_odom_);
      }
      lastNDTPose = now_transform;
    }

    if (if_init_pose == true) {
      static int cnt = 0;
      // RemovePlan(sub_map_ptr_);
      if (cfg_.scan_matcher_type == "ndt") {
        ScanMatchNdt(sub_map_ptr_, map_to_odom_);
      } else {
        ScanMatchICP(sub_map_ptr_, map_to_odom_);
      }
      ROS_INFO("init scan match");
      lastNDTPose = now_transform;
      cnt++;
      if (cnt > 10) {
        cnt = 0;
        if_init_pose = false;
        ndt_.setStepSize(cfg_.ndt.step_size);
        // ndt_.setStepSize(cfg_.ndt.resolution);
      }
    }
    PublishBasePose();
    if (cfg_.debug) {
      PublishNdtPosePathOdom();
      t1 = chrono::steady_clock::now();
      // ROS_INFO("scan match: %ldms",
      //          chrono::duration_cast<chrono::milliseconds>(t1 - t0).count());
    }
  }

  void CloudPublishData(Cloud &cloud_input, ros::Time time,
                        std::string frame_id) {
    sensor_msgs::PointCloud2Ptr cloud_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(cloud_input, *cloud_ptr);

    cloud_ptr->header.stamp = time;
    cloud_ptr->header.frame_id = frame_id;
    map_publisher_.publish(*cloud_ptr);
  }
  void SubMapCloudPublish(Cloud &cloud_input, ros::Time time,
                          std::string frame_id) {
    sensor_msgs::PointCloud2Ptr cloud_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(cloud_input, *cloud_ptr);

    cloud_ptr->header.stamp = time;
    cloud_ptr->header.frame_id = frame_id;
    sub_map_publisher_.publish(*cloud_ptr);
  }

  void PubGlobalMap() {
    if (load_map_success_ == true && map_publisher_.getNumSubscribers() != 0) {
      static int sub_num = 0;
      if (sub_num < map_publisher_.getNumSubscribers()) {
        CloudPublishData(*global_map_ptr_, ros::Time::now(), cfg_.world_frame);
        sub_num = map_publisher_.getNumSubscribers();
      } else {
        sub_num = map_publisher_.getNumSubscribers();
      }
    }
  }

  void FrontLaserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // ROS_INFO("FrontLaserCallback");
    cloud_time_front_ = msg->header.stamp;
    front_lidar_data_buf_.push_back(*msg);
    if (front_lidar_data_buf_.size() > 10) {
      front_lidar_data_buf_.pop_front();
    }
    PublishMapToOdomTF();
    PublishWorldToMapTF();
  }
  void RearLaserCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    rear_lidar_data_buf_.push_back(*msg);
    if (rear_lidar_data_buf_.size() > 10) {
      rear_lidar_data_buf_.pop_front();
    }
  }

  void ScanMatchNdt(Cloud::Ptr &pcPtr, tf::Pose &baseMap) {
    static chrono::steady_clock::time_point t0, t1;
    ndt_.setInputSource(pcPtr);
    // ndt_.setInputTarget(local_map_ptr_);
    Eigen::Affine3d baseMapMat;
    tf::poseTFToEigen(baseMap, baseMapMat);
    Cloud::Ptr outputCloudPtr(new Cloud);
    if (cfg_.debug) t0 = chrono::steady_clock::now();
    ndt_.align(*outputCloudPtr, baseMapMat.matrix().cast<float>());
    if (cfg_.debug) t1 = chrono::steady_clock::now();

    auto tNDT = ndt_.getFinalTransformation();

    tf::poseEigenToTF(Eigen::Affine3d(tNDT.cast<double>()), baseMap);

    if (cfg_.debug)
      ROS_INFO("NDT: %ldms",
               chrono::duration_cast<chrono::milliseconds>(t1 - t0).count());
  }
  void ScanMatchICP(Cloud::Ptr &pcPtr, tf::Pose &baseMap) {
    static chrono::steady_clock::time_point t0, t1;
    icp_.setInputSource(pcPtr);
    Eigen::Affine3d baseMapMat;
    tf::poseTFToEigen(baseMap, baseMapMat);
    Cloud::Ptr outputCloudPtr(new Cloud);
    if (cfg_.debug) t0 = chrono::steady_clock::now();
    icp_.align(*outputCloudPtr, baseMapMat.matrix().cast<float>());
    if (cfg_.debug) t1 = chrono::steady_clock::now();

    auto tICP = icp_.getFinalTransformation();

    tf::poseEigenToTF(Eigen::Affine3d(tICP.cast<double>()), baseMap);

    if (cfg_.debug)
      ROS_INFO("ICP: %ldms",
               chrono::duration_cast<chrono::milliseconds>(t1 - t0).count());
  }

  void PublishMapToOdomTF() {
    geometry_msgs::TransformStamped tfMsg;
    tfMsg.header.stamp = ros::Time::now();

    tfMsg.header.frame_id = cfg_.map_frame;
    tfMsg.child_frame_id = cfg_.odom_frame;

    tf::Pose map_to_base;
    map_to_base = map_to_odom_ * odom_to_base_;
    map_to_base.setOrigin(tf::Vector3(map_to_base.getOrigin().x(),
                                      map_to_base.getOrigin().y(), 0));

    tf::Pose map_to_odom;
    map_to_odom = map_to_base * odom_to_base_.inverse();

    tfMsg.transform.translation.x = map_to_odom.getOrigin().x();
    tfMsg.transform.translation.y = map_to_odom.getOrigin().y();
    tfMsg.transform.translation.z = map_to_odom.getOrigin().z();
    tfMsg.transform.rotation.x = map_to_odom.getRotation().x();
    tfMsg.transform.rotation.y = map_to_odom.getRotation().y();
    tfMsg.transform.rotation.z = map_to_odom.getRotation().z();
    tfMsg.transform.rotation.w = map_to_odom.getRotation().w();
    br_.sendTransform(tfMsg);
  }

  void PublishWorldToMapTF() {
    geometry_msgs::TransformStamped tfMsg;
    tf::Pose localization_transform;
    localization_transform = map_to_odom_ * odom_to_base_;
    tfMsg.header.stamp = ros::Time::now();
    tfMsg.header.frame_id = cfg_.world_frame;
    tfMsg.child_frame_id = cfg_.map_frame;
    tfMsg.transform.translation.x = 0;
    tfMsg.transform.translation.y = 0;
    tfMsg.transform.translation.z = localization_transform.getOrigin().z();
    tfMsg.transform.rotation.x = 0;
    tfMsg.transform.rotation.y = 0;
    tfMsg.transform.rotation.z = 0;
    tfMsg.transform.rotation.w = 1;
    br_.sendTransform(tfMsg);
  }

  void PublishBasePose() {
    tf::Pose localization_transform;
    localization_transform = map_to_odom_ * odom_to_base_;

    geometry_msgs::PoseStamped localization_pose_msg;
    localization_pose_msg.header.frame_id = cfg_.map_frame;
    localization_pose_msg.header.stamp = cloud_time_front_;
    tf::poseTFToMsg(localization_transform, localization_pose_msg.pose);

    localization_pose_publisher_.publish(localization_pose_msg);
  }
  void PublishNdtPosePathOdom() {
    tf::Pose localization_transform;
    localization_transform = map_to_odom_ * odom_to_base_;
    geometry_msgs::PoseStamped localization_pose_msg;
    localization_pose_msg.header.frame_id = cfg_.map_frame;
    localization_pose_msg.header.stamp = cloud_time_front_;
    tf::poseTFToMsg(localization_transform, localization_pose_msg.pose);

    if (!localization_path_.poses.empty()) {
      geometry_msgs::PoseStamped last_pose_msg =
          localization_path_.poses.back();

      Eigen::Vector3d displacement_vector(
          localization_pose_msg.pose.position.x - last_pose_msg.pose.position.x,
          localization_pose_msg.pose.position.y - last_pose_msg.pose.position.y,
          localization_pose_msg.pose.position.z -
              last_pose_msg.pose.position.z);

      if (displacement_vector.norm() > path_threshold_) {
        localization_path_.poses.push_back(localization_pose_msg);
      }
    } else {
      localization_path_.poses.push_back(localization_pose_msg);
    }
    localization_path_publisher_.publish(localization_path_);

    nav_msgs::Odometry localization_odom_msg;
    localization_odom_msg.header.frame_id = cfg_.map_frame;
    localization_odom_msg.header.stamp = cloud_time_front_;
    localization_odom_msg.child_frame_id = cfg_.base_frame;
    localization_odom_msg.pose.pose = localization_pose_msg.pose;
    localization_odom_msg.pose.pose.position.z = 0.0;
    localization_odom_publisher_.publish(localization_odom_msg);
  }

  bool GetTransformWithTF(const std::string &source_frame,
                          const std::string &target_frame, ros::Time time,
                          tf::Pose &out_transform) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(source_frame, target_frame, time,
                                             ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(10, "GetTransformWithTF error: %s", ex.what());
      return false;
    }
    tf::Pose tf_transform(tf::Quaternion(transform.transform.rotation.x,
                                         transform.transform.rotation.y,
                                         transform.transform.rotation.z,
                                         transform.transform.rotation.w),

                          tf::Vector3(transform.transform.translation.x,
                                      transform.transform.translation.y,
                                      transform.transform.translation.z));
    out_transform = tf_transform;
    return true;
  }
  bool GetTransformWithTF(const std::string &source_frame,
                          const std::string &target_frame, ros::Time time,
                          Eigen::Affine3d &out_transform) {
    tf::Pose tf_transform;
    if (GetTransformWithTF(source_frame, target_frame, time, tf_transform) ==
        false) {
      return false;
    }
    tf::transformTFToEigen(tf_transform, out_transform);
    return true;
  }

  Cloud TransformCloud(const Cloud &cloud, const std::string &source_frame,
                       const std::string &target_frame, ros::Time time) {
    geometry_msgs::TransformStamped base_link_to_scan_transform;
    Cloud out_sensor_points;
    Eigen::Affine3d transform;
    if (GetTransformWithTF(source_frame, target_frame, time, transform) == true)
      pcl::transformPointCloud(cloud, out_sensor_points, transform);
    return out_sensor_points;
  }

  bool LoadPCDFile(Cloud::Ptr cloud_ptr_, std::string pcd_file_name) {
    if (pcl::io::loadPCDFile<Point>(pcd_file_name, *cloud_ptr_) == -1) {
      PCL_ERROR("Couldn't read file %s \n", pcd_file_name.c_str());
      return false;
    }

    PCL_INFO("Load File %s  success\n", pcd_file_name.c_str());
    return true;
  }

  void LoadMapToScanMatcher() {
    voxel_grid_filter_.setInputCloud(global_map_ptr_);
    voxel_grid_filter_.filter(*global_map_ptr_);
    if (cfg_.scan_matcher_type == "ndt") {
      ndt_.setInputTarget(global_map_ptr_);
    } else {
      pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>);
      kdtree->setInputCloud(global_map_ptr_);
      icp_.setSearchMethodTarget(kdtree);
      icp_.setInputTarget(global_map_ptr_);
    }
  }
};

void Localization::RunLocalization() {
  tf2_ros::TransformListener tfListener(tf_buffer_);
  ros::Rate r(10);
  if (LoadPCDFile(global_map_ptr_, cfg_.map_load_file)) {
    // Eigen::Affine3f transCur = pcl::getTransformation(0, 0, 1.3, M_PI, 0, 0);
    // pcl::transformPointCloud(*global_map_ptr_, *global_map_ptr_, transCur);
    load_map_success_ = true;
    LoadMapToScanMatcher();
  } else {
    load_map_success_ = false;
  }
  if (LoadPCDFile(trajectory_ptr_, cfg_.traj_load_file)) {
    load_traj_success_ = true;
  } else {
    load_traj_success_ = false;
  }
  while (ros::ok()) {
    PubGlobalMap();

    ScanMatchProcess();

    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cube_localization");
  ros::NodeHandle nh("~");

  Localization localization(nh);
  localization.RunLocalization();

  return 0;
}