#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <thread>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <sensor_msgs/JointState.h>
#include <unordered_map>
#include <vector>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicArmLowState = "rt/lowstate";
static const std::string kSavedPositionFile = "saved_positions.txt";

constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;

enum JointIndex {
  kRightHipYaw = 8,
  kRightHipRoll = 0,
  kRightHipPitch = 1,
  kRightKnee = 2,
  kRightAnkle = 11,
  kLeftHipYaw = 7,
  kLeftHipRoll = 3,
  kLeftHipPitch = 4,
  kLeftKnee = 5,
  kLeftAnkle = 10,
  kWaistYaw = 6,
  kNotUsedJoint = 9,
  kRightShoulderPitch = 12,
  kRightShoulderRoll = 13,
  kRightShoulderYaw = 14,
  kRightElbow = 15,
  kLeftShoulderPitch = 16,
  kLeftShoulderRoll = 17,
  kLeftShoulderYaw = 18,
  kLeftElbow = 19,
};

class RobotJointPub {
 public:
  RobotJointPub() {
    ros::NodeHandle nh;
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    lowstate_subscriber.reset(
        new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
            kTopicArmLowState));
    lowstate_subscriber->InitChannel(
        std::bind(&RobotJointPub::LowStateMessageHandler, this,
                  std::placeholders::_1),
        1);

    joint_state_publisher_ =
        nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        
    for (size_t i = 0; i < joint_map_.size(); i++) {
        joint_state_msg_.name.push_back(JointName(static_cast<JointIndex>(i)));
    }
  }

 private:
void publish_joint_states() {
    joint_state_msg_.header.stamp = ros::Time::now();  // Set timestamp
    
    // 获取 joint_map_ 和 low_state.motor_state() 的大小
    size_t num_joints = std::min(joint_map_.size(), low_state.motor_state().size());
    joint_state_msg_.position.resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i) {
        joint_state_msg_.position[i] = low_state.motor_state()[i].q();
    }

    joint_state_publisher_.publish(joint_state_msg_);
}

  void LowStateMessageHandler(const void *message) {
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    publish_joint_states();
  }

  std::string JointName(JointIndex joint_index) {
    auto it = joint_map_.find(joint_index);
    if (it != joint_map_.end()) {
      return it->second;
    } else {
      return "unknown_joint";
    }
  }

  const std::unordered_map<JointIndex, std::string> joint_map_ = {
      {JointIndex::kRightHipYaw, "right_hip_yaw_joint"},
      {JointIndex::kRightHipRoll, "right_hip_roll_joint"},
      {JointIndex::kRightHipPitch, "right_hip_pitch_joint"},
      {JointIndex::kRightKnee, "right_knee_joint"},
      {JointIndex::kRightAnkle, "right_ankle_joint"},
      {JointIndex::kLeftHipYaw, "left_hip_yaw_joint"},
      {JointIndex::kLeftHipRoll, "left_hip_roll_joint"},
      {JointIndex::kLeftHipPitch, "left_hip_pitch_joint"},
      {JointIndex::kLeftKnee, "left_knee_joint"},
      {JointIndex::kLeftAnkle, "left_ankle_joint"},
      {JointIndex::kWaistYaw, "torso_joint"},
      {JointIndex::kRightShoulderPitch, "right_shoulder_pitch_joint"},
      {JointIndex::kRightShoulderRoll, "right_shoulder_roll_joint"},
      {JointIndex::kRightShoulderYaw, "right_shoulder_yaw_joint"},
      {JointIndex::kRightElbow, "right_elbow_joint"},
      {JointIndex::kLeftShoulderPitch, "left_shoulder_pitch_joint"},
      {JointIndex::kLeftShoulderRoll, "left_shoulder_roll_joint"},
      {JointIndex::kLeftShoulderYaw, "left_shoulder_yaw_joint"},
      {JointIndex::kLeftElbow, "left_elbow_joint"},
      {JointIndex::kNotUsedJoint, "not_used_joint"}};

  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      lowstate_subscriber;
  ros::Publisher joint_state_publisher_;
  sensor_msgs::JointState joint_state_msg_;
  unitree_go::msg::dds_::LowState_ low_state{};
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robot_arm_joint_pub");
  RobotJointPub node;
  ros::spin();
  return 0;
}
