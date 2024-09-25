#include <array>
#include <chrono>
#include <fstream>
#include <humanoid_msgs/SavePose.h>
#include <humanoid_msgs/MovePose.h>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <thread>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <vector>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicArmLowState = "rt/lowstate";
static const std::string kSavedPositionFile = "saved_positions.txt";

constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;

enum JointIndex
{
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

class RobotArmController
{
public:
  RobotArmController(ros::NodeHandle &nh)
  {
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    // ROS1 services
    save_position_service_ = nh.advertiseService(
        "save_arm_position", &RobotArmController::save_position, this);
    move_to_position_service_ = nh.advertiseService(
        "move_to_saved_position", &RobotArmController::move_to_position, this);

    arm_sdk_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
            kTopicArmSDK));
    arm_sdk_publisher_->InitChannel();

    lowstate_subscriber.reset(
        new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
            kTopicArmLowState));
    lowstate_subscriber->InitChannel(
        std::bind(&RobotArmController::LowStateMessageHandler, this, std::placeholders::_1), 1);

    // Load saved positions from file
    load_saved_positions();
  }

private:
  bool save_position(humanoid_msgs::SavePose::Request &req,
                     humanoid_msgs::SavePose::Response &res)
  {
    int pose_index = req.pose_index;

    if (pose_index < 0)
    {
      res.success = false;
      res.message = "Pose index cannot be negative.";
      return false;
    }

    if ((size_t)pose_index >= saved_positions.size())
    {
      saved_positions.resize(pose_index + 1);
    }

    for (size_t i = 0; i < arm_joints_.size(); ++i)
    {
      saved_positions[pose_index].at(i) =
          low_state.motor_state().at(arm_joints_.at(i)).q();
    }
    save_positions_to_file();
    res.success = true;
    res.message = "Arm position saved at index " + std::to_string(pose_index) + ".";
    return true;
  }

  void LowStateMessageHandler(const void *message)
  {
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    // ROS_INFO("Get data");
  }

  bool move_to_position(humanoid_msgs::MovePose::Request &req,
                        humanoid_msgs::MovePose::Response &res)
  {
    int pose_index = req.pose_index;

    if (pose_index < 0 || (size_t)pose_index >= saved_positions.size())
    {
      res.success = false;
      res.message = "Pose index out of range.";
      return false;
    }

    auto sleep_time =
        std::chrono::milliseconds(static_cast<int>(control_dt_ / 0.001f));
    std::array<float, 9> current_jpos_des{};

    for (size_t i = 0; i < current_jpos_des.size(); ++i)
    {
      current_jpos_des.at(i) =
          low_state.motor_state().at(arm_joints_.at(i)).q();
    }

    float weight = 0.f;
    static bool if_first = true;
    for (int i = 0; i < num_time_steps_; ++i)
    {
      if (if_first)
      {
        // increase weight
        weight += delta_weight;
        weight = std::clamp(weight, 0.f, 1.f);
        std::cout << weight << std::endl;

        // set weight
        msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight * weight);
      }

      for (size_t j = 0; j < saved_positions[pose_index].size(); ++j)
      {
        current_jpos_des.at(j) += std::clamp(
            saved_positions[pose_index].at(j) - current_jpos_des.at(j),
            min_joint_delta_, max_joint_delta_);
      }

      for (size_t j = 0; j < saved_positions[pose_index].size(); ++j)
      {
        msg_.motor_cmd().at(arm_joints_.at(j)).q(current_jpos_des.at(j));
        msg_.motor_cmd().at(arm_joints_.at(j)).dq(dq_);
        msg_.motor_cmd().at(arm_joints_.at(j)).kp(kp_);
        msg_.motor_cmd().at(arm_joints_.at(j)).kd(kd_);
        msg_.motor_cmd().at(arm_joints_.at(j)).tau(tau_ff_);
      }
      // ROS_INFO("%2f", (float)i / num_time_steps_);

      arm_sdk_publisher_->Write(msg_);
      std::this_thread::sleep_for(sleep_time);
    }

    if_first = false;
    res.success = true;
    res.message = "Arm moved to saved position at index " +
                  std::to_string(pose_index) + ".";
    return true;
  }

  void save_positions_to_file()
  {
    std::ofstream file(kSavedPositionFile);
    if (file.is_open())
    {
      for (const auto &pose : saved_positions)
      {
        for (const auto &pos : pose)
        {
          file << pos << ' ';
        }
        file << '\n';
      }
      file.close();
    }
    else
    {
      ROS_ERROR("Unable to open file for saving positions.");
    }
  }

  void load_saved_positions()
  {
    std::ifstream file(kSavedPositionFile);
    if (file.is_open())
    {
      std::string line;
      while (std::getline(file, line))
      {
        std::istringstream iss(line);
        std::array<float, 9> pose{};
        for (auto &pos : pose)
        {
          iss >> pos;
        }
        saved_positions.push_back(pose);
      }
      file.close();
    }
    else
    {
      ROS_WARN("No saved positions file found. Using empty positions.");
    }
  }

  ros::ServiceServer save_position_service_;
  ros::ServiceServer move_to_position_service_;

  std::array<JointIndex, 9> arm_joints_ = {
      JointIndex::kLeftShoulderPitch, JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftShoulderYaw, JointIndex::kLeftElbow,
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightShoulderYaw, JointIndex::kRightElbow,
      JointIndex::kWaistYaw};

  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>
      arm_sdk_publisher_;
  unitree_go::msg::dds_::LowCmd_ msg_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      lowstate_subscriber;
  float control_dt_ = 0.02f;
  float kp_ = 60.f;
  float kd_ = 1.5f;
  float dq_ = 0.f;
  float tau_ff_ = 0.f;
  float max_joint_delta_ = 0.5f * control_dt_;
  float min_joint_delta_ = -0.5f * control_dt_;
  int num_time_steps_ = static_cast<int>(5.0f / control_dt_);
  float weight = 0.f;
  float weight_rate = 0.2f;
  float delta_weight = weight_rate * control_dt_;
  unitree_go::msg::dds_::LowState_ low_state{};
  std::vector<std::array<float, 9>> saved_positions{};
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robot_arm_controller");
  ros::NodeHandle nh;
  RobotArmController controller(nh);
  ros::spin();
  return 0;
}
