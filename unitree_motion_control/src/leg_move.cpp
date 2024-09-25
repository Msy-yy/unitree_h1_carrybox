#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <unitree/robot/h1/loco/h1_loco_api.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>

class RobotControlNode {
public:
  RobotControlNode() {
    // Initialize ROS1 subscriber to cmd_vel topic
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RobotControlNode::cmdVelCallback, this);

    // Initialize ROS1 services for robot actions
    damp_srv_ = nh_.advertiseService("/damp", &RobotControlNode::dampCallback, this);
    start_srv_ = nh_.advertiseService("/start", &RobotControlNode::startCallback, this);
    stand_up_srv_ = nh_.advertiseService("/stand_up", &RobotControlNode::standUpCallback, this);
    zero_torque_srv_ = nh_.advertiseService("/zero_torque", &RobotControlNode::zeroTorqueCallback, this);
    stop_move_srv_ = nh_.advertiseService("/stop_move", &RobotControlNode::stopMoveCallback, this);
    high_stand_srv_ = nh_.advertiseService("/high_stand", &RobotControlNode::highStandCallback, this);
    low_stand_srv_ = nh_.advertiseService("/low_stand", &RobotControlNode::lowStandCallback, this);
    balance_stand_srv_ = nh_.advertiseService("/balance_stand", &RobotControlNode::balanceStandCallback, this);

    // Initialize Unitree robot client
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    client_ = new unitree::robot::h1::LocoClient;
    client_->Init();
    client_->SetTimeout(10.f);

    ROS_INFO("Robot control node started.");
  }
~RobotControlNode(){
    delete client_;
}
private:


  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float omega = msg->angular.z;
    float duration = 1.0f; // Default duration
    client_->SetVelocity(vx, vy, omega, duration);
    ROS_INFO("Set velocity: vx=%.2f, vy=%.2f, omega=%.2f", vx, vy, omega);
  }

  // Service Callbacks for robot actions
  bool dampCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->Damp();
    res.success = true;
    res.message = "Damp action triggered.";
    return true;
  }

  bool startCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->Start();
    res.success = true;
    res.message = "Start action triggered.";
    return true;
  }

  bool standUpCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->StandUp();
    res.success = true;
    res.message = "Stand up action triggered.";
    return true;
  }

  bool zeroTorqueCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->ZeroTorque();
    res.success = true;
    res.message = "Zero torque action triggered.";
    return true;
  }

  bool stopMoveCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->StopMove();
    res.success = true;
    res.message = "Stop move action triggered.";
    return true;
  }

  bool highStandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->HighStand();
    res.success = true;
    res.message = "High stand action triggered.";
    return true;
  }

  bool lowStandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->LowStand();
    res.success = true;
    res.message = "Low stand action triggered.";
    return true;
  }

  bool balanceStandCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    client_->BalanceStand();
    res.success = true;
    res.message = "Balance stand action triggered.";
    return true;
  }


  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::ServiceServer damp_srv_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stand_up_srv_;
  ros::ServiceServer zero_torque_srv_;
  ros::ServiceServer stop_move_srv_;
  ros::ServiceServer high_stand_srv_;
  ros::ServiceServer low_stand_srv_;
  ros::ServiceServer balance_stand_srv_;
  unitree::robot::h1::LocoClient *client_;
  
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_control_node");
  RobotControlNode robot_control_node;
  ros::spin();
  return 0;
}

