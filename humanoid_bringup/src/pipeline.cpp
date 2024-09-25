#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <humanoid_msgs/MovePose.h>
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include <tf/tf.h> 
#include <fstream>

struct Pose
{
    double x;
    double y;
    double theta;
};
enum class RobotState {
    MOVING_TO_GOAL,
    ALIGNING_TO_QR,
    PICKING_BOX,
    DONE
};


class RobotBoxTask
{
public:
    RobotBoxTask(ros::NodeHandle &n)
    {
        // Node handle
        nh_ = n;

        // Load saved points
        

        // Service for setting start and goal points
        set_start_points_srv_ = nh_.advertiseService("set_start_points", &RobotBoxTask::set_start_points_callback, this);
        set_goal_points_srv_ = nh_.advertiseService("set_goal_points", &RobotBoxTask::set_goal_points_callback, this);
        start_task_srv_ = nh_.advertiseService("set_start_task", &RobotBoxTask::set_start_task_callback, this);

        // Publisher for robot movement
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

        // Subscriber for QR code detection and localizer pose
        qr_code_sub_ = nh_.subscribe("qr_code_pose", 10, &RobotBoxTask::qr_code_callback, this);
        pose_subscription_ = nh_.subscribe("localizer_pose", 10, &RobotBoxTask::pose_callback, this);

        // Timer to move robot if the goal point is set
        timer_ = nh_.createTimer(ros::Duration(0.5), &RobotBoxTask::controlLoop, this);
        load_points();
    }

private:
    // Node handle
    ros::NodeHandle nh_;

    // Variables to store start and goal points
    geometry_msgs::PoseStamped start_point_, goal_point_, current_position_, qr_code_pose_;

    // ROS publishers, subscribers, and services
    ros::Publisher pose_pub_;
    ros::Subscriber qr_code_sub_;
    ros::Subscriber pose_subscription_;
    ros::ServiceServer set_start_points_srv_;
    ros::ServiceServer set_goal_points_srv_;
    ros::ServiceServer start_task_srv_;
    ros::Timer timer_;
    int switch_num = 0;
    RobotState current_state_;
    bool qr_code_received_=false;

    bool has_goal_ = false;
    bool start_task = false;

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_position_ = *msg;
    }

    bool set_start_points_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            start_point_.pose = current_position_.pose; // Set the start position
        }
        save_points();
        ROS_INFO("Save start point");
        res.success = true;
        return true;
    }

    bool set_goal_points_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            goal_point_.pose = current_position_.pose; // Set the goal position
        }
        save_points();
        ROS_INFO("Save goal point");
        res.success = true;
        return true;
    }

    bool set_start_task_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        start_task = req.data;
        if(start_task==false){
            pose_pub_.publish(current_position_);
            current_state_ = RobotState::DONE;
        } else {
            current_state_ = RobotState::MOVING_TO_GOAL;
        }
        res.success = true;
        return true;
    }
    bool ArePosesClose(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2, double position_tolerance = 0.05, double yaw_tolerance = 0.1)
    {
        // Calculate Euclidean distance between x and y positions
        double dx = pose1.pose.position.x - pose2.pose.position.x;
        double dy = pose1.pose.position.y - pose2.pose.position.y;
        double position_distance = std::sqrt(dx * dx + dy * dy);

        // Check if the x, y positions are within tolerance
        if (position_distance > position_tolerance)
        {
            return false;
        }

        // Calculate the yaw from the quaternion using tf::getYaw
        double yaw1 = tf::getYaw(pose1.pose.orientation);
        double yaw2 = tf::getYaw(pose2.pose.orientation);

        // Calculate the difference in yaw
        double yaw_diff = std::fabs(yaw1 - yaw2);

        // Normalize the yaw difference to be within [0, pi]
        if (yaw_diff > M_PI)
        {
            yaw_diff = 2 * M_PI - yaw_diff;
        }

        // Check if the yaw difference is within tolerance
        if (yaw_diff > yaw_tolerance)
        {
            return false;
        }

        // If both position and yaw are within tolerances, the poses are close
        return true;
    }

    // Timer callback to move robot to goal
    void controlLoop(const ros::TimerEvent&) {
        if(start_task==false)
            return ;
        switch (current_state_) {
            case RobotState::MOVING_TO_GOAL:
                // Publish the goal position
                pose_pub_.publish(goal_point_);
                ROS_INFO("Moving to goal...");

                // Check if we have reached the goal position
                if (ArePosesClose(goal_point_, current_position_)) {
                    ROS_INFO("Reached the goal. Transitioning to QR code alignment...");
                    current_state_ = RobotState::ALIGNING_TO_QR;
                    qr_code_received_ = false;
                }
                break;

            case RobotState::ALIGNING_TO_QR:
                // Publish the QR code position to approach it
                if (qr_code_received_) {
                    
                    pose_pub_.publish(qr_code_pose_);
                    ROS_INFO("Aligning with QR code...");

                    if (ArePosesClose(qr_code_pose_, current_position_)) {
                        ROS_INFO("Aligned with QR code. Transitioning to pick up box...");
                        current_state_ = RobotState::PICKING_BOX;
                        qr_code_received_ = false;
                    }
                }
                break;

            case RobotState::PICKING_BOX:
                // Call function to move the robot's arm
                SetArmToPose(1);
                SetArmToPose(2);
                ROS_INFO("Picking up the box...");

                current_state_ = RobotState::DONE;
                break;

            case RobotState::DONE:
                ROS_INFO("Task completed.");
                break;
        }
    }

    // Callback to handle QR code detection
    void qr_code_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        qr_code_pose_ = *msg;
        qr_code_received_ = true;
    }

    // Save start and goal points to a file
    void save_points()
    {
        std::ofstream file("points.txt");
        if (file.is_open())
        {
            std::cout << "File opened successfully" << std::endl;
            file << start_point_.pose.position.x << " " << start_point_.pose.position.y << " "
                << start_point_.pose.position.z << " "
                << start_point_.pose.orientation.x << " " << start_point_.pose.orientation.y << " "
                << start_point_.pose.orientation.z << " " << start_point_.pose.orientation.w << "\n";
                
            file << goal_point_.pose.position.x << " " << goal_point_.pose.position.y << " "
                << goal_point_.pose.position.z << " "
                << goal_point_.pose.orientation.x << " " << goal_point_.pose.orientation.y << " "
                << goal_point_.pose.orientation.z << " " << goal_point_.pose.orientation.w << "\n";
                
            if (file.fail())
            {
                std::cerr << "Failed to write to file." << std::endl;
            }
            else
            {
                std::cout << "Saved points successfully" << std::endl;
            }
            
            file.close();
        }
        else
        {
            std::cerr << "Failed to open file." << std::endl;
        }
    }


    // Load start and goal points from a file
    void load_points()
    {
        std::ifstream file("points.txt");
        if (file.is_open())
        {
            file >> start_point_.pose.position.x >> start_point_.pose.position.y >> start_point_.pose.position.z >> start_point_.pose.orientation.x >> start_point_.pose.orientation.y >> start_point_.pose.orientation.z >> start_point_.pose.orientation.w;
            file >> goal_point_.pose.position.x >> goal_point_.pose.position.y >> goal_point_.pose.position.z >> goal_point_.pose.orientation.x >> goal_point_.pose.orientation.y >> goal_point_.pose.orientation.z >> goal_point_.pose.orientation.w;
            file.close();
        }
    }

    void SetArmToPose(int pos_index)
    {
        ros::ServiceClient client = nh_.serviceClient<humanoid_msgs::MovePose>("move_to_saved_position");
        humanoid_msgs::MovePose srv;
        srv.request.pose_index = pos_index;

        if (client.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("Move arm to pos%d successfully", pos_index);
            }
            else
            {
                ROS_WARN("Failed to move arm to pos%d ");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service move_to_saved_position");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_box_task");

    ros::NodeHandle nh;
    RobotBoxTask robot_box_task(nh);
    ros::spin();
    return 0;
}
