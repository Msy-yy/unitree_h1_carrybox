#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <math.h>

class OmnidirectionalChassisController {
public:
    enum State {
        ROTATE_TO_TARGET = 1,
        MOVE_TO_TARGET,
        ALIGN_TO_GOAL,
        FINE_ADJUST_TO_GOAL,
    };

    OmnidirectionalChassisController() : current_state_(ROTATE_TO_TARGET), goal_received_(false) {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &OmnidirectionalChassisController::goalCallback, this);
        angular_tolerance_ = 0.05;
        linear_tolerance_ = 0.1;  // ���������̶ȵ����ø�С�Ա㾫ȷ��λ
        max_linear_speed_ = 0.8;
        max_angular_speed_ = 1.0;
    }

    // Ŀ���ص�����
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        target_theta_ = tf::getYaw(msg->pose.orientation);
        goal_received_ = true;
        current_state_ = ROTATE_TO_TARGET;
        ROS_INFO("Received new goal: x = %f, y = %f, theta = %f", target_x_, target_y_, target_theta_);
    }

void moveToGoal() {
    ros::Rate rate(10);
    while (ros::ok()) {
        if (!goal_received_) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }


        tf::StampedTransform transform;
        try {
            listener_.lookupTransform("map", "base_link", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }

        double dx = target_x_ - transform.getOrigin().x();
        double dy = target_y_ - transform.getOrigin().y();
        double distance_to_goal = sqrt(dx * dx + dy * dy);
        double goal_angle = atan2(dy, dx);
        double current_theta = tf::getYaw(transform.getRotation());
        double angle_diff = normalizeAngle(goal_angle - current_theta);

        geometry_msgs::Twist cmd_vel_msg;

        switch (current_state_) {
            case ROTATE_TO_TARGET:

                if (fabs(angle_diff) > angular_tolerance_) {
                    cmd_vel_msg.angular.z = std::copysign(std::min(max_angular_speed_, fabs(angle_diff)), angle_diff);
                } else {
                    current_state_ = MOVE_TO_TARGET;
                }
                break;

            case MOVE_TO_TARGET:

                if (distance_to_goal > linear_tolerance_) {
                    double linear_speed = std::min(max_linear_speed_, distance_to_goal);
                    cmd_vel_msg.linear.x = linear_speed * cos(angle_diff);
                    cmd_vel_msg.linear.y = linear_speed * sin(angle_diff);
                    // ���ֽǶȵ���
                    cmd_vel_msg.angular.z = std::copysign(std::min(max_angular_speed_, fabs(angle_diff)), angle_diff);
                } else {
                    current_state_ = ALIGN_TO_GOAL;
                }
                break;

            case ALIGN_TO_GOAL:{
                double final_angle_diff = normalizeAngle(target_theta_ - current_theta);
                if (fabs(final_angle_diff) > angular_tolerance_) {
                    cmd_vel_msg.angular.z = std::copysign(std::min(max_angular_speed_, fabs(final_angle_diff)), final_angle_diff);
                } else {
                    // ����΢��λ��״̬
                    current_state_ = FINE_ADJUST_TO_GOAL;
                }}
                break;

            case FINE_ADJUST_TO_GOAL:{
                if (distance_to_goal > linear_tolerance_) {
                    double fine_linear_speed = std::min(0.1, distance_to_goal);
                    cmd_vel_msg.linear.x = fine_linear_speed * cos(angle_diff);
                    cmd_vel_msg.linear.y = fine_linear_speed * sin(angle_diff);
                } else {

                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.linear.y = 0.0;
                    cmd_vel_msg.angular.z = 0.0;
                    ROS_INFO("Reached goal with fine adjustment!");
                    goal_received_ = false; 
                }
                }
                break;
        }

        cmd_vel_pub_.publish(cmd_vel_msg);
        rate.sleep();
    }
}


private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber goal_sub_;
    tf::TransformListener listener_;

    State current_state_;
    bool goal_received_;
    double target_x_;
    double target_y_;
    double target_theta_;
    double angular_tolerance_;
    double linear_tolerance_;
    double max_linear_speed_;
    double max_angular_speed_;

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "omni_chassis_controller");

    OmnidirectionalChassisController controller;

    // ��ѭ���ȴ���ִ��Ŀ���
    controller.moveToGoal();

    return 0;
}
