#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"
 
using namespace std::chrono_literals;
 
// Define constants
const std::vector<std::string> arm_joints = {
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
};
 

class ExampleJointTrajectoryPublisherCpp : public rclcpp::Node
{
public:
    ExampleJointTrajectoryPublisherCpp()
        : Node("planning_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node started. Publishing to /arm_moveit_controller/joint_trajectory");
        // Create the publisher of the desired arm and gripper goal poses
        arm_pose_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_moveit_controller/joint_trajectory", 1);
 
        // Create a timer to periodically call the timerCallback function
        timer_ = create_wall_timer(5s, std::bind(&ExampleJointTrajectoryPublisherCpp::timerCallback, this));
 
        frame_id_ = "base_link";
 
        // Desired time from the trajectory start to arrive at the trajectory point.
        // Needs to be less than or equal to the timer period above to allow
        // the robotic arm to smoothly transition between points.
        duration_sec_ = 2;
        duration_nanosec_ = 0.5 * 1e9;  // (seconds * 1e9)
 
        // Set the desired goal poses for the robotic arm.
        arm_positions_ = {
            {0.0, 0.0, 0.0},  // Home location
            {0.0, 0.0, -1.56},  // UP_45
            {0.0, -1.56, 0.0},
        };
 

 
        this->declare_parameter<int>("state_cmd", 0);
        RCLCPP_INFO(this->get_logger(), "Available states: 0:Home, 1:UP_45, 2:Custom");
    }
 
private:
    void timerCallback()
    {
        int state_cmd = this->get_parameter("state_cmd").as_int();
        if (state_cmd < 0 || state_cmd >= static_cast<int>(arm_positions_.size())) {
            RCLCPP_WARN(this->get_logger(), "state_cmd (%d) out of range, using 0", state_cmd);
            state_cmd = 0;
        }
        RCLCPP_INFO(this->get_logger(), "Publishing state_cmd: %d -> [%.2f, %.2f, %.2f]", state_cmd, arm_positions_[state_cmd][0], arm_positions_[state_cmd][1], arm_positions_[state_cmd][2]);
        // Create new JointTrajectory messages for arm and gripper
        auto msg_arm = trajectory_msgs::msg::JointTrajectory();
        msg_arm.header.frame_id = frame_id_;
        msg_arm.joint_names = arm_joints;
 
 
        // Create JointTrajectoryPoints for arm and gripper
        auto point_arm = trajectory_msgs::msg::JointTrajectoryPoint();
        point_arm.positions = arm_positions_[state_cmd];
        point_arm.time_from_start = rclcpp::Duration(duration_sec_, duration_nanosec_);
        msg_arm.points.push_back(point_arm);
        arm_pose_publisher_->publish(msg_arm);
 
    }
 
    // Publishers for arm and gripper joint trajectories
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pose_publisher_;
 
    // Timer for periodic callback
    rclcpp::TimerBase::SharedPtr timer_;
 
    // Frame ID for the joint trajectories
    std::string frame_id_;
 
    // Duration for each trajectory point
    int duration_sec_;
    int duration_nanosec_;
 
    // Desired goal poses for the robotic arm and gripper
    std::vector<std::vector<double>> arm_positions_;
 
    // Index to keep track of the current trajectory point
    size_t index_;
};
 
int main(int argc, char * argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);
 
    // Create an instance of the ExampleJointTrajectoryPublisherCpp node
    auto node = std::make_shared<ExampleJointTrajectoryPublisherCpp>();
 
    // Spin the node to execute the callbacks
    rclcpp::spin(node);
 
    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
 
    return 0;
}