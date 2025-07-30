#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>

class JointStateReader : public rclcpp::Node {
public:
  JointStateReader() : Node("joint_state_reader") {
    // Khởi tạo MoveGroupInterface trong hàm riêng để đảm bảo node đã sẵn sàng
    initMoveGroup();
    // Subscribe /joint_states
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&JointStateReader::joint_state_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "JointStateReader initialized");
  }

private:
  void initMoveGroup() {
    try {
      // Khởi tạo MoveGroupInterface sau khi node được tạo
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "arm_group"); // Thay "arm_group" bằng tên group của bạn
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::string output = "Joint States:\n";
    for (size_t i = 0; i < msg->name.size(); ++i) {
      output += msg->name[i] + ": " + std::to_string(msg->position[i]) + "\n";
    }
    RCLCPP_INFO(this->get_logger(), "%s", output.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Tạo node bằng shared_ptr trước khi sử dụng
  auto node = std::make_shared<JointStateReader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}