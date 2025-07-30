#include <rclcpp/rclcpp.hpp>
#include <can_odrive_interface/odrive_interface.hpp>
#include <std_msgs/msg/float32.hpp>

class TestVelocity : public rclcpp::Node {
public:
    TestVelocity() : Node("test_velocity") {
        odrive_ = std::make_shared<can_odrive_interface::ODriveInterface>();
        
        // Khởi tạo chế độ velocity cho Node 0
        odrive_->InitVelocityMode(1);

        // Tạo timer để gửi lệnh vận tốc mỗi 1 giây
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestVelocity::send_velocity, this));
        
        RCLCPP_INFO(this->get_logger(), "Test Velocity Node started");
    }

private:
    void send_velocity() {
        static float velocity = 0.0; // Vận tốc ban đầu: 1 turn/s
        odrive_->SendVelocityCommand(1, velocity);
        RCLCPP_INFO(this->get_logger(), "Gửi vận tốc %.2f turns/s cho Node 1", velocity);

    }

    std::shared_ptr<can_odrive_interface::ODriveInterface> odrive_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestVelocity>());
    rclcpp::shutdown();
    return 0;
}