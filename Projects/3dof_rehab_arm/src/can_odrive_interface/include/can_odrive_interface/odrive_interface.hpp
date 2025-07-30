#ifndef CAN_ODRIVE_INTERFACE_HPP
#define CAN_ODRIVE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <std_msgs/msg/float32.hpp>

namespace can_odrive_interface {

class ODriveInterface : public rclcpp::Node {
public:
    static constexpr double RAD_TO_REV = 4.0 / M_PI; 
    ODriveInterface();
    ~ODriveInterface();

    void InitVelocityMode(int node_id);
    void InitPositionMode(int node_id);
    void InitTorqueMode(int node_id);
    void SendVelocityCommand(int node_id, float velocity);
    void SendPositionCommand(int node_id, float position);
    void SendTorqueCommand(int node_id, float torque);
    void ReadEncoder();
    void get_encoder_data(int node_id, float &pos, float &vel);

private:
    struct EncoderData {
        float position = 0.0;
        float velocity = 0.0;
    };
    // EncoderData encoder_data_[2];
    std::vector<EncoderData> encoder_data_; // Thay bằng vector
    void process_params(int node_id, uint8_t* data);
    void initialize_odrive(int node_id);
    // void request_params();
    void read_can_messages();
    // void process_params(int node_id, uint8_t* data);
    int send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data);

    int sock_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread reader_thread_;
    bool running_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_0_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub_0_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_1_;
    // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub_1_;
};

}  // namespace can_odrive_interface

#endif  // CAN_ODRIVE_INTERFACE_HPP


