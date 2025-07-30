// #include <can_odrive_interface/odrive_interface.hpp>

// namespace can_odrive_interface {

// const double REV_TO_RAD = M_PI / 4.0; // 1 turn = π/4 radian

  
// ODriveInterface::ODriveInterface() : Node("odrive_interface"), running_(true) {
//     RCLCPP_INFO(this->get_logger(), "ODrive Interface Node Started!");

//     // Tạo socket CAN
//     sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//     if (sock_ < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Không thể mở socket CAN!");
//         rclcpp::shutdown();
//         return;
//     }

//     struct ifreq ifr;
//     strcpy(ifr.ifr_name, "can0");
//     if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Không thể lấy chỉ mục CAN interface!");
//         close(sock_);
//         rclcpp::shutdown();
//         return;
//     }

//     struct sockaddr_can addr = {};
//     addr.can_family = AF_CAN;
//     addr.can_ifindex = ifr.ifr_ifindex;

//     if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Không thể kết nối CAN!");
//         close(sock_);
//         rclcpp::shutdown();
//         return;
//     }

//     // Tạo publisher cho Node 0 và Node 1
//     pos_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("/motor/position_0", 10);
//     vel_pub_0_ = this->create_publisher<std_msgs::msg::Float32>("/motor/velocity_0", 10);
//     pos_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("/motor/position_1", 10);
//     vel_pub_1_ = this->create_publisher<std_msgs::msg::Float32>("/motor/velocity_1", 10);
// }

// ODriveInterface::~ODriveInterface() {
//     running_ = false;
//     if (reader_thread_.joinable()) {
//         reader_thread_.join();
//     }
//     if (sock_ >= 0) {
//         close(sock_);
//     }
// }

// void ODriveInterface::initialize_odrive(int node_id) {
//     // Đặt trạng thái IDLE
//     if (send_can_frame(node_id, 0x07, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_IDLE for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_IDLE", node_id);
//     usleep(2000000);

//     // // // Chạy hiệu chuẩn toàn phần
//     // if (send_can_frame(node_id, 0x07, {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//     //     RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_FULL_CALIBRATION_SEQUENCE for Node %d", node_id);
//     //     return;
//     // }
//     // RCLCPP_INFO(this->get_logger(), "Node %d running AXIS_STATE_FULL_CALIBRATION_SEQUENCE", node_id);
//     // usleep(15000000);
// }

// void ODriveInterface::InitVelocityMode(int node_id) {
//     RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Velocity Control Mode", node_id);

//     initialize_odrive(node_id);

//     // Đặt chế độ điều khiển VELOCITY_CONTROL (0x02) và PASSTHROUGH (0x01)
//     if (send_can_frame(node_id, 0x0B, {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set VELOCITY_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to VELOCITY_CONTROL with PASSTHROUGH", node_id);
//     usleep(2000000);

//     // Kích hoạt CLOSED_LOOP_CONTROL
//     if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL", node_id);
//     usleep(2000000);

//     RCLCPP_INFO(this->get_logger(), "Node %d Velocity Mode initialization completed", node_id);
// }

// void ODriveInterface::InitPositionMode(int node_id) {
//     RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Position Control Mode", node_id);

//     initialize_odrive(node_id);

//     // Đặt chế độ điều khiển POSITION_CONTROL (0x03) và PASSTHROUGH (0x01)
//     struct can_frame frame;
//     frame.can_id = (node_id << 5) | 0x00B; // Set_Controller_Modes
//     frame.can_dlc = 8;
//     int32_t control_mode = 3; // Position Control
//     int32_t input_mode = 1;   // Pass-through
//     std::memcpy(frame.data, &control_mode, sizeof(int32_t));
//     std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
//     if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Set_Controller_Modes cho Node %d!", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Đặt Position Control Mode cho Node %d", node_id);
//     usleep(2000000);

//     // Kích hoạt CLOSED_LOOP_CONTROL
//     frame.can_id = (node_id << 5) | 0x007; // Set_Axis_Requested_State
//     frame.can_dlc = 4;
//     int32_t axis_state = 8; // Closed Loop Control
//     std::memcpy(frame.data, &axis_state, sizeof(int32_t));
//     std::memset(frame.data + 4, 0, 4);
//     if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Closed Loop Control cho Node %d!", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Đặt Closed Loop Control cho Node %d", node_id);
//     usleep(2000000);

//     RCLCPP_INFO(this->get_logger(), "Node %d Position Mode initialization completed", node_id);
// }

// void ODriveInterface::InitTorqueMode(int node_id) {
//     RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Torque Control Mode", node_id);

//     initialize_odrive(node_id);

//     // Đặt chế độ điều khiển TORQUE_CONTROL (0x01) và PASSTHROUGH (0x01)
//     if (send_can_frame(node_id, 0x0B, {0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set TORQUE_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to TORQUE_CONTROL with PASSTHROUGH", node_id);
//     usleep(2000000);

//     // Kích hoạt CLOSED_LOOP_CONTROL
//     if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d", node_id);
//         return;
//     }
//     RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL", node_id);
//     usleep(2000000);

//     RCLCPP_INFO(this->get_logger(), "Node %d Torque Mode initialization completed", node_id);
// }

// void ODriveInterface::SendVelocityCommand(int node_id, float velocity) {
//     struct can_frame frame;
//     frame.can_id = (node_id << 5) | 0x0D; // Set_Input_Vel
//     frame.can_dlc = 8;
//     std::memcpy(frame.data, &velocity, sizeof(float));
//     std::memset(frame.data + 4, 0, 4);
//     if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh CAN cho động cơ %d!", node_id);
//     } else {
//         RCLCPP_INFO(this->get_logger(), "Gửi vận tốc %.2f đến ODrive ID %d", velocity, node_id);
//     }
// }

// void ODriveInterface::SendPositionCommand(int node_id, float position) {
//     float position_in_turns = position; // Chuyển đổi từ rad sang turns
//     struct can_frame frame;
//     frame.can_id = (node_id << 5) | 0x0C; // Set_Input_Pos
//     frame.can_dlc = 8;
//     int16_t vel_ff = 0; // Vận tốc feedforward
//     int16_t torque_ff = 0; // Mô-men xoắn feedforward
//     std::memcpy(frame.data, &position_in_turns, sizeof(float));
//     std::memcpy(frame.data + 4, &vel_ff, sizeof(int16_t));
//     std::memcpy(frame.data + 6, &torque_ff, sizeof(int16_t));
//     if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh vị trí CAN cho động cơ %d!", node_id);
//     } else {
//         RCLCPP_INFO(this->get_logger(), "Gửi vị trí %.2f turns đến ODrive ID %d", position_in_turns, node_id);
//     }
// }

// void ODriveInterface::SendTorqueCommand(int node_id, float torque) {
//     struct can_frame frame;
//     frame.can_id = (node_id << 5) | 0x0E; // Set_Input_Torque
//     frame.can_dlc = sizeof(float);
//     std::memcpy(frame.data, &torque, sizeof(float));
//     if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame to Node %d", node_id);
//     } else {
//         RCLCPP_INFO(this->get_logger(), "Sent Torque to Node %d: %.5f Nm over CAN", node_id, torque);
//     }
// }

// void ODriveInterface::ReadEncoder() {
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(50),
//         std::bind(&ODriveInterface::request_params, this));
//     reader_thread_ = std::thread(&ODriveInterface::read_can_messages, this);
// }

// void ODriveInterface::request_params() {
//     send_can_frame(0, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
//     send_can_frame(1, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
// }

// void ODriveInterface::read_can_messages() {
//     struct can_frame frame;
//     while (running_ && rclcpp::ok()) {
//         ssize_t nbytes = read(sock_, &frame, sizeof(struct can_frame));
//         if (nbytes < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Lỗi đọc dữ liệu CAN!");
//             continue;
//         }
//         if (nbytes == sizeof(struct can_frame)) {
//             if (frame.can_id == (0 << 5 | 0x009)) {
//                 process_params(0, frame.data);
//             } else if (frame.can_id == (1 << 5 | 0x009)) {
//                 process_params(1, frame.data);
//             }
//         }
//     }
// }

// // các hàm lấy dữ liệu khác ở trên nó chỉ publish dữ liệu encoder lên topic chứ không lưu vào biến
// // hàm này sẽ lấy dữ liệu encoder từ ODrive và lưu vào biến encoder_data_
// void ODriveInterface::get_encoder_data(int node_id, float &pos, float &vel) {
//     pos = encoder_data_[node_id].position;
//     vel = encoder_data_[node_id].velocity;
// }

// void ODriveInterface::process_params(int node_id, uint8_t* data) {
//     float pos_estimate, vel_estimate;
//     std::memcpy(&pos_estimate, data, sizeof(float));
//     std::memcpy(&vel_estimate, data + 4, sizeof(float));
    
   
//     std_msgs::msg::Float32 pos_msg, vel_msg;
//     pos_msg.data = pos_estimate * REV_TO_RAD; // Chuyển đổi turns sang radian
//     vel_msg.data = vel_estimate;
//     if (node_id == 0) {
//         pos_pub_0_->publish(pos_msg); // Chuyển đổi turns sang radian
//         vel_pub_0_->publish(vel_msg);
//     } else if (node_id == 1) {
//         pos_pub_1_->publish(pos_msg);
//         vel_pub_1_->publish(vel_msg);
//     }

//     // Lưu dữ liệu vào encoder_data_
//     std::memcpy(&encoder_data_[node_id].position, data, sizeof(float));
//     std::memcpy(&encoder_data_[node_id].velocity, data + 4, sizeof(float));

//     // RCLCPP_INFO(this->get_logger(), "Node %d: Pos_Estimate = %.3f rads, Vel_Estimate = %.3f turns/s",
//     //             node_id, pos_estimate * REV_TO_RAD, vel_estimate);
// }

// int ODriveInterface::send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
//     struct can_frame frame;
//     frame.can_id = (node_id << 5) | cmd_id;
//     frame.can_dlc = 8;
//     size_t i = 0;
//     for (auto byte : data) {
//         if (i < 8) frame.data[i++] = byte;
//     }
//     while (i < 8) frame.data[i++] = 0;
//     if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
//         RCLCPP_ERROR(this->get_logger(), "Lỗi gửi frame CAN cho node %d, cmd_id 0x%02x!", node_id, cmd_id);
//         return -1;
//     }
//     RCLCPP_INFO(this->get_logger(), "Gửi frame CAN cho node %d, cmd_id 0x%02x", node_id, cmd_id);
//     return 0;
// }

// }  // namespace can_odrive_interface

#include <can_odrive_interface/odrive_interface.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <thread>

namespace can_odrive_interface {

const double REV_TO_RAD = M_PI / 4.0; // 1 turn = π/4 radian

ODriveInterface::ODriveInterface() : Node("odrive_interface"), running_(true) {
    RCLCPP_INFO(this->get_logger(), "ODrive Interface Initialized!");

    // Lấy joint_number từ parameters (nếu có)
    int joint_number = 4; // Mặc định là 4, có thể lấy từ ROS2 parameters
    declare_parameter("joint_number", 4);
    get_parameter("joint_number", joint_number);
    encoder_data_.resize(joint_number);

    // Tạo socket CAN
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Không thể mở socket CAN!");
        rclcpp::shutdown();
        return;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Không thể lấy chỉ mục CAN interface!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Không thể kết nối CAN!");
        close(sock_);
        rclcpp::shutdown();
        return;
    }

    // Khởi tạo reader thread để đọc tin nhắn CAN
    reader_thread_ = std::thread(&ODriveInterface::read_can_messages, this);
}

ODriveInterface::~ODriveInterface() {
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    if (sock_ >= 0) {
        close(sock_);
    }
}

void ODriveInterface::initialize_odrive(int node_id) {
    // Đặt trạng thái IDLE
    if (send_can_frame(node_id, 0x07, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_IDLE for Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_IDLE", node_id);
    usleep(2000000);
}

void ODriveInterface::InitVelocityMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Velocity Control Mode", node_id);

    initialize_odrive(node_id);

    // Đặt chế độ điều khiển VELOCITY_CONTROL (0x02) và PASSTHROUGH (0x01)
    if (send_can_frame(node_id, 0x0B, {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set VELOCITY_CONTROL for Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d set to VELOCITY_CONTROL with PASSTHROUGH", node_id);
    usleep(2000000);

    // Kích hoạt CLOSED_LOOP_CONTROL
    if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL", node_id);
    usleep(2000000);

    RCLCPP_INFO(this->get_logger(), "Node %d Velocity Mode initialization completed", node_id);
}

void ODriveInterface::InitPositionMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Position Control Mode", node_id);

    initialize_odrive(node_id);

    // Đặt chế độ điều khiển POSITION_CONTROL (0x03) và PASSTHROUGH (0x01)
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x00B; // Set_Controller_Modes
    frame.can_dlc = 8;
    int32_t control_mode = 3; // Position Control
    int32_t input_mode = 1;   // Pass-through
    std::memcpy(frame.data, &control_mode, sizeof(int32_t));
    std::memcpy(frame.data + 4, &input_mode, sizeof(int32_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Set_Controller_Modes cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Position Control Mode cho Node %d", node_id);
    usleep(2000000);

    // Kích hoạt CLOSED_LOOP_CONTROL
    frame.can_id = (node_id << 5) | 0x007; // Set_Axis_Requested_State
    frame.can_dlc = 4;
    int32_t axis_state = 8; // Closed Loop Control
    std::memcpy(frame.data, &axis_state, sizeof(int32_t));
    std::memset(frame.data + 4, 0, 4);
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh Closed Loop Control cho Node %d!", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Đặt Closed Loop Control cho Node %d", node_id);
    usleep(2000000);

    RCLCPP_INFO(this->get_logger(), "Node %d Position Mode initialization completed", node_id);
}

void ODriveInterface::InitTorqueMode(int node_id) {
    RCLCPP_INFO(this->get_logger(), "Initializing Node %d with Torque Control Mode", node_id);

    initialize_odrive(node_id);

    // Đặt chế độ điều khiển TORQUE_CONTROL (0x01) và PASSTHROUGH (0x01)
    if (send_can_frame(node_id, 0x0B, {0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set TORQUE_CONTROL for Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d set to TORQUE_CONTROL with PASSTHROUGH", node_id);
    usleep(2000000);

    // Kích hoạt CLOSED_LOOP_CONTROL
    if (send_can_frame(node_id, 0x07, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set AXIS_STATE_CLOSED_LOOP_CONTROL for Node %d", node_id);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Node %d set to AXIS_STATE_CLOSED_LOOP_CONTROL", node_id);
    usleep(2000000);

    RCLCPP_INFO(this->get_logger(), "Node %d Torque Mode initialization completed", node_id);
}

void ODriveInterface::SendVelocityCommand(int node_id, float velocity) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0D; // Set_Input_Vel
    frame.can_dlc = 8;
    std::memcpy(frame.data, &velocity, sizeof(float));
    std::memset(frame.data + 4, 0, 4);
    if ( write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh CAN cho động cơ %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Gửi vận tốc %.2f đến ODrive ID %d", velocity, node_id);
    }
}

void ODriveInterface::SendPositionCommand(int node_id, float position) {
    float position_in_turns = position; // Giả sử position đã ở đơn vị turns
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0C; // Set_Input_Pos
    frame.can_dlc = 8;
    int16_t vel_ff = 0; // Vận tốc feedforward
    int16_t torque_ff = 0; // Mô-men xoắn feedforward
    std::memcpy(frame.data, &position_in_turns, sizeof(float));
    std::memcpy(frame.data + 4, &vel_ff, sizeof(int16_t));
    std::memcpy(frame.data + 6, &torque_ff, sizeof(int16_t));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi lệnh vị trí CAN cho động cơ %d!", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Gửi vị trí %.2f turns đến ODrive ID %d", position_in_turns, node_id);
    }
}

void ODriveInterface::SendTorqueCommand(int node_id, float torque) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | 0x0E; // Set_Input_Torque
    frame.can_dlc = sizeof(float);
    std::memcpy(frame.data, &torque, sizeof(float));
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error sending CAN frame to Node %d", node_id);
    } else {
        RCLCPP_INFO(this->get_logger(), "Sent Torque to Node %d: %.5f Nm over CAN", node_id, torque);
    }
}

// void ODriveInterface::ReadEncoder() {
//     // Gửi yêu cầu dữ liệu encoder cho cả hai node
//     send_can_frame(0, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
//     send_can_frame(1, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
// }

void ODriveInterface::ReadEncoder() {
    for (size_t i = 0; i < encoder_data_.size(); ++i) {
        send_can_frame(i, 0x009, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    }
}

void ODriveInterface::read_can_messages() {
    struct can_frame frame;
    while (running_ && rclcpp::ok()) {
        ssize_t nbytes = read(sock_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi đọc dữ liệu CAN!");
            continue;
        }
        if (nbytes == sizeof(struct can_frame)) {
            int node_id = frame.can_id >> 5;
            if ((frame.can_id & 0x1F) == 0x009 && static_cast<size_t>(node_id) < encoder_data_.size()) {
                process_params(node_id, frame.data);
            }
        }
    }
}

// void ODriveInterface::get_encoder_data(int node_id, float &pos, float &vel) {
//     if (node_id >= 0 && node_id < 2) {
//         pos = encoder_data_[node_id].position;
//         vel = encoder_data_[node_id].velocity;
//     } else {
//         pos = 0.0;
//         vel = 0.0;
//         RCLCPP_ERROR(this->get_logger(), "Invalid node_id: %d", node_id);
//     }
// }

void ODriveInterface::get_encoder_data(int node_id, float &pos, float &vel) {
    if (node_id >= 0 && static_cast<size_t>(node_id) < encoder_data_.size()) {
        pos = encoder_data_[node_id].position;
        vel = encoder_data_[node_id].velocity;
    } else {
        pos = 0.0;
        vel = 0.0;
        RCLCPP_ERROR(this->get_logger(), "Invalid node_id: %d", node_id);
    }
}

void ODriveInterface::process_params(int node_id, uint8_t* data) {
    float pos_estimate, vel_estimate;
    std::memcpy(&pos_estimate, data, sizeof(float));
    std::memcpy(&vel_estimate, data + 4, sizeof(float));

    // Lưu dữ liệu vào encoder_data_
    encoder_data_[node_id].position = pos_estimate;
    encoder_data_[node_id].velocity = vel_estimate;

    // RCLCPP_INFO(this->get_logger(), "Node %d: Pos_Estimate = %.3f turns, Vel_Estimate = %.3f turns/s",
    //             node_id, pos_estimate, vel_estimate);
}

int ODriveInterface::send_can_frame(int node_id, uint8_t cmd_id, std::initializer_list<uint8_t> data) {
    struct can_frame frame;
    frame.can_id = (node_id << 5) | cmd_id;
    frame.can_dlc = 8;
    size_t i = 0;
    for (auto byte : data) {
        if (i < 8) frame.data[i++] = byte;
    }
    while (i < 8) frame.data[i++] = 0;
    if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(this->get_logger(), "Lỗi gửi frame CAN cho node %d, cmd_id 0x%02x!", node_id, cmd_id);
        return -1;
    }
    RCLCPP_INFO(this->get_logger(), "Gửi frame CAN cho node %d, cmd_id 0x%02x", node_id, cmd_id);
    return 0;
}

}  // namespace can_odrive_interface