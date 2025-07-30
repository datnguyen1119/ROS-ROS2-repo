// #ifndef ODRIVE_JOINT_HPP
// #define ODRIVE_JOINT_HPP

// #include <string>
// #include <cmath>

// class Joint {
// public:
//     std::string name;
//     int node_id;
//     double pos;
//     double vel;
//     double cmd;
//     bool enable_control;
//     std::string control_mode;
//     int enc_counts_per_rev;


//     Joint(const std::string& joint_name, int node_id, int counts_per_rev, const std::string& mode = "position") {
//         setup(joint_name, node_id, mode);
//     }

//     void setup(const std::string& joint_name, int joint_id, const std::string& mode = "position") {
//         name = joint_name;
//         node_id = joint_id;
//         control_mode = mode;
//         enable_control = true; // Mặc định bật điều khiển
//     }

//     // double calc_position(double encoder_turns) {
//     //     // Chuyển đổi từ giá trị encoder (vòng quay - turns) sang radian
//     //     pos = encoder_turns * rads_per_count;
//     //     return pos;
//     // }

//     // double calc_velocity(double encoder_vel_turns) {
//     //     // Chuyển đổi từ vận tốc encoder (vòng quay/giây - turns/s) sang radian/giây
//     //     vel = encoder_vel_turns * rads_per_count;
//     //     return vel;
//     // }

//     // double convert_command_to_turns() {
//     //     // Chuyển đổi lệnh điều khiển (radian) sang vòng quay (turns) cho ODrive
//     //     return cmd / rads_per_count;
//     // }
// };

// #endif // ODRIVE_JOINT_HPP

#ifndef ODRIVE_JOINT_HPP
#define ODRIVE_JOINT_HPP

#include <string>

namespace odrive_hardware_interface {

class OdriveJoint {
public:
    std::string name; // Tên khớp, ví dụ: "elbow_joint", "elbow_joint"
    int node_id; // ID động cơ ODrive (1 cho shoulder_joint, 2 cho elbow_joint)
    double pos; // Vị trí khớp (radian)
    double vel; // Vận tốc khớp (radian/s)
    double cmd; // Lệnh điều khiển (radian cho position mode)
    bool enable_control; // Bật/tắt điều khiển
    std::string control_mode; // Chế độ điều khiển: "position", "velocity", hoặc "torque"
    int enc_counts_per_rev; // Số xung encoder trên mỗi vòng (mặc định 8192)

    // Hàm khởi tạo thông tin khớp
    void setup(const std::string& name, int node_id, int counts_per_rev, const std::string& mode) {
        this->name = name;
        this->node_id = node_id;
        this->enc_counts_per_rev = counts_per_rev;
        this->control_mode = mode;
        this->enable_control = true;
        this->pos = 0.0;
        this->vel = 0.0;
        this->cmd = 0.0;
    }

    // Chuyển đổi vị trí từ vòng quay (turns) sang radian
    double calc_position(float pos_turns) const {
        return pos_turns * 2.0 * M_PI; // turns -> radian
    }

    // Chuyển đổi vận tốc từ vòng quay/giây (turns/s) sang radian/giây
    double calc_velocity(float vel_turns) const {
        return vel_turns * 2.0 * M_PI; // turns/s -> radian/s
    }

    // Chuyển đổi lệnh điều khiển từ radian sang vòng quay (turns)
    float convert_command_to_turns() const {
        return static_cast<float>(cmd / (2.0 * M_PI)); // radian -> turns
    }
};

}  // namespace odrive_hardware_interface

#endif  // ODRIVE_JOINT_HPP
