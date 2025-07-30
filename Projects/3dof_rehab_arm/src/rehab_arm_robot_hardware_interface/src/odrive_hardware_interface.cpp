#include "rehab_arm_robot_hardware_interface/odrive_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <can_odrive_interface/odrive_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <vector>
#include <string>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface {

hardware_interface::CallbackReturn ODriveSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
  info_ = info;

  // Lấy active_joints từ hardware_parameters
  if (info_.hardware_parameters.find("active_joints") == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Parameter 'active_joints' not found in hardware parameters!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  cfg_.active_joints = info_.hardware_parameters.at("active_joints");
  std::vector<std::string> active_joints;
  std::stringstream ss(cfg_.active_joints);
  std::string joint_name;
  while (std::getline(ss, joint_name, ',')) {
    active_joints.push_back(joint_name);
  }
  joint_number_ = active_joints.size(); // joint_number_ = 2 (shoulder_joint, elbow_joint)

  // Lấy joint_number từ hardware_parameters (kiểm tra tính nhất quán)
  if (info_.hardware_parameters.find("joint_number") == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Parameter 'joint_number' not found in hardware parameters!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  size_t joint_number_param = std::stoi(info_.hardware_parameters.at("joint_number"));
  if (joint_number_param != joint_number_) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Mismatch between joint_number (%zu) and active_joints size (%zu)!",
      joint_number_param, joint_number_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Lấy joint_names và joint_ids
  cfg_.joint_names.resize(joint_number_);
  cfg_.joint_ids.resize(joint_number_);
  for (size_t i = 0; i < joint_number_; ++i) {
    std::string param_name = "joint" + std::to_string(i + 1) + "_name"; // joint1_name, joint2_name
    std::string id_param_name = "joint" + std::to_string(i + 1) + "_id"; // joint1_id, joint2_id
    if (info_.hardware_parameters.find(param_name) == info_.hardware_parameters.end() ||
        info_.hardware_parameters.find(id_param_name) == info_.hardware_parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ODriveSystemHardware"),
        "Missing joint name or ID for joint %zu!", i);
      return hardware_interface::CallbackReturn::ERROR;
    }
    cfg_.joint_names[i] = info_.hardware_parameters.at(param_name); // shoulder_joint, elbow_joint
    cfg_.joint_ids[i] = std::stoi(info_.hardware_parameters.at(id_param_name)); // 1, 2
  }

  // Lấy các tham số khác
  if (info_.hardware_parameters.find("can_bus_name") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("can_bus_baudrate") == info_.hardware_parameters.end() ||
      info_.hardware_parameters.find("control_mode") == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Missing can_bus_name, can_bus_baudrate, or control_mode in hardware parameters!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  cfg_.can_bus_name = info_.hardware_parameters.at("can_bus_name"); // can0
  cfg_.can_bus_baudrate = info_.hardware_parameters.at("can_bus_baudrate"); // 500000
  cfg_.control_mode = info_.hardware_parameters.at("control_mode"); // position

  // Kiểm tra cấu hình joint
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Chỉ kiểm tra các khớp trong active_joints
    bool is_active = false;
    for (const auto & active_joint : active_joints) {
      if (joint.name == active_joint) {
        is_active = true;
        break;
      }
    }
    if (!is_active) {
      continue; // Bỏ qua base_joint, wrist_joint
    }

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ODriveSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ODriveSystemHardware"),
        "Joint '%s' has '%s' command interface found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ODriveSystemHardware"),
        "Joint '%s' has %zu state interfaces. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ODriveSystemHardware"),
        "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ODriveSystemHardware"),
        "Joint '%s' has '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Khởi tạo odrive_interface_ và joints_
  odrive_interface_ = std::make_shared<can_odrive_interface::ODriveInterface>();
  joints_.resize(joint_number_);
  int enc_counts_per_rev = 8192; // Giá trị mặc định của ODrive
  for (size_t i = 0; i < joint_number_; ++i) {
    joints_[i].setup(cfg_.joint_names[i], cfg_.joint_ids[i], enc_counts_per_rev, cfg_.control_mode);
    odrive_interface_->InitPositionMode(cfg_.joint_ids[i]);
    RCLCPP_INFO(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Initialized joint %s with node_id %d", joints_[i].name.c_str(), joints_[i].node_id);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ODriveSystemHardware"), "Configuring hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ODriveSystemHardware"), "Cleaning up hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto & joint : joints_) {
    joint.enable_control = true;
    RCLCPP_INFO(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Activated control for joint %s", joint.name.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto & joint : joints_) {
    joint.enable_control = false;
    RCLCPP_INFO(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Deactivated control for joint %s", joint.name.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < joint_number_; ++i) {
    interfaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].pos);
    interfaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].vel);
    RCLCPP_INFO(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Exported state interfaces for joint %s", joints_[i].name.c_str());
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joint_number_; ++i) {
    interfaces.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].cmd);
    RCLCPP_INFO(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Exported command interface for joint %s", joints_[i].name.c_str());
  }
  return interfaces;
}

hardware_interface::return_type ODriveSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  odrive_interface_->ReadEncoder();
  for (size_t i = 0; i < joint_number_; ++i) {
    float pos_turns, vel_turns;
    odrive_interface_->get_encoder_data(joints_[i].node_id, pos_turns, vel_turns);
    joints_[i].pos = joints_[i].calc_position(pos_turns);
    joints_[i].vel = joints_[i].calc_velocity(vel_turns);
    RCLCPP_DEBUG(
      rclcpp::get_logger("ODriveSystemHardware"),
      "Read joint %s: pos=%.3f rad, vel=%.3f rad/s", joints_[i].name.c_str(), joints_[i].pos, joints_[i].vel);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  for (size_t i = 0; i < joint_number_; ++i) {
    if (joints_[i].enable_control && joints_[i].control_mode == "position") {
      float cmd_turns = joints_[i].convert_command_to_turns();
      odrive_interface_->SendPositionCommand(joints_[i].node_id, cmd_turns);
      RCLCPP_DEBUG(
        rclcpp::get_logger("ODriveSystemHardware"),
        "Wrote command to joint %s: cmd=%.3f rad (%.3f turns)", joints_[i].name.c_str(), joints_[i].cmd, cmd_turns);
    }
  }
  return hardware_interface::return_type::OK;
}

//phải thêm cái này thì mới có thể sử dụng pluginlib
PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveSystemHardware,
  hardware_interface::SystemInterface)

}  // namespace odrive_hardware_interface