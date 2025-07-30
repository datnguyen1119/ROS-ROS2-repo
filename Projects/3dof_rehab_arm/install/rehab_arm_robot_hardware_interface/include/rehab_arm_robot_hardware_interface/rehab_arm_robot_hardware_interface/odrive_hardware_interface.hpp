#ifndef ODRIVE_HARDWARE_INTERFACE_HPP
#define ODRIVE_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "rclcpp_lifecycle/state.hpp"

#include <can_odrive_interface/odrive_interface.hpp>
#include "rehab_arm_robot_hardware_interface/odrive_joint.hpp"

namespace odrive_hardware_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ODriveSystemHardware : public hardware_interface::SystemInterface {

struct Config {
    std::vector<std::string> joint_names; // Danh sách tên khớp (shoulder_joint, elbow_joint)
    std::vector<int> joint_ids; // Danh sách node_id (1, 2)
    std::string active_joints; // Chuỗi joint được điều khiển (shoulder_joint,elbow_joint)
    std::string can_bus_name = "";
    std::string can_bus_baudrate = "";
    std::string control_mode = ""; // "position" trong trường hợp này
  };

public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  
  // std::vector<double> position_states_;
  // std::vector<double> velocity_states_;
  // std::vector<double> position_commands_;

  std::shared_ptr<can_odrive_interface::ODriveInterface> odrive_interface_;
  Config cfg_;
  std::vector<OdriveJoint> joints_;
  size_t joint_number_;
  hardware_interface::HardwareInfo info_;


};

}  // namespace odrive_hardware_interface

#endif  // ODRIVE_HARDWARE_INTERFACE_HPP
