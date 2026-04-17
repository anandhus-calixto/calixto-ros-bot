#include "calixto-ros-bot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace calixto_ros_bot
{

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Read config params
  cfg_.front_left_wheel_name  = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.rear_left_wheel_name   = info_.hardware_parameters["rear_left_wheel_name"];
  cfg_.rear_right_wheel_name  = info_.hardware_parameters["rear_right_wheel_name"];
  cfg_.loop_rate        = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device           = info_.hardware_parameters["device"];
  cfg_.baud_rate        = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms       = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Setup all 4 wheels
  wheel_fl_.setup(cfg_.front_left_wheel_name,  cfg_.enc_counts_per_rev);
  wheel_fr_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_rl_.setup(cfg_.rear_left_wheel_name,   cfg_.enc_counts_per_rev);
  wheel_rr_.setup(cfg_.rear_right_wheel_name,  cfg_.enc_counts_per_rev);

  // Validate joints — expect 1 command, 2 state interfaces each
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interfaces. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(wheel_fl_.name, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos);
  state_interfaces.emplace_back(wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.vel);

  state_interfaces.emplace_back(wheel_fr_.name, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos);
  state_interfaces.emplace_back(wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.vel);

  state_interfaces.emplace_back(wheel_rl_.name, hardware_interface::HW_IF_POSITION, &wheel_rl_.pos);
  state_interfaces.emplace_back(wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.vel);

  state_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_POSITION, &wheel_rr_.pos);
  state_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.vel);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.cmd);
  command_interfaces.emplace_back(wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.cmd);
  command_interfaces.emplace_back(wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.cmd);
  command_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  /*

  // for reading hardware
  if (!comms_.connected())
    return hardware_interface::return_type::ERROR;

  int fl_ticks, fr_ticks, rl_ticks, rr_ticks;
  comms_.read_encoder_values(fl_ticks, fr_ticks, rl_ticks, rr_ticks);

  // Store previous positions for velocity calculation
  double prev_fl = wheel_fl_.pos;
  double prev_fr = wheel_fr_.pos;
  double prev_rl = wheel_rl_.pos;
  double prev_rr = wheel_rr_.pos;

  // Convert ticks to radians
  wheel_fl_.pos = fl_ticks * wheel_fl_.rads_per_count;
  wheel_fr_.pos = fr_ticks * wheel_fr_.rads_per_count;
  wheel_rl_.pos = rl_ticks * wheel_rl_.rads_per_count;
  wheel_rr_.pos = rr_ticks * wheel_rr_.rads_per_count;

  // Velocity = delta position / delta time
  double dt = period.seconds();
  wheel_fl_.vel = (wheel_fl_.pos - prev_fl) / dt;
  wheel_fr_.vel = (wheel_fr_.pos - prev_fr) / dt;
  wheel_rl_.vel = (wheel_rl_.pos - prev_rl) / dt;
  wheel_rr_.vel = (wheel_rr_.pos - prev_rr) / dt;
  */

  // fake hardware for tench bench purpose
  // No encoders connected — dummy feedback
  wheel_fl_.vel = 0.0;
  wheel_fr_.vel = 0.0;
  wheel_rl_.vel = 0.0;
  wheel_rr_.vel = 0.0;

  // Keep positions at 0 too (no movement tracked)
  wheel_fl_.pos = 0.0;
  wheel_fr_.pos = 0.0;
  wheel_rl_.pos = 0.0;
  wheel_rr_.pos = 0.0;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
    return hardware_interface::return_type::ERROR;

      RCLCPP_INFO_THROTTLE(
    get_logger(), *clock_, 1000,
    "CMD: FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
    wheel_fl_.cmd,
    wheel_fr_.cmd,
    wheel_rl_.cmd,
    wheel_rr_.cmd
  );

  // For diff_drive_controller: fl=rl=left, fr=rr=right
  // For mecanum_drive_controller: all 4 are independent
  // Either way, just pass all 4 — firmware handles it
  comms_.set_motor_values(
    wheel_fl_.cmd,
    wheel_fr_.cmd,
    wheel_rl_.cmd,
    wheel_rr_.cmd
  );

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  calixto_ros_bot::DiffBotSystemHardware, hardware_interface::SystemInterface)
