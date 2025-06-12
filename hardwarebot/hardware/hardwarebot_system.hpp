#ifndef HARDWAREBOT_SYSTEM_HPP_
#define HARDWAREBOT_SYSTEM_HPP_
#pragma once

#include <vector>
#include <string>

#include <boost/asio.hpp>                     // ❶ full ASIO header
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hardwarebot_hardware
{

class HardwarebotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HardwarebotSystem)

  // ───────────── life-cycle ─────────────
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*prev*/) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*prev*/) override;

  // ───────────── interfaces ─────────────
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ───────────── I/O loop ───────────────
  hardware_interface::return_type read (const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // -------- parameters (with sane defaults) -----------------
  std::string port_           {"/dev/ttyS3"};
  uint32_t    baud_           {115200};
  std::string cmd_prefix_     {"~"};
  double      steps_per_rad_  {4000.0};

  double      steps_per_mm_   {200.0};        // ❷ NEW
  uint32_t    microsteps_     {16};           // ❷ NEW
  double      max_feed_mm_s_  {50.0};         // ❷ NEW
  //-----------------------------------------------------------

  // -------- joint state / command buffers -------------------
  std::vector<double> pos_, vel_, eff_, cmd_, vel_cmd_;
  //-----------------------------------------------------------

  // -------- serial objects ----------------------------------
  boost::asio::io_service  io_;
  boost::asio::serial_port serial_{io_};
  //-----------------------------------------------------------

  // helper to send one line over the serial port
  void send_line_(const std::string & line);   // ❸ not const
};

}  // namespace hardwarebot_hardware
#endif  // HARDWAREBOT_SYSTEM_HPP_

