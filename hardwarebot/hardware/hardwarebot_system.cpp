#include "hardwarebot_system.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <sstream>

namespace hardwarebot_hardware
{

// ───────────────── helper ─────────────────
void HardwarebotSystem::send_line_(const std::string & line)
{
  boost::asio::write(serial_, boost::asio::buffer(line));
}

// ───────────────── on_init ────────────────
hardware_interface::CallbackReturn
HardwarebotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // ① read <param> tags --------------------------------------------------
  try
  {
    const auto & p = info_.hardware_parameters;     // ← correct member name
    if (p.count("port"))           port_          = p.at("port");
    if (p.count("baud"))           baud_          = std::stoul(p.at("baud"));
    if (p.count("cmd_prefix"))     cmd_prefix_    = p.at("cmd_prefix");
    if (p.count("steps_per_rad"))  steps_per_rad_ = std::stod(p.at("steps_per_rad"));
    if (p.count("steps_per_mm"))   steps_per_mm_  = std::stod(p.at("steps_per_mm"));
    if (p.count("microsteps"))     microsteps_    = std::stoul(p.at("microsteps"));
    if (p.count("max_feed_mm_s"))  max_feed_mm_s_ = std::stod(p.at("max_feed_mm_s"));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HardwarebotSystem"),
                 "Parameter error: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // ② allocate joint buffers --------------------------------------------
  const std::size_t n = info_.joints.size();
  pos_.assign(n, 0.0);   vel_.assign(n, 0.0);   eff_.assign(n, 0.0);
  cmd_.assign(n, 0.0);   vel_cmd_.assign(n, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("HardwarebotSystem"),
              "Parsed %zu joints. Opening %s @ %u baud …",
              n, port_.c_str(), baud_);

  // ③ open serial port ---------------------------------------------------
  try
  {
    using spb = boost::asio::serial_port_base;
    serial_.open(port_);
    serial_.set_option(spb::baud_rate(baud_));
    serial_.set_option(spb::character_size(8));
    serial_.set_option(spb::parity(spb::parity::none));
    serial_.set_option(spb::stop_bits(spb::stop_bits::one));
    serial_.set_option(spb::flow_control(spb::flow_control::none));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HardwarebotSystem"),
                 "Failed to open serial: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

// ───────────── export interfaces ─────────────
std::vector<hardware_interface::StateInterface>
HardwarebotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  for (std::size_t i = 0; i < pos_.size(); ++i) {
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT  , &eff_[i]);
  }
  return v;
}

std::vector<hardware_interface::CommandInterface>
HardwarebotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  for (std::size_t i = 0; i < cmd_.size(); ++i) {
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
  }
  return v;
}

// ───────────── life-cycle hooks ────────────
hardware_interface::CallbackReturn
HardwarebotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HardwarebotSystem"), "Activated");
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn
HardwarebotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HardwarebotSystem"), "Deactivated");
  return CallbackReturn::SUCCESS;
}

// ───────────────────── read ─────────────────
hardware_interface::return_type
HardwarebotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: parse feedback from Arduino
  pos_ = cmd_;                      // optimistic echo
  std::fill(vel_.begin(), vel_.end(), 0.0);
  std::fill(eff_.begin(), eff_.end(), 0.0);
  return hardware_interface::return_type::OK;
}

// ───────────────────── write ────────────────
hardware_interface::return_type
HardwarebotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < cmd_.size(); ++i)
  {
    const std::string & joint = info_.joints[i].name;
    long steps = static_cast<long>(cmd_[i] * steps_per_rad_);
    std::ostringstream line;
    line << cmd_prefix_ << "J " << joint << ' ' << steps << '\n';

    try {
      send_line_(line.str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("HardwarebotSystem"),
                   "Serial write failed: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hardwarebot_hardware

/* pluginlib hook --------------------------------------------------------*/
PLUGINLIB_EXPORT_CLASS(hardwarebot_hardware::HardwarebotSystem,
                       hardware_interface::SystemInterface)

