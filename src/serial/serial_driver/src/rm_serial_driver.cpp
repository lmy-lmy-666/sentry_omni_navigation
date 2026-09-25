// Copyright 2024 Boombroke. Apache-2.0.
//
// v3.1 single-packet implementation.
// See rm_serial_driver.hpp for protocol overview.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
#include "rm_serial_driver/write_all.hpp"

namespace rm_serial_driver
{

namespace
{
constexpr size_t kRxChunkSize = 256;
constexpr size_t kRxRingMaxSize = 2048;
}  // namespace

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver (protocol v3.1)");
  getParams();

  this->declare_parameter<bool>("enable_vel_log", false);
  this->get_parameter("enable_vel_log", enable_vel_log_);
  if (enable_vel_log_) {
    std::string log_path =
      "/tmp/vel_log_" + std::to_string(this->now().nanoseconds()) + ".csv";
    vel_log_file_.open(log_path);
    vel_log_file_ << "timestamp_ns,lx,ly,az,mode\n";
    RCLCPP_INFO(get_logger(), "Velocity logging enabled: %s", log_path.c_str());
  }

  nav_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_chassis", 10,
    std::bind(&RMSerialDriver::sendNavCmd, this, std::placeholders::_1));

  motion_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "motion_manager/state", 10,
    [this](const std_msgs::msg::UInt8::SharedPtr msg) { current_mode_ = msg->data; });

  gimbal_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", 10);
  chassis_attitude_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "serial/chassis_attitude", 10);
  robot_status_pub_ = this->create_publisher<rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", 10);
  chassis_status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "serial/chassis_status", 10);
  game_status_pub_ = this->create_publisher<rm_interfaces::msg::GameStatus>(
    "referee/game_status", 10);
  all_hp_pub_ = this->create_publisher<rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", 10);
  rfid_pub_ = this->create_publisher<rm_interfaces::msg::RfidStatus>(
    "referee/rfidStatus", 10);

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&RMSerialDriver::sendHeartbeat, this));

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      RCLCPP_INFO(
        get_logger(), "Opened %s; TX logs indicate local writes, not MCU ACKs",
        device_name_.c_str());
      receive_thread_ = std::thread(&RMSerialDriver::receiveLoop, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
}

RMSerialDriver::~RMSerialDriver()
{
  if (vel_log_file_.is_open()) {
    vel_log_file_.close();
  }
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (serial_driver_->port() && serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveLoop()
{
  std::vector<uint8_t> chunk(kRxChunkSize);

  while (rclcpp::ok()) {
    try {
      const size_t n = serial_driver_->port()->receive(chunk);
      if (n == 0) {
        continue;
      }
      if (rx_ring_.size() + n > kRxRingMaxSize) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "RX ring overflow (%zu bytes), resetting", rx_ring_.size());
        rx_ring_.clear();
      }
      rx_ring_.insert(rx_ring_.end(), chunk.begin(), chunk.begin() + n);
      parseRxRing();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Receive error: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::parseRxRing()
{
  while (rx_ring_.size() >= 3) {
    const uint8_t hdr = rx_ring_[0];

    if (isTxHeader(hdr)) {
      const size_t frame_len = frameSizeForHeader(hdr);
      if (frame_len == 0 || rx_ring_.size() < frame_len) {
        break;
      }
      rx_ring_.erase(rx_ring_.begin(), rx_ring_.begin() + static_cast<std::ptrdiff_t>(frame_len));
      RCLCPP_DEBUG(
        get_logger(), "Skipped TX echo 0x%02X (%zuB)", hdr, frame_len);
      continue;
    }

    if (!isRxHeader(hdr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Unknown header 0x%02X, dropping byte", hdr);
      rx_ring_.erase(rx_ring_.begin());
      continue;
    }

    const size_t frame_len = frameSizeForHeader(hdr);
    if (frame_len == 0 || rx_ring_.size() < frame_len) {
      break;
    }

    const uint8_t plen = rx_ring_[1];
    if (plen != payloadSizeForHeader(hdr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LEN mismatch on 0x%02X: got %u want %zu", hdr, static_cast<unsigned>(plen),
        payloadSizeForHeader(hdr));
      rx_ring_.erase(rx_ring_.begin());
      continue;
    }

    std::vector<uint8_t> full(rx_ring_.begin(), rx_ring_.begin() + static_cast<std::ptrdiff_t>(frame_len));
    rx_ring_.erase(rx_ring_.begin(), rx_ring_.begin() + static_cast<std::ptrdiff_t>(frame_len));

    if (!crc16::Verify_CRC16_Check_Sum(full.data(), full.size())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "CRC failed on header 0x%02X (%zuB)", hdr, full.size());
      continue;
    }

    if (hdr == HDR_RX_TELEMETRY) {
      handleTelemetry(fromVector<RxTelemetryPacket>(full));
    }
  }
}

void RMSerialDriver::handleTelemetry(const RxTelemetryPacket & pkt)
{
  const auto stamp = now();

  sensor_msgs::msg::JointState gimbal;
  gimbal.header.stamp = stamp;
  gimbal.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  gimbal.position = {pkt.gimbal_pitch, pkt.gimbal_yaw};
  gimbal_joint_pub_->publish(gimbal);

  sensor_msgs::msg::JointState chassis;
  chassis.header.stamp = stamp;
  chassis.name = {"chassis_pitch", "chassis_yaw"};
  chassis.position = {pkt.chassis_pitch, pkt.chassis_yaw};
  chassis_attitude_pub_->publish(chassis);

  rm_interfaces::msg::RobotStatus status;
  status.current_hp = pkt.current_hp;
  status.projectile_allowance_17mm = pkt.projectile_allowance_17mm;
  robot_status_pub_->publish(status);

  std_msgs::msg::Float32MultiArray chassis_extra;
  chassis_extra.data = {pkt.chassis_power, static_cast<float>(pkt.chassis_mode)};
  chassis_status_pub_->publish(chassis_extra);

  rm_interfaces::msg::GameStatus game;
  game.game_progress = pkt.game_progress;
  game.stage_remain_time = pkt.stage_remain_time;
  game_status_pub_->publish(game);

  rm_interfaces::msg::GameRobotHP hp;
  hp.ally_1_robot_hp = pkt.ally_1_robot_hp;
  hp.ally_2_robot_hp = pkt.ally_2_robot_hp;
  hp.ally_3_robot_hp = pkt.ally_3_robot_hp;
  hp.ally_4_robot_hp = pkt.ally_4_robot_hp;
  hp.ally_7_robot_hp = pkt.ally_7_robot_hp;
  hp.ally_outpost_hp = pkt.ally_outpost_hp;
  hp.ally_base_hp = pkt.ally_base_hp;
  hp.event_data = pkt.event_data;
  all_hp_pub_->publish(hp);

  rm_interfaces::msg::RfidStatus rfid;
  rfid.friendly_supply_zone_non_exchange = pkt.rfid_base;
  rfid_pub_->publish(rfid);
}

void RMSerialDriver::sendNavCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_lx_ = static_cast<float>(msg->linear.x);
  last_ly_ = static_cast<float>(msg->linear.y);
  last_az_ = static_cast<float>(msg->angular.z);
  sendControlPacket(last_lx_, last_ly_, last_az_, current_mode_);
}

void RMSerialDriver::sendHeartbeat()
{
  if (!serial_driver_->port() || !serial_driver_->port()->is_open()) {
    return;
  }
  sendControlPacket(last_lx_, last_ly_, last_az_, current_mode_);
}

void RMSerialDriver::sendControlPacket(float lx, float ly, float az, uint8_t mode)
{
  try {
    TxControlPacket pkt;
    pkt.lx        = lx;
    pkt.ly        = ly;
    pkt.az        = az;
    pkt.mode      = mode;
    pkt.reserved  = 0;
    pkt.ros_state = ros_state_;
    pkt.reserved2 = 0;

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));

    const auto data = toVector(pkt);
    writeAll(data, [this](const std::vector<uint8_t> & bytes) {
      return serial_driver_->port()->send(bytes);
    });

    if (enable_vel_log_ && vel_log_file_.is_open()) {
      vel_log_file_ << this->now().nanoseconds() << ',' << pkt.lx << ',' << pkt.ly << ','
                    << pkt.az << ',' << static_cast<int>(pkt.mode) << '\n';
    }

    RCLCPP_DEBUG(
      get_logger(), "TX locally written to %s: lx=%.3f ly=%.3f az=%.3f mode=%u (%zuB)",
      device_name_.c_str(), pkt.lx, pkt.ly, pkt.az,
      static_cast<unsigned>(pkt.mode), data.size());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Error while sending Control: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");
    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");
    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  rx_ring_.clear();
  while (rclcpp::ok()) {
    try {
      if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
      return;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
