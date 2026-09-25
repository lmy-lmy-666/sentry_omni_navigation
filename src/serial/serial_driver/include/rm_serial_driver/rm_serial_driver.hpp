// Copyright 2024 Boombroke. Apache-2.0.
//
// rm_serial_driver — ROS 2 driver for the v3.1 single-packet serial protocol.
//   - Frame: [HEADER 1B][LEN 1B][PAYLOAD N B][CRC16 2B], total = N + 4
//   - RX (MCU→ROS): 0x50 Telemetry/200Hz (55B)
//   - TX (ROS→MCU): 0xA0 Control/20Hz + 1Hz keepalive

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"

#include "rm_serial_driver/packet.hpp"

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver() override;

private:
  // ---------- lifecycle ----------
  void getParams();
  void receiveLoop();
  void parseRxRing();
  void reopenPort();

  // ---------- TX (ROS → MCU) ----------
  void sendNavCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendHeartbeat();
  void sendControlPacket(float lx, float ly, float az, uint8_t mode);

  // ---------- RX dispatcher ----------
  void handleTelemetry(const RxTelemetryPacket & pkt);

  // ---------- IO state ----------
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::thread receive_thread_;
  std::vector<uint8_t> rx_ring_;

  // ---------- subscribers ----------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motion_state_sub_;

  // ---------- publishers ----------
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr chassis_attitude_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr chassis_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameRobotHP>::SharedPtr all_hp_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RfidStatus>::SharedPtr rfid_pub_;

  // ---------- 1Hz keepalive timer (Control packet) ----------
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // ---------- runtime state ----------
  uint8_t current_mode_{0};
  uint8_t ros_state_{2};
  float last_lx_{0.f};
  float last_ly_{0.f};
  float last_az_{0.f};

  // ---------- vel debug log ----------
  bool enable_vel_log_{false};
  std::ofstream vel_log_file_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
