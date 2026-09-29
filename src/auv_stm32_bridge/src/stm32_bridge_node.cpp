// Copyright 2026 hanwen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "auv_stm32_bridge/motion_target.hpp"
#include "auv_stm32_bridge/protocol.h"
#include "auv_stm32_bridge/serial_port.hpp"
#include "auv_stm32_bridge/stream_parser.hpp"
#include "auv_stm32_bridge/telemetry_decoder.hpp"
#include "auv_interfaces/msg/depth.hpp"
#include "auv_interfaces/msg/stm32_status.hpp"
#include "auv_interfaces/srv/set_armed.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

namespace auv_stm32_bridge
{

class Stm32BridgeNode final : public rclcpp::Node
{
public:
  Stm32BridgeNode()
  : Node("stm32_bridge")
  {
    device_ = declare_parameter<std::string>("serial_device", "");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    const double reconnect_rate_hz = declare_parameter<double>("reconnect_rate_hz", 1.0);
    const double heartbeat_rate_hz = declare_parameter<double>("heartbeat_rate_hz", 20.0);
    connection_timeout_ms_ = declare_parameter<int>("connection_timeout_ms", 500);
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", "depth_link");
    const int command_timeout_ms = declare_parameter<int>("command_timeout_ms", 250);
    const double max_velocity_mps = declare_parameter<double>("max_velocity_mps", 2.0);
    const double max_depth_m = declare_parameter<double>("max_depth_m", 20.0);
    const bool arm_on_startup = declare_parameter<bool>("arm_on_startup", false);

    if (reconnect_rate_hz <= 0.0) {
      throw std::invalid_argument("reconnect_rate_hz must be greater than zero");
    }
    if (heartbeat_rate_hz <= 0.0 || connection_timeout_ms_ <= 0) {
      throw std::invalid_argument("heartbeat_rate_hz and connection_timeout_ms must be positive");
    }
    if (command_timeout_ms <= 0) {
      throw std::invalid_argument("command_timeout_ms must be positive");
    }
    motion_gate_ = std::make_unique<MotionTargetGate>(
      static_cast<uint64_t>(command_timeout_ms), static_cast<float>(max_velocity_mps),
      static_cast<float>(max_depth_m));
    if (arm_on_startup) {
      RCLCPP_ERROR(get_logger(), "arm_on_startup=true rejected: bridge always starts DISARMED");
    }

    io_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    status_publisher_ = create_publisher<auv_interfaces::msg::Stm32Status>(
      "/stm32/status", rclcpp::QoS(10));
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    depth_publisher_ = create_publisher<auv_interfaces::msg::Depth>(
      "/depth", rclcpp::SensorDataQoS());
    armed_service_ = create_service<auv_interfaces::srv::SetArmed>(
      "/stm32/set_armed",
      std::bind(
        &Stm32BridgeNode::handle_set_armed, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), service_callback_group_);
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", command_qos,
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!motion_gate_->update_velocity(steady_now_ms(), message->linear.x, message->linear.y)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "rejected invalid /cmd_vel target");
        }
      });
    depth_command_subscription_ = create_subscription<std_msgs::msg::Float32>(
      "/cmd_depth", command_qos,
      [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!motion_gate_->update_depth(steady_now_ms(), message->data)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "rejected invalid /cmd_depth target");
        }
      });
    yaw_command_subscription_ = create_subscription<std_msgs::msg::Float32>(
      "/cmd_yaw", command_qos,
      [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (!motion_gate_->update_yaw(steady_now_ms(), message->data)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "rejected invalid /cmd_yaw target");
        }
      });

    reconnect_timer_ = create_wall_timer(
      to_nanoseconds(1.0 / reconnect_rate_hz), std::bind(&Stm32BridgeNode::connect, this),
      io_callback_group_);
    io_timer_ = create_wall_timer(
      to_nanoseconds(1.0 / heartbeat_rate_hz), std::bind(&Stm32BridgeNode::io_tick, this),
      io_callback_group_);

    if (device_.empty()) {
      RCLCPP_WARN(
        get_logger(), "serial_device is empty; transport is disabled and vehicle remains DISARMED");
    } else {
      connect();
    }
    publish_status();
  }

private:
  static std::chrono::nanoseconds to_nanoseconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  static uint64_t steady_now_ms()
  {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void connect()
  {
    if (serial_port_.is_open() || device_.empty()) {
      return;
    }
    try {
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      serial_port_.open(device_, baud_rate_);
      RCLCPP_INFO(
        get_logger(), "serial transport opened: %s at %d baud", device_.c_str(),
        baud_rate_);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "serial transport unavailable: %s", error.what());
    }
  }

  void io_tick()
  {
    if (serial_port_.is_open()) {
      try {
        receive_frames();
        send_heartbeat();
        send_motion_target();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "serial I/O failed: %s", error.what());
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);
        serial_port_.close();
        connected_ = false;
        parser_.reset();
      }
    }
    publish_status();
  }

  void receive_frames()
  {
    std::array<uint8_t, 256> input{};
    std::size_t received;
    {
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      received = serial_port_.read(input.data(), input.size());
    }
    for (const auto & frame : parser_.consume(input.data(), received)) {
      last_valid_frame_ = std::chrono::steady_clock::now();
      connected_ = true;
      if (frame.message_type == AUV_PROTOCOL_MSG_STATUS) {
        decode_status(frame.payload);
      } else if (frame.message_type == AUV_PROTOCOL_MSG_ACK) {
        receive_ack(frame.payload);
      } else if (frame.message_type == AUV_PROTOCOL_MSG_IMU) {
        publish_imu(frame.payload);
      } else if (frame.message_type == AUV_PROTOCOL_MSG_DEPTH) {
        publish_depth(frame.payload);
      }
    }
    if (connected_ &&
      std::chrono::steady_clock::now() - last_valid_frame_ >
      std::chrono::milliseconds(connection_timeout_ms_))
    {
      connected_ = false;
      status_ = auv_interfaces::msg::Stm32Status();
    }
  }

  void send_heartbeat()
  {
    std::array<uint8_t, 8> payload{};
    auv_protocol_write_u32_le(payload.data(), transmit_sequence_.fetch_add(1U));
    const auto uptime = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at_).count();
    auv_protocol_write_u32_le(&payload[4], static_cast<uint32_t>(uptime));

    std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> frame{};
    const std::size_t frame_size = auv_protocol_encode_frame(
      AUV_PROTOCOL_MSG_HEARTBEAT, payload.data(), payload.size(), frame.data(), frame.size());
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    if (serial_port_.write(frame.data(), frame_size) != frame_size) {
      throw std::runtime_error("partial heartbeat write");
    }
  }

  void send_motion_target()
  {
    std::optional<MotionTarget> target;
    if (!connected_.load()) {return;}
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      target = motion_gate_->fresh_target(steady_now_ms());
    }
    if (!target.has_value()) {return;}

    const uint32_t sequence = transmit_sequence_.fetch_add(1U);
    const auto payload = encode_motion_target_payload(sequence, *target);
    std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> frame{};
    const std::size_t frame_size = auv_protocol_encode_frame(
      AUV_PROTOCOL_MSG_MOTION_TARGET, payload.data(), payload.size(), frame.data(), frame.size());
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    if (serial_port_.write(frame.data(), frame_size) != frame_size) {
      throw std::runtime_error("partial MOTION_TARGET write");
    }
  }

  void receive_ack(const std::vector<uint8_t> & payload)
  {
    if (payload.size() != 6U || payload[0] != AUV_PROTOCOL_MSG_SET_ARMED) {
      return;
    }
    const uint32_t sequence = auv_protocol_read_u32_le(&payload[2]);
    std::lock_guard<std::mutex> ack_lock(ack_mutex_);
    if (ack_waiting_ && sequence == ack_sequence_) {
      ack_result_ = payload[1];
      ack_received_ = true;
      ack_condition_.notify_one();
    }
  }

  void decode_status(const std::vector<uint8_t> & payload)
  {
    constexpr std::size_t kFixedSize = 30U;
    if (payload.size() < kFixedSize) {
      return;
    }
    const std::size_t thruster_count = payload[29];
    if (thruster_count > 8U || payload.size() != kFixedSize + thruster_count * 2U) {
      return;
    }
    const uint8_t state_flags = payload[4];
    status_.armed = (state_flags & 0x01U) != 0U;
    status_.leak_detected = (state_flags & 0x02U) != 0U;
    status_.error_flags = auv_protocol_read_u32_le(&payload[5]);
    status_.voltage = auv_protocol_read_f32_le(&payload[9]);
    status_.depth = auv_protocol_read_f32_le(&payload[13]);
    status_.roll = auv_protocol_read_f32_le(&payload[17]);
    status_.pitch = auv_protocol_read_f32_le(&payload[21]);
    status_.yaw = auv_protocol_read_f32_le(&payload[25]);
    status_.thruster_outputs.clear();
    status_.thruster_outputs.reserve(thruster_count);
    for (std::size_t index = 0; index < thruster_count; ++index) {
      const auto raw = auv_protocol_read_i16_le(&payload[kFixedSize + index * 2U]);
      status_.thruster_outputs.push_back(static_cast<float>(raw) / 1000.0F);
    }
  }

  void publish_imu(const std::vector<uint8_t> & payload)
  {
    ImuTelemetry telemetry;
    if (!decode_imu_telemetry(payload, telemetry)) {
      return;
    }

    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = imu_frame_id_;
    message.orientation.x = telemetry.orientation[0];
    message.orientation.y = telemetry.orientation[1];
    message.orientation.z = telemetry.orientation[2];
    message.orientation.w = telemetry.orientation[3];
    message.angular_velocity.x = telemetry.angular_velocity[0];
    message.angular_velocity.y = telemetry.angular_velocity[1];
    message.angular_velocity.z = telemetry.angular_velocity[2];
    message.linear_acceleration.x = telemetry.linear_acceleration[0];
    message.linear_acceleration.y = telemetry.linear_acceleration[1];
    message.linear_acceleration.z = telemetry.linear_acceleration[2];
    if (!telemetry.angular_velocity_available) {
      message.angular_velocity_covariance[0] = -1.0;
    }
    if (!telemetry.linear_acceleration_available) {
      message.linear_acceleration_covariance[0] = -1.0;
    }
    imu_publisher_->publish(message);
  }

  void publish_depth(const std::vector<uint8_t> & payload)
  {
    DepthTelemetry telemetry;
    if (!decode_depth_telemetry(payload, telemetry)) {
      return;
    }
    auv_interfaces::msg::Depth message;
    message.header.stamp = now();
    message.header.frame_id = depth_frame_id_;
    message.depth = telemetry.depth;
    message.valid = telemetry.valid;
    depth_publisher_->publish(message);
  }

  void publish_status()
  {
    status_.header.stamp = now();
    status_.connected = connected_;
    if (!serial_port_.is_open()) {
      status_.armed = false;
      status_.error_flags |= kTransportUnavailable;
    } else if (!connected_) {
      status_.armed = false;
      status_.error_flags |= kProtocolTimeout;
    }
    status_publisher_->publish(status_);
  }

  void handle_set_armed(
    const std::shared_ptr<auv_interfaces::srv::SetArmed::Request> request,
    std::shared_ptr<auv_interfaces::srv::SetArmed::Response> response)
  {
    bool transport_open;
    {
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      transport_open = serial_port_.is_open();
    }
    if (!transport_open || !connected_.load()) {
      response->accepted = false;
      response->message = "STM32 is not connected";
      return;
    }

    std::unique_lock<std::mutex> ack_lock(ack_mutex_);
    ack_sequence_ = transmit_sequence_.fetch_add(1U);
    ack_waiting_ = true;
    ack_received_ = false;
    std::array<uint8_t, 5> payload{};
    auv_protocol_write_u32_le(payload.data(), ack_sequence_);
    payload[4] = request->armed ? 1U : 0U;
    std::array<uint8_t, AUV_PROTOCOL_MAX_FRAME_SIZE> frame{};
    const std::size_t frame_size = auv_protocol_encode_frame(
      AUV_PROTOCOL_MSG_SET_ARMED, payload.data(), payload.size(), frame.data(), frame.size());
    {
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      if (serial_port_.write(frame.data(), frame_size) != frame_size) {
        ack_waiting_ = false;
        response->accepted = false;
        response->message = "failed to write complete SET_ARMED frame";
        return;
      }
    }
    const bool received = ack_condition_.wait_for(
      ack_lock, std::chrono::milliseconds(300), [this]() {return ack_received_;});
    ack_waiting_ = false;
    if (!received) {
      response->accepted = false;
      response->message = "STM32 SET_ARMED acknowledgement timed out";
      return;
    }
    response->accepted = ack_result_ == 0U;
    response->message = ack_result_message(ack_result_);
  }

  static std::string ack_result_message(const uint8_t result)
  {
    switch (result) {
      case 0U: return "STM32 accepted SET_ARMED";
      case 1U: return "STM32 rejected malformed SET_ARMED";
      case 2U: return "STM32 rejected command while disarmed";
      case 3U: return "STM32 rejected unsafe ARM request";
      case 4U: return "STM32 does not support SET_ARMED";
      default: return "STM32 returned unknown acknowledgement result";
    }
  }

  static constexpr uint32_t kTransportUnavailable = 1U << 0;
  static constexpr uint32_t kProtocolTimeout = 1U << 1;

  std::string device_;
  std::string imu_frame_id_;
  std::string depth_frame_id_;
  int baud_rate_{115200};
  int connection_timeout_ms_{500};
  std::atomic<uint32_t> transmit_sequence_{0};
  std::atomic<bool> connected_{false};
  const std::chrono::steady_clock::time_point started_at_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_valid_frame_{};
  SerialPort serial_port_;
  StreamParser parser_;
  auv_interfaces::msg::Stm32Status status_;
  rclcpp::Publisher<auv_interfaces::msg::Stm32Status>::SharedPtr status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<auv_interfaces::msg::Depth>::SharedPtr depth_publisher_;
  rclcpp::Service<auv_interfaces::srv::SetArmed>::SharedPtr armed_service_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_command_subscription_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr io_timer_;
  rclcpp::CallbackGroup::SharedPtr io_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  std::mutex serial_mutex_;
  std::mutex ack_mutex_;
  std::mutex command_mutex_;
  std::unique_ptr<MotionTargetGate> motion_gate_;
  std::condition_variable ack_condition_;
  bool ack_waiting_{false};
  bool ack_received_{false};
  uint32_t ack_sequence_{0};
  uint8_t ack_result_{0};
};

}  // namespace auv_stm32_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auv_stm32_bridge::Stm32BridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
