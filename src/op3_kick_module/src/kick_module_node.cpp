/*******************************************************************************
 * Copyright 2026 AROC26 Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/**
 * @file kick_module_node.cpp
 * @brief Main node untuk op3_kick_module
 * 
 * Node ini menjalankan KickController untuk mengontrol tendangan presisi
 * pada robot humanoid ROBOTIS OP3.
 * 
 * Topics:
 *   Subscribed:
 *     - /robotis/open_cr/imu (sensor_msgs/Imu): Data IMU untuk balance
 *     - /robotis/present_joint_states (sensor_msgs/JointState): State joint saat ini
 *     - /robotis/ball_position (geometry_msgs/Point): Posisi bola
 *     - /robotis/kick/command (std_msgs/String): Command tendangan
 * 
 *   Published:
 *     - /robotis/kick/joint_command (sensor_msgs/JointState): Command joint
 *     - /robotis/kick/status (std_msgs/String): Status tendangan
 *     - /robotis/kick/done (std_msgs/Bool): Notifikasi tendangan selesai
 * 
 * Commands yang diterima:
 *     - "kick_short": Tendangan pendek
 *     - "kick_long": Tendangan jauh
 *     - "kick_shoot": Tendangan keras ke gawang
 *     - "kick_auto": Tendangan otomatis (tipe ditentukan planner)
 *     - "cancel": Batalkan tendangan
 */

#include <rclcpp/rclcpp.hpp>
#include "op3_kick_module/kick_controller.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto kick_controller = std::make_shared<op3_kick_module::KickController>();

  // Initialize dengan config path default
  std::string config_path = "";
  
  // Cek apakah ada argument untuk config path
  if (argc > 1)
  {
    config_path = argv[1];
  }

  if (!kick_controller->initialize(config_path))
  {
    RCLCPP_ERROR(kick_controller->get_logger(), "Failed to initialize Kick Controller");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(kick_controller->get_logger(), "=================================");
  RCLCPP_INFO(kick_controller->get_logger(), "  OP3 Kick Module Started");
  RCLCPP_INFO(kick_controller->get_logger(), "=================================");
  RCLCPP_INFO(kick_controller->get_logger(), "Commands:");
  RCLCPP_INFO(kick_controller->get_logger(), "  - kick_short : Short pass");
  RCLCPP_INFO(kick_controller->get_logger(), "  - kick_long  : Long pass");
  RCLCPP_INFO(kick_controller->get_logger(), "  - kick_shoot : Shoot");
  RCLCPP_INFO(kick_controller->get_logger(), "  - kick_auto  : Auto kick");
  RCLCPP_INFO(kick_controller->get_logger(), "  - cancel     : Cancel kick");
  RCLCPP_INFO(kick_controller->get_logger(), "=================================");

  rclcpp::spin(kick_controller);
  rclcpp::shutdown();
  
  return 0;
}
