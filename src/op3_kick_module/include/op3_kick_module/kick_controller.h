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

#ifndef OP3_KICK_MODULE__KICK_CONTROLLER_H_
#define OP3_KICK_MODULE__KICK_CONTROLLER_H_

#include <memory>
#include <string>
#include <map>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "op3_kick_module/kick_types.h"
#include "op3_kick_module/kick_trajectory.h"
#include "op3_kick_module/kick_planner.h"

namespace op3_kick_module
{

/**
 * @brief Controller utama untuk tendangan presisi robot humanoid
 * 
 * Fitur utama:
 * - Trajectory planning dengan minimum jerk untuk gerakan smooth
 * - Multiple tipe tendangan (short pass, long pass, shoot)
 * - Balance compensation menggunakan IMU feedback
 * - Pre-kick alignment untuk presisi
 * - Configurable parameters via YAML
 */
class KickController : public rclcpp::Node
{
public:
  KickController();
  explicit KickController(const rclcpp::NodeOptions& options);
  ~KickController();

  /**
   * @brief Inisialisasi controller
   * @param config_path Path ke file konfigurasi YAML
   * @return true jika berhasil
   */
  bool initialize(const std::string& config_path = "");

  /**
   * @brief Request tendangan dengan tipe tertentu
   * @param kick_type Tipe tendangan
   * @param target Target tendangan (opsional)
   * @return true jika request diterima
   */
  bool requestKick(KickType kick_type, const Point2D& target = Point2D());

  /**
   * @brief Request tendangan otomatis (tipe ditentukan oleh planner)
   * @param target Target tendangan (opsional)
   * @return true jika request diterima
   */
  bool requestAutoKick(const Point2D& target = Point2D());

  /**
   * @brief Batalkan tendangan yang sedang berjalan
   */
  void cancelKick();

  /**
   * @brief Cek apakah tendangan sedang berjalan
   * @return true jika sedang menendang
   */
  bool isKicking() const { return current_phase_ != KickPhase::IDLE; }

  /**
   * @brief Dapatkan fase tendangan saat ini
   * @return Fase tendangan
   */
  KickPhase getCurrentPhase() const { return current_phase_; }

  /**
   * @brief Dapatkan status bola
   * @return Status bola
   */
  BallState getBallState() const { return ball_state_; }

  /**
   * @brief Update ball state dari external source
   * @param ball_state Status bola baru
   */
  void updateBallState(const BallState& ball_state);

  /**
   * @brief Main process loop - dipanggil setiap control cycle
   */
  void process();

private:
  // ==========================================================================
  // ROS2 Callbacks
  // ==========================================================================
  
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void ballPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void kickCommandCallback(const std_msgs::msg::String::SharedPtr msg);

  // ==========================================================================
  // Internal Methods
  // ==========================================================================
  
  /**
   * @brief Load konfigurasi dari file YAML
   */
  bool loadConfig(const std::string& config_path);

  /**
   * @brief Proses fase alignment
   */
  void processAlignment();

  /**
   * @brief Proses fase stabilizing
   */
  void processStabilizing();

  /**
   * @brief Proses eksekusi tendangan
   */
  void processKickExecution();

  /**
   * @brief Proses recovery setelah tendangan
   */
  void processRecovery();

  /**
   * @brief Hitung kompensasi balance berdasarkan IMU
   * @param target_state Target joint state
   * @return Compensated joint state
   */
  LegJointState computeBalanceCompensation(const LegJointState& target_state);

  /**
   * @brief Kirim command ke joint
   * @param kick_leg_state State untuk kaki tendang
   * @param support_leg_state State untuk kaki penyangga
   */
  void sendJointCommand(const LegJointState& kick_leg_state,
                        const LegJointState& support_leg_state);

  /**
   * @brief Publish status tendangan
   */
  void publishKickStatus();

  /**
   * @brief Transisi ke fase berikutnya
   * @param new_phase Fase baru
   */
  void transitionToPhase(KickPhase new_phase);

  /**
   * @brief Reset semua state
   */
  void resetState();

  /**
   * @brief Dapatkan nama joint untuk kaki kiri
   */
  std::vector<std::string> getLeftLegJointNames();

  /**
   * @brief Dapatkan nama joint untuk kaki kanan
   */
  std::vector<std::string> getRightLegJointNames();

  // ==========================================================================
  // ROS2 Publishers & Subscribers
  // ==========================================================================
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_position_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kick_command_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kick_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kick_done_pub_;

  rclcpp::TimerBase::SharedPtr process_timer_;

  // ==========================================================================
  // Components
  // ==========================================================================
  
  std::unique_ptr<KickTrajectory> kick_trajectory_;
  std::unique_ptr<KickPlanner> kick_planner_;

  // ==========================================================================
  // State Variables
  // ==========================================================================
  
  KickConfig config_;
  KickPhase current_phase_;
  KickPlan current_plan_;
  
  BallState ball_state_;
  IMUState imu_state_;
  
  LegJointState left_leg_state_;
  LegJointState right_leg_state_;
  LegJointState kick_leg_initial_state_;
  LegJointState support_leg_initial_state_;

  // Timing
  rclcpp::Time kick_start_time_;
  rclcpp::Time phase_start_time_;
  double alignment_elapsed_time_;
  double stabilizing_elapsed_time_;

  // Mutex for thread safety
  mutable std::mutex state_mutex_;
  mutable std::mutex imu_mutex_;

  // Flags
  bool is_initialized_;
  bool kick_requested_;
  bool alignment_complete_;
  bool is_stable_;

  // Joint name mappings
  std::map<std::string, int> joint_name_to_id_;
  std::vector<std::string> left_leg_joints_;
  std::vector<std::string> right_leg_joints_;
};

}  // namespace op3_kick_module

#endif  // OP3_KICK_MODULE__KICK_CONTROLLER_H_
