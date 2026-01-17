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

#include "op3_kick_module/kick_controller.h"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace op3_kick_module
{

KickController::KickController()
  : Node("kick_controller"),
    current_phase_(KickPhase::IDLE),
    alignment_elapsed_time_(0.0),
    stabilizing_elapsed_time_(0.0),
    is_initialized_(false),
    kick_requested_(false),
    alignment_complete_(false),
    is_stable_(false)
{
  kick_trajectory_ = std::make_unique<KickTrajectory>();
  kick_planner_ = std::make_unique<KickPlanner>();

  // Inisialisasi nama joint kaki
  left_leg_joints_ = {"l_hip_yaw", "l_hip_roll", "l_hip_pitch", 
                      "l_knee", "l_ank_pitch", "l_ank_roll"};
  right_leg_joints_ = {"r_hip_yaw", "r_hip_roll", "r_hip_pitch",
                       "r_knee", "r_ank_pitch", "r_ank_roll"};
}

KickController::KickController(const rclcpp::NodeOptions& options)
  : Node("kick_controller", options),
    current_phase_(KickPhase::IDLE),
    alignment_elapsed_time_(0.0),
    stabilizing_elapsed_time_(0.0),
    is_initialized_(false),
    kick_requested_(false),
    alignment_complete_(false),
    is_stable_(false)
{
  kick_trajectory_ = std::make_unique<KickTrajectory>();
  kick_planner_ = std::make_unique<KickPlanner>();

  left_leg_joints_ = {"l_hip_yaw", "l_hip_roll", "l_hip_pitch",
                      "l_knee", "l_ank_pitch", "l_ank_roll"};
  right_leg_joints_ = {"r_hip_yaw", "r_hip_roll", "r_hip_pitch",
                       "r_knee", "r_ank_pitch", "r_ank_roll"};
}

KickController::~KickController()
{
}

bool KickController::initialize(const std::string& config_path)
{
  std::string path = config_path;
  if (path.empty())
  {
    path = ament_index_cpp::get_package_share_directory("op3_kick_module") 
           + "/config/kick_config.yaml";
  }

  if (!loadConfig(path))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load config from: %s", path.c_str());
    return false;
  }

  // Initialize components
  kick_trajectory_->initialize(config_.control_cycle_sec);
  kick_planner_->initialize(config_);

  // Setup ROS2 subscribers
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/robotis/open_cr/imu", 10,
    std::bind(&KickController::imuCallback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/robotis/present_joint_states", 10,
    std::bind(&KickController::jointStateCallback, this, std::placeholders::_1));

  ball_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/robotis/ball_position", 10,
    std::bind(&KickController::ballPositionCallback, this, std::placeholders::_1));

  kick_command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robotis/kick/command", 10,
    std::bind(&KickController::kickCommandCallback, this, std::placeholders::_1));

  // Setup ROS2 publishers
  joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/robotis/kick/joint_command", 10);

  kick_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/robotis/kick/status", 10);

  kick_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/robotis/kick/done", 10);

  // Setup process timer
  auto period = std::chrono::duration<double>(config_.control_cycle_sec);
  process_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&KickController::process, this));

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Kick Controller initialized successfully");
  return true;
}

bool KickController::loadConfig(const std::string& config_path)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(config_path);

    // Load kick types
    if (doc["kick_types"])
    {
      auto kick_types_node = doc["kick_types"];
      
      if (kick_types_node["short_pass"])
      {
        auto sp = kick_types_node["short_pass"];
        config_.kick_types[KickType::SHORT_PASS].strength = sp["strength"].as<double>(0.4);
        config_.kick_types[KickType::SHORT_PASS].swing_back_angle = sp["swing_back"].as<double>(15.0);
        config_.kick_types[KickType::SHORT_PASS].swing_time = sp["swing_time"].as<double>(0.3);
      }

      if (kick_types_node["long_pass"])
      {
        auto lp = kick_types_node["long_pass"];
        config_.kick_types[KickType::LONG_PASS].strength = lp["strength"].as<double>(0.7);
        config_.kick_types[KickType::LONG_PASS].swing_back_angle = lp["swing_back"].as<double>(30.0);
        config_.kick_types[KickType::LONG_PASS].swing_time = lp["swing_time"].as<double>(0.4);
      }

      if (kick_types_node["shoot"])
      {
        auto sh = kick_types_node["shoot"];
        config_.kick_types[KickType::SHOOT].strength = sh["strength"].as<double>(1.0);
        config_.kick_types[KickType::SHOOT].swing_back_angle = sh["swing_back"].as<double>(45.0);
        config_.kick_types[KickType::SHOOT].swing_time = sh["swing_time"].as<double>(0.5);
      }
    }

    // Load alignment params
    if (doc["alignment"])
    {
      auto align = doc["alignment"];
      config_.alignment.distance_to_ball = align["distance_to_ball"].as<double>(0.18);
      config_.alignment.distance_tolerance = align["distance_tolerance"].as<double>(0.02);
      config_.alignment.angle_tolerance = align["angle_tolerance"].as<double>(8.0);
      config_.alignment.y_offset_tolerance = align["y_offset_tolerance"].as<double>(0.03);
      config_.alignment.stability_threshold = align["stability_threshold"].as<double>(0.02);
    }

    // Load balance params
    if (doc["balance"])
    {
      auto bal = doc["balance"];
      config_.balance.hip_roll_gain = bal["hip_roll_gain"].as<double>(0.35);
      config_.balance.knee_gain = bal["knee_gain"].as<double>(0.3);
      config_.balance.ankle_roll_gain = bal["ankle_roll_gain"].as<double>(0.7);
      config_.balance.ankle_pitch_gain = bal["ankle_pitch_gain"].as<double>(0.9);
    }

    // Load timing params
    if (doc["timing"])
    {
      auto tim = doc["timing"];
      config_.control_cycle_sec = tim["control_cycle_sec"].as<double>(0.008);
      config_.lift_leg_time = tim["lift_leg_time"].as<double>(0.3);
      config_.recovery_time = tim["recovery_time"].as<double>(0.5);
    }

    RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error loading config: %s", e.what());
    return false;
  }
}

void KickController::resetState()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_phase_ = KickPhase::IDLE;
  kick_requested_ = false;
  alignment_complete_ = false;
  is_stable_ = false;
  alignment_elapsed_time_ = 0.0;
  stabilizing_elapsed_time_ = 0.0;
  kick_trajectory_->reset();
}

// =============================================================================
// ROS2 Callbacks
// =============================================================================

void KickController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  
  // Convert quaternion to euler angles
  double qw = msg->orientation.w;
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;

  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  imu_state_.roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1)
    imu_state_.pitch = std::copysign(M_PI / 2, sinp);
  else
    imu_state_.pitch = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  imu_state_.yaw = std::atan2(siny_cosp, cosy_cosp);

  // Angular velocities
  imu_state_.roll_velocity = msg->angular_velocity.x;
  imu_state_.pitch_velocity = msg->angular_velocity.y;
  imu_state_.yaw_velocity = msg->angular_velocity.z;
}

void KickController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    const std::string& name = msg->name[i];
    double position = msg->position[i];

    // Left leg joints
    if (name == "l_hip_yaw") left_leg_state_.hip_yaw = position;
    else if (name == "l_hip_roll") left_leg_state_.hip_roll = position;
    else if (name == "l_hip_pitch") left_leg_state_.hip_pitch = position;
    else if (name == "l_knee") left_leg_state_.knee = position;
    else if (name == "l_ank_pitch") left_leg_state_.ankle_pitch = position;
    else if (name == "l_ank_roll") left_leg_state_.ankle_roll = position;
    // Right leg joints
    else if (name == "r_hip_yaw") right_leg_state_.hip_yaw = position;
    else if (name == "r_hip_roll") right_leg_state_.hip_roll = position;
    else if (name == "r_hip_pitch") right_leg_state_.hip_pitch = position;
    else if (name == "r_knee") right_leg_state_.knee = position;
    else if (name == "r_ank_pitch") right_leg_state_.ankle_pitch = position;
    else if (name == "r_ank_roll") right_leg_state_.ankle_roll = position;
  }
}

void KickController::ballPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  ball_state_.position.x = msg->x;
  ball_state_.position.y = msg->y;
  ball_state_.distance = std::sqrt(msg->x * msg->x + msg->y * msg->y);
  ball_state_.angle = std::atan2(msg->y, msg->x);
  ball_state_.is_valid = (msg->z > 0);  // z digunakan sebagai confidence
  ball_state_.confidence = msg->z;
}

void KickController::kickCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string command = msg->data;
  
  if (command == "kick_short")
  {
    requestKick(KickType::SHORT_PASS);
  }
  else if (command == "kick_long")
  {
    requestKick(KickType::LONG_PASS);
  }
  else if (command == "kick_shoot")
  {
    requestKick(KickType::SHOOT);
  }
  else if (command == "kick_auto")
  {
    requestAutoKick();
  }
  else if (command == "cancel")
  {
    cancelKick();
  }
  
  RCLCPP_INFO(this->get_logger(), "Received kick command: %s", command.c_str());
}

// =============================================================================
// Public Methods
// =============================================================================

bool KickController::requestKick(KickType kick_type, const Point2D& target)
{
  if (!is_initialized_)
  {
    RCLCPP_ERROR(this->get_logger(), "Kick Controller not initialized");
    return false;
  }

  if (current_phase_ != KickPhase::IDLE)
  {
    RCLCPP_WARN(this->get_logger(), "Kick already in progress");
    return false;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  // Plan the kick
  if (target.x != 0 || target.y != 0)
  {
    Pose2D robot_pose;  // Asumsi robot di origin
    current_plan_ = kick_planner_->planKick(ball_state_, target, robot_pose);
  }
  else
  {
    current_plan_ = kick_planner_->planSimpleKick(ball_state_);
  }

  // Override kick type jika di-specify
  current_plan_.type = kick_type;

  if (!current_plan_.is_valid)
  {
    RCLCPP_WARN(this->get_logger(), "Kick plan invalid: %s", current_plan_.reason.c_str());
    return false;
  }

  kick_requested_ = true;
  transitionToPhase(KickPhase::ALIGNING);
  
  RCLCPP_INFO(this->get_logger(), "Kick requested: type=%d, foot=%d", 
              static_cast<int>(kick_type), static_cast<int>(current_plan_.foot));
  
  return true;
}

bool KickController::requestAutoKick(const Point2D& target)
{
  if (!is_initialized_ || current_phase_ != KickPhase::IDLE)
    return false;

  std::lock_guard<std::mutex> lock(state_mutex_);

  if (target.x != 0 || target.y != 0)
  {
    Pose2D robot_pose;
    current_plan_ = kick_planner_->planKick(ball_state_, target, robot_pose);
  }
  else
  {
    current_plan_ = kick_planner_->planSimpleKick(ball_state_);
  }

  if (!current_plan_.is_valid)
  {
    RCLCPP_WARN(this->get_logger(), "Auto kick plan invalid: %s", current_plan_.reason.c_str());
    return false;
  }

  kick_requested_ = true;
  transitionToPhase(KickPhase::ALIGNING);
  return true;
}

void KickController::cancelKick()
{
  RCLCPP_INFO(this->get_logger(), "Kick cancelled");
  resetState();
  
  std_msgs::msg::Bool done_msg;
  done_msg.data = false;
  kick_done_pub_->publish(done_msg);
}

void KickController::updateBallState(const BallState& ball_state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ball_state_ = ball_state;
}

// =============================================================================
// Main Process Loop
// =============================================================================

void KickController::process()
{
  if (!is_initialized_)
    return;

  switch (current_phase_)
  {
    case KickPhase::IDLE:
      // Tidak ada yang perlu dilakukan
      break;

    case KickPhase::ALIGNING:
      processAlignment();
      break;

    case KickPhase::STABILIZING:
      processStabilizing();
      break;

    case KickPhase::LIFT_LEG:
    case KickPhase::SWING_BACK:
    case KickPhase::SWING_FORWARD:
    case KickPhase::FOLLOW_THROUGH:
      processKickExecution();
      break;

    case KickPhase::RECOVERY:
      processRecovery();
      break;

    case KickPhase::COMPLETED:
    {
      RCLCPP_INFO(this->get_logger(), "Kick completed successfully");
      std_msgs::msg::Bool done_msg;
      done_msg.data = true;
      kick_done_pub_->publish(done_msg);
      resetState();
      break;
    }

    case KickPhase::FAILED:
    {
      RCLCPP_ERROR(this->get_logger(), "Kick failed");
      std_msgs::msg::Bool done_msg;
      done_msg.data = false;
      kick_done_pub_->publish(done_msg);
      resetState();
      break;
    }

    default:
      break;
  }

  publishKickStatus();
}

void KickController::processAlignment()
{
  alignment_elapsed_time_ += config_.control_cycle_sec;

  // Check timeout
  if (alignment_elapsed_time_ > config_.alignment.alignment_timeout)
  {
    RCLCPP_WARN(this->get_logger(), "Alignment timeout");
    transitionToPhase(KickPhase::FAILED);
    return;
  }

  // Check alignment status
  AlignmentStatus status = kick_planner_->checkAlignment(ball_state_);

  if (status == AlignmentStatus::ALIGNED)
  {
    RCLCPP_INFO(this->get_logger(), "Alignment complete");
    alignment_complete_ = true;
    transitionToPhase(KickPhase::STABILIZING);
    return;
  }

  // Calculate alignment adjustment (untuk dikirim ke walking module)
  double adj_x, adj_y, adj_theta;
  kick_planner_->calculateAlignmentAdjustment(ball_state_, adj_x, adj_y, adj_theta);

  // TODO: Send adjustment ke walking module
  // Untuk sekarang, skip langsung ke stabilizing jika bola cukup dekat
  if (ball_state_.is_valid && ball_state_.distance < config_.alignment.distance_to_ball * 1.5)
  {
    transitionToPhase(KickPhase::STABILIZING);
  }
}

void KickController::processStabilizing()
{
  stabilizing_elapsed_time_ += config_.control_cycle_sec;

  // Check IMU stability
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    is_stable_ = imu_state_.isStable(config_.alignment.stability_threshold);
  }

  // Tunggu minimal 0.3 detik dan IMU stabil
  if (stabilizing_elapsed_time_ > 0.3 && is_stable_)
  {
    RCLCPP_INFO(this->get_logger(), "Robot stabilized, starting kick");
    
    // Simpan initial state kaki
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_plan_.foot == KickFoot::RIGHT)
      {
        kick_leg_initial_state_ = right_leg_state_;
        support_leg_initial_state_ = left_leg_state_;
      }
      else
      {
        kick_leg_initial_state_ = left_leg_state_;
        support_leg_initial_state_ = right_leg_state_;
      }
    }

    // Generate trajectory
    KickTypeParams params = config_.kick_types[current_plan_.type];
    params.strength = current_plan_.strength;
    
    if (!kick_trajectory_->generateKickTrajectory(params, kick_leg_initial_state_, current_plan_.foot))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate kick trajectory");
      transitionToPhase(KickPhase::FAILED);
      return;
    }

    kick_start_time_ = this->now();
    transitionToPhase(KickPhase::LIFT_LEG);
    return;
  }

  // Timeout untuk stabilizing
  if (stabilizing_elapsed_time_ > 2.0)
  {
    RCLCPP_WARN(this->get_logger(), "Stabilizing timeout, proceeding anyway");
    
    // Simpan initial state dan generate trajectory
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_plan_.foot == KickFoot::RIGHT)
      {
        kick_leg_initial_state_ = right_leg_state_;
        support_leg_initial_state_ = left_leg_state_;
      }
      else
      {
        kick_leg_initial_state_ = left_leg_state_;
        support_leg_initial_state_ = right_leg_state_;
      }
    }

    KickTypeParams params = config_.kick_types[current_plan_.type];
    params.strength = current_plan_.strength;
    kick_trajectory_->generateKickTrajectory(params, kick_leg_initial_state_, current_plan_.foot);
    
    kick_start_time_ = this->now();
    transitionToPhase(KickPhase::LIFT_LEG);
  }
}

void KickController::processKickExecution()
{
  // Hitung elapsed time sejak kick dimulai
  double elapsed = (this->now() - kick_start_time_).seconds();

  // Dapatkan target joint state dari trajectory
  LegJointState target_state = kick_trajectory_->getJointPosition(elapsed);
  
  // Apply balance compensation
  LegJointState compensated_state = computeBalanceCompensation(target_state);

  // Kirim command ke joint
  sendJointCommand(compensated_state, support_leg_initial_state_);

  // Update phase berdasarkan trajectory
  KickPhase trajectory_phase = kick_trajectory_->getPhase(elapsed);
  
  if (trajectory_phase != current_phase_ && trajectory_phase != KickPhase::IDLE)
  {
    transitionToPhase(trajectory_phase);
  }

  // Check if completed
  if (kick_trajectory_->isFinished(elapsed))
  {
    transitionToPhase(KickPhase::COMPLETED);
  }
}

void KickController::processRecovery()
{
  // Recovery sudah dihandle di trajectory
  // Phase akan berubah ke COMPLETED melalui processKickExecution
}

LegJointState KickController::computeBalanceCompensation(const LegJointState& target_state)
{
  LegJointState compensated = target_state;
  
  std::lock_guard<std::mutex> lock(imu_mutex_);

  // Kompensasi berdasarkan IMU roll
  double roll_compensation = -imu_state_.roll * config_.balance.ankle_roll_gain;
  compensated.ankle_roll += roll_compensation;
  compensated.hip_roll += imu_state_.roll * config_.balance.hip_roll_gain;

  // Kompensasi berdasarkan IMU pitch
  double pitch_compensation = -imu_state_.pitch * config_.balance.ankle_pitch_gain;
  compensated.ankle_pitch += pitch_compensation;
  compensated.knee += imu_state_.pitch * config_.balance.knee_gain;

  return compensated;
}

void KickController::sendJointCommand(const LegJointState& kick_leg_state,
                                       const LegJointState& support_leg_state)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();

  std::vector<std::string>& kick_joints = (current_plan_.foot == KickFoot::RIGHT) 
                                           ? right_leg_joints_ : left_leg_joints_;
  std::vector<std::string>& support_joints = (current_plan_.foot == KickFoot::RIGHT)
                                              ? left_leg_joints_ : right_leg_joints_;

  // Kick leg
  msg.name.insert(msg.name.end(), kick_joints.begin(), kick_joints.end());
  msg.position.push_back(kick_leg_state.hip_yaw);
  msg.position.push_back(kick_leg_state.hip_roll);
  msg.position.push_back(kick_leg_state.hip_pitch);
  msg.position.push_back(kick_leg_state.knee);
  msg.position.push_back(kick_leg_state.ankle_pitch);
  msg.position.push_back(kick_leg_state.ankle_roll);

  // Support leg
  msg.name.insert(msg.name.end(), support_joints.begin(), support_joints.end());
  msg.position.push_back(support_leg_state.hip_yaw);
  msg.position.push_back(support_leg_state.hip_roll);
  msg.position.push_back(support_leg_state.hip_pitch);
  msg.position.push_back(support_leg_state.knee);
  msg.position.push_back(support_leg_state.ankle_pitch);
  msg.position.push_back(support_leg_state.ankle_roll);

  joint_command_pub_->publish(msg);
}

void KickController::transitionToPhase(KickPhase new_phase)
{
  RCLCPP_INFO(this->get_logger(), "Phase transition: %d -> %d", 
              static_cast<int>(current_phase_), static_cast<int>(new_phase));
  current_phase_ = new_phase;
  phase_start_time_ = this->now();
}

void KickController::publishKickStatus()
{
  std_msgs::msg::String status_msg;
  
  switch (current_phase_)
  {
    case KickPhase::IDLE: status_msg.data = "idle"; break;
    case KickPhase::ALIGNING: status_msg.data = "aligning"; break;
    case KickPhase::STABILIZING: status_msg.data = "stabilizing"; break;
    case KickPhase::LIFT_LEG: status_msg.data = "lift_leg"; break;
    case KickPhase::SWING_BACK: status_msg.data = "swing_back"; break;
    case KickPhase::SWING_FORWARD: status_msg.data = "swing_forward"; break;
    case KickPhase::FOLLOW_THROUGH: status_msg.data = "follow_through"; break;
    case KickPhase::RECOVERY: status_msg.data = "recovery"; break;
    case KickPhase::COMPLETED: status_msg.data = "completed"; break;
    case KickPhase::FAILED: status_msg.data = "failed"; break;
    default: status_msg.data = "unknown"; break;
  }

  kick_status_pub_->publish(status_msg);
}

std::vector<std::string> KickController::getLeftLegJointNames()
{
  return left_leg_joints_;
}

std::vector<std::string> KickController::getRightLegJointNames()
{
  return right_leg_joints_;
}

}  // namespace op3_kick_module
