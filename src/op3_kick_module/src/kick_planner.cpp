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

#include "op3_kick_module/kick_planner.h"
#include <cmath>
#include <algorithm>

namespace op3_kick_module
{

KickPlanner::KickPlanner()
  : short_pass_max_distance_(1.5),
    long_pass_max_distance_(4.0),
    min_strength_(0.3),
    max_strength_(1.0),
    is_initialized_(false)
{
}

KickPlanner::~KickPlanner()
{
}

void KickPlanner::initialize(const KickConfig& config)
{
  config_ = config;
  is_initialized_ = true;
}

KickPlan KickPlanner::planKick(const BallState& ball_state,
                                const Point2D& target,
                                const Pose2D& robot_pose)
{
  KickPlan plan;
  
  // Validasi bola
  if (!isBallKickable(ball_state))
  {
    plan.is_valid = false;
    plan.reason = "Ball not in kickable position";
    return plan;
  }

  // Hitung jarak ke target
  double distance_to_target = calculateDistance(ball_state.position, target);

  // Tentukan parameter tendangan
  plan.foot = determineFoot(ball_state.angle);
  plan.type = determineKickType(distance_to_target);
  plan.strength = calculateStrength(distance_to_target, plan.type);
  plan.direction = calculateDirection(ball_state.position, target, robot_pose);
  plan.target = target;
  plan.is_valid = true;

  return plan;
}

KickPlan KickPlanner::planSimpleKick(const BallState& ball_state)
{
  KickPlan plan;

  // Validasi bola
  if (!isBallKickable(ball_state))
  {
    plan.is_valid = false;
    plan.reason = "Ball not in kickable position";
    return plan;
  }

  // Simple kick - tendang ke depan dengan kekuatan sedang
  plan.foot = determineFoot(ball_state.angle);
  plan.type = KickType::SHORT_PASS;
  plan.strength = 0.6;
  plan.direction = 0.0;  // Lurus ke depan
  plan.is_valid = true;

  return plan;
}

KickFoot KickPlanner::determineFoot(double ball_angle)
{
  // Jika bola di sebelah kanan (sudut negatif), gunakan kaki kanan
  // Jika bola di sebelah kiri (sudut positif), gunakan kaki kiri
  if (ball_angle >= 0)
  {
    return KickFoot::LEFT;
  }
  else
  {
    return KickFoot::RIGHT;
  }
}

KickType KickPlanner::determineKickType(double distance)
{
  if (distance <= short_pass_max_distance_)
  {
    return KickType::SHORT_PASS;
  }
  else if (distance <= long_pass_max_distance_)
  {
    return KickType::LONG_PASS;
  }
  else
  {
    return KickType::SHOOT;
  }
}

double KickPlanner::calculateStrength(double distance, KickType kick_type)
{
  double strength = min_strength_;

  switch (kick_type)
  {
    case KickType::SHORT_PASS:
    {
      // Linear mapping dari jarak ke kekuatan
      double ratio = distance / short_pass_max_distance_;
      strength = min_strength_ + ratio * (0.5 - min_strength_);
      break;
    }
    case KickType::LONG_PASS:
    {
      double ratio = (distance - short_pass_max_distance_) / 
                     (long_pass_max_distance_ - short_pass_max_distance_);
      strength = 0.5 + ratio * (0.8 - 0.5);
      break;
    }
    case KickType::SHOOT:
    {
      // Tendangan maksimal untuk shoot
      strength = max_strength_;
      break;
    }
    default:
      strength = 0.5;
      break;
  }

  // Clamp ke range yang valid
  return std::clamp(strength, min_strength_, max_strength_);
}

double KickPlanner::calculateDirection(const Point2D& ball_pos,
                                        const Point2D& target_pos,
                                        const Pose2D& robot_pose)
{
  // Hitung sudut dari bola ke target dalam koordinat global
  double dx = target_pos.x - ball_pos.x;
  double dy = target_pos.y - ball_pos.y;
  double global_angle = std::atan2(dy, dx);

  // Konversi ke koordinat relatif robot
  double relative_angle = normalizeAngle(global_angle - robot_pose.theta);

  return relative_angle;
}

bool KickPlanner::isBallKickable(const BallState& ball_state)
{
  if (!ball_state.is_valid)
    return false;

  // Cek jarak ke bola
  if (ball_state.distance > config_.alignment.distance_to_ball + config_.alignment.distance_tolerance)
    return false;

  if (ball_state.distance < config_.alignment.distance_to_ball - config_.alignment.distance_tolerance * 2)
    return false;

  // Cek sudut ke bola
  double angle_deg = std::abs(ball_state.angle) * 180.0 / M_PI;
  if (angle_deg > config_.alignment.angle_tolerance * 2)
    return false;

  return true;
}

AlignmentStatus KickPlanner::checkAlignment(const BallState& ball_state)
{
  if (!ball_state.is_valid)
    return AlignmentStatus::NOT_ALIGNED;

  double distance_error = std::abs(ball_state.distance - config_.alignment.distance_to_ball);
  double angle_error = std::abs(ball_state.angle) * 180.0 / M_PI;

  // Cek apakah sudah aligned
  if (distance_error <= config_.alignment.distance_tolerance &&
      angle_error <= config_.alignment.angle_tolerance)
  {
    return AlignmentStatus::ALIGNED;
  }

  return AlignmentStatus::NOT_ALIGNED;
}

void KickPlanner::calculateAlignmentAdjustment(const BallState& ball_state,
                                                double& adjustment_x,
                                                double& adjustment_y,
                                                double& adjustment_theta)
{
  adjustment_x = 0.0;
  adjustment_y = 0.0;
  adjustment_theta = 0.0;

  if (!ball_state.is_valid)
    return;

  // Hitung error jarak
  double distance_error = ball_state.distance - config_.alignment.distance_to_ball;
  
  // Hitung error posisi Y (lateral)
  double y_error = ball_state.distance * std::sin(ball_state.angle);

  // Hitung error sudut
  double angle_error = ball_state.angle;

  // Adjustment forward/backward
  // Positif berarti perlu maju, negatif berarti perlu mundur
  if (std::abs(distance_error) > config_.alignment.distance_tolerance)
  {
    adjustment_x = -distance_error * 0.5;  // Gain 0.5
    adjustment_x = std::clamp(adjustment_x, -0.03, 0.03);  // Max 3cm per step
  }

  // Adjustment lateral
  if (std::abs(y_error) > config_.alignment.y_offset_tolerance)
  {
    adjustment_y = -y_error * 0.3;  // Gain 0.3
    adjustment_y = std::clamp(adjustment_y, -0.02, 0.02);  // Max 2cm per step
  }

  // Adjustment rotasi
  if (std::abs(angle_error) > config_.alignment.angle_tolerance * M_PI / 180.0)
  {
    adjustment_theta = -angle_error * 0.2;  // Gain 0.2
    adjustment_theta = std::clamp(adjustment_theta, -0.1, 0.1);  // Max ~6 derajat per step
  }
}

void KickPlanner::updateConfig(const KickConfig& config)
{
  config_ = config;
}

double KickPlanner::calculateDistance(const Point2D& p1, const Point2D& p2)
{
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  return std::sqrt(dx * dx + dy * dy);
}

double KickPlanner::calculateAngle(const Point2D& from, const Point2D& to)
{
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  return std::atan2(dy, dx);
}

double KickPlanner::normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

}  // namespace op3_kick_module
