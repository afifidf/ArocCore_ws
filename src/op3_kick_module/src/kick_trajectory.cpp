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

#include "op3_kick_module/kick_trajectory.h"
#include <iostream>
#include <cmath>

namespace op3_kick_module
{

KickTrajectory::KickTrajectory()
  : control_cycle_sec_(0.008),
    total_duration_(0.0),
    kick_foot_(KickFoot::RIGHT),
    is_initialized_(false),
    trajectory_generated_(false)
{
}

KickTrajectory::~KickTrajectory()
{
}

void KickTrajectory::initialize(double control_cycle_sec)
{
  control_cycle_sec_ = control_cycle_sec;
  is_initialized_ = true;
}

bool KickTrajectory::generateKickTrajectory(const KickTypeParams& kick_params,
                                             const LegJointState& initial_joint_state,
                                             KickFoot foot)
{
  if (!is_initialized_)
  {
    std::cerr << "[KickTrajectory] Not initialized!" << std::endl;
    return false;
  }

  // Clear previous trajectory
  reset();

  kick_foot_ = foot;
  initial_state_ = initial_joint_state;

  // =========================================================================
  // Timing Configuration
  // =========================================================================
  
  double lift_time = 0.25;                              // Waktu angkat kaki
  double swing_back_time = kick_params.swing_time * 0.4;  // 40% untuk swing back
  double swing_forward_time = kick_params.swing_time * 0.4; // 40% untuk swing forward
  double follow_through_time = kick_params.swing_time * 0.2; // 20% untuk follow through
  double recovery_time = 0.4;                           // Waktu recovery

  // Set timing untuk setiap fase
  lift_leg_start_ = 0.0;
  lift_leg_end_ = lift_time;

  swing_back_start_ = lift_leg_end_;
  swing_back_end_ = swing_back_start_ + swing_back_time;

  swing_forward_start_ = swing_back_end_;
  swing_forward_end_ = swing_forward_start_ + swing_forward_time;

  follow_through_start_ = swing_forward_end_;
  follow_through_end_ = follow_through_start_ + follow_through_time;

  recovery_start_ = follow_through_end_;
  recovery_end_ = recovery_start_ + recovery_time;

  total_duration_ = recovery_end_;

  // =========================================================================
  // Define Key Poses
  // =========================================================================
  
  // Konversi sudut dari derajat ke radian
  double swing_back_rad = kick_params.swing_back_angle * M_PI / 180.0;
  double follow_through_rad = kick_params.follow_through * M_PI / 180.0;

  // Pose saat kaki diangkat (siap ayun)
  LegJointState lift_pose;
  lift_pose.hip_yaw = initial_state_.hip_yaw;
  lift_pose.hip_roll = initial_state_.hip_roll + 0.05;  // Sedikit roll untuk keseimbangan
  lift_pose.hip_pitch = initial_state_.hip_pitch - 0.3; // Angkat paha
  lift_pose.knee = initial_state_.knee + 0.5;           // Tekuk lutut
  lift_pose.ankle_pitch = initial_state_.ankle_pitch + 0.2;
  lift_pose.ankle_roll = initial_state_.ankle_roll;

  // Pose saat swing back (ayunan ke belakang)
  LegJointState swing_back_pose;
  swing_back_pose.hip_yaw = initial_state_.hip_yaw;
  swing_back_pose.hip_roll = lift_pose.hip_roll;
  swing_back_pose.hip_pitch = initial_state_.hip_pitch + swing_back_rad; // Ayun ke belakang
  swing_back_pose.knee = initial_state_.knee + 0.3;  // Sedikit tekuk
  swing_back_pose.ankle_pitch = initial_state_.ankle_pitch - 0.1;
  swing_back_pose.ankle_roll = initial_state_.ankle_roll;

  // Pose saat kontak dengan bola (swing forward)
  LegJointState contact_pose;
  contact_pose.hip_yaw = initial_state_.hip_yaw;
  contact_pose.hip_roll = lift_pose.hip_roll;
  contact_pose.hip_pitch = initial_state_.hip_pitch - 0.5 * kick_params.strength; // Ayun ke depan
  contact_pose.knee = initial_state_.knee + 0.1;  // Hampir lurus
  contact_pose.ankle_pitch = initial_state_.ankle_pitch + 0.1;
  contact_pose.ankle_roll = initial_state_.ankle_roll;

  // Pose follow through
  LegJointState follow_through_pose;
  follow_through_pose.hip_yaw = initial_state_.hip_yaw;
  follow_through_pose.hip_roll = lift_pose.hip_roll;
  follow_through_pose.hip_pitch = contact_pose.hip_pitch - follow_through_rad;
  follow_through_pose.knee = contact_pose.knee;
  follow_through_pose.ankle_pitch = contact_pose.ankle_pitch;
  follow_through_pose.ankle_roll = initial_state_.ankle_roll;

  // =========================================================================
  // Generate Trajectory Segments
  // =========================================================================
  
  // Kecepatan nol untuk awal dan akhir (smooth start/stop)
  LegJointState zero_vel;
  zero_vel.hip_yaw = 0.0;
  zero_vel.hip_roll = 0.0;
  zero_vel.hip_pitch = 0.0;
  zero_vel.knee = 0.0;
  zero_vel.ankle_pitch = 0.0;
  zero_vel.ankle_roll = 0.0;

  // Kecepatan tinggi untuk kontak
  LegJointState contact_vel;
  contact_vel.hip_yaw = 0.0;
  contact_vel.hip_roll = 0.0;
  contact_vel.hip_pitch = -kick_params.contact_velocity * kick_params.strength;
  contact_vel.knee = 0.0;
  contact_vel.ankle_pitch = 0.0;
  contact_vel.ankle_roll = 0.0;

  // Fase 1: Lift Leg
  generatePhaseTrajectory(KickPhase::LIFT_LEG,
                          lift_leg_start_, lift_leg_end_,
                          initial_state_, lift_pose,
                          zero_vel, zero_vel);

  // Fase 2: Swing Back
  generatePhaseTrajectory(KickPhase::SWING_BACK,
                          swing_back_start_, swing_back_end_,
                          lift_pose, swing_back_pose,
                          zero_vel, zero_vel);

  // Fase 3: Swing Forward (dengan kecepatan tinggi di akhir)
  generatePhaseTrajectory(KickPhase::SWING_FORWARD,
                          swing_forward_start_, swing_forward_end_,
                          swing_back_pose, contact_pose,
                          zero_vel, contact_vel);

  // Fase 4: Follow Through
  generatePhaseTrajectory(KickPhase::FOLLOW_THROUGH,
                          follow_through_start_, follow_through_end_,
                          contact_pose, follow_through_pose,
                          contact_vel, zero_vel);

  // Fase 5: Recovery
  generatePhaseTrajectory(KickPhase::RECOVERY,
                          recovery_start_, recovery_end_,
                          follow_through_pose, initial_state_,
                          zero_vel, zero_vel);

  trajectory_generated_ = true;
  return true;
}

void KickTrajectory::generatePhaseTrajectory(KickPhase phase,
                                              double start_time, double end_time,
                                              const LegJointState& start_state,
                                              const LegJointState& end_state,
                                              const LegJointState& start_vel,
                                              const LegJointState& end_vel)
{
  // Generate untuk setiap joint
  auto addSegment = [&](std::vector<JointTrajectorySegment>& trajectory,
                        double p0, double pf, double v0, double vf) {
    JointTrajectorySegment segment;
    segment.start_time = start_time;
    segment.end_time = end_time;
    segment.phase = phase;
    segment.coeffs = computeMinimumJerkCoeffs(start_time, end_time, p0, pf, v0, vf, 0.0, 0.0);
    trajectory.push_back(segment);
  };

  addSegment(hip_yaw_trajectory_, start_state.hip_yaw, end_state.hip_yaw,
             start_vel.hip_yaw, end_vel.hip_yaw);
  addSegment(hip_roll_trajectory_, start_state.hip_roll, end_state.hip_roll,
             start_vel.hip_roll, end_vel.hip_roll);
  addSegment(hip_pitch_trajectory_, start_state.hip_pitch, end_state.hip_pitch,
             start_vel.hip_pitch, end_vel.hip_pitch);
  addSegment(knee_trajectory_, start_state.knee, end_state.knee,
             start_vel.knee, end_vel.knee);
  addSegment(ankle_pitch_trajectory_, start_state.ankle_pitch, end_state.ankle_pitch,
             start_vel.ankle_pitch, end_vel.ankle_pitch);
  addSegment(ankle_roll_trajectory_, start_state.ankle_roll, end_state.ankle_roll,
             start_vel.ankle_roll, end_vel.ankle_roll);
}

Eigen::VectorXd KickTrajectory::computeMinimumJerkCoeffs(double t0, double tf,
                                                          double p0, double pf,
                                                          double v0, double vf,
                                                          double a0, double af)
{
  // Minimum jerk trajectory menggunakan polynomial orde 5
  // p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
  
  Eigen::MatrixXd T(6, 6);
  Eigen::VectorXd b(6);
  
  auto pow2 = [](double x) { return x * x; };
  auto pow3 = [](double x) { return x * x * x; };
  auto pow4 = [](double x) { return x * x * x * x; };
  auto pow5 = [](double x) { return x * x * x * x * x; };
  
  // Boundary conditions matrix
  T << 1, t0, pow2(t0), pow3(t0), pow4(t0), pow5(t0),
       0, 1, 2*t0, 3*pow2(t0), 4*pow3(t0), 5*pow4(t0),
       0, 0, 2, 6*t0, 12*pow2(t0), 20*pow3(t0),
       1, tf, pow2(tf), pow3(tf), pow4(tf), pow5(tf),
       0, 1, 2*tf, 3*pow2(tf), 4*pow3(tf), 5*pow4(tf),
       0, 0, 2, 6*tf, 12*pow2(tf), 20*pow3(tf);
  
  // Boundary conditions vector
  b << p0, v0, a0, pf, vf, af;
  
  // Solve for coefficients
  Eigen::VectorXd coeffs = T.colPivHouseholderQr().solve(b);
  
  return coeffs;
}

double KickTrajectory::evaluatePolynomial(const Eigen::VectorXd& coeffs, double t) const
{
  return coeffs(0) + coeffs(1)*t + coeffs(2)*t*t + coeffs(3)*t*t*t +
         coeffs(4)*t*t*t*t + coeffs(5)*t*t*t*t*t;
}

double KickTrajectory::evaluatePolynomialDerivative(const Eigen::VectorXd& coeffs, double t) const
{
  return coeffs(1) + 2*coeffs(2)*t + 3*coeffs(3)*t*t +
         4*coeffs(4)*t*t*t + 5*coeffs(5)*t*t*t*t;
}

LegJointState KickTrajectory::getJointPosition(double time) const
{
  LegJointState state;
  
  if (!trajectory_generated_)
  {
    return initial_state_;
  }

  // Clamp time
  if (time < 0) time = 0;
  if (time > total_duration_) time = total_duration_;

  // Find dan evaluate segment yang sesuai untuk setiap joint
  auto evaluateJoint = [&](const std::vector<JointTrajectorySegment>& trajectory) -> double {
    for (const auto& seg : trajectory)
    {
      if (time >= seg.start_time && time <= seg.end_time)
      {
        return evaluatePolynomial(seg.coeffs, time);
      }
    }
    // Return nilai terakhir jika tidak ditemukan
    if (!trajectory.empty())
    {
      return evaluatePolynomial(trajectory.back().coeffs, trajectory.back().end_time);
    }
    return 0.0;
  };

  state.hip_yaw = evaluateJoint(hip_yaw_trajectory_);
  state.hip_roll = evaluateJoint(hip_roll_trajectory_);
  state.hip_pitch = evaluateJoint(hip_pitch_trajectory_);
  state.knee = evaluateJoint(knee_trajectory_);
  state.ankle_pitch = evaluateJoint(ankle_pitch_trajectory_);
  state.ankle_roll = evaluateJoint(ankle_roll_trajectory_);

  return state;
}

LegJointState KickTrajectory::getJointVelocity(double time) const
{
  LegJointState velocity;
  
  if (!trajectory_generated_)
  {
    return velocity;  // Return zero velocity
  }

  // Clamp time
  if (time < 0) time = 0;
  if (time > total_duration_) time = total_duration_;

  auto evaluateJointVel = [&](const std::vector<JointTrajectorySegment>& trajectory) -> double {
    for (const auto& seg : trajectory)
    {
      if (time >= seg.start_time && time <= seg.end_time)
      {
        return evaluatePolynomialDerivative(seg.coeffs, time);
      }
    }
    return 0.0;
  };

  velocity.hip_yaw = evaluateJointVel(hip_yaw_trajectory_);
  velocity.hip_roll = evaluateJointVel(hip_roll_trajectory_);
  velocity.hip_pitch = evaluateJointVel(hip_pitch_trajectory_);
  velocity.knee = evaluateJointVel(knee_trajectory_);
  velocity.ankle_pitch = evaluateJointVel(ankle_pitch_trajectory_);
  velocity.ankle_roll = evaluateJointVel(ankle_roll_trajectory_);

  return velocity;
}

KickPhase KickTrajectory::getPhase(double time) const
{
  if (!trajectory_generated_)
    return KickPhase::IDLE;

  if (time < lift_leg_start_)
    return KickPhase::IDLE;
  else if (time < lift_leg_end_)
    return KickPhase::LIFT_LEG;
  else if (time < swing_back_end_)
    return KickPhase::SWING_BACK;
  else if (time < swing_forward_end_)
    return KickPhase::SWING_FORWARD;
  else if (time < follow_through_end_)
    return KickPhase::FOLLOW_THROUGH;
  else if (time < recovery_end_)
    return KickPhase::RECOVERY;
  else
    return KickPhase::COMPLETED;
}

void KickTrajectory::reset()
{
  hip_yaw_trajectory_.clear();
  hip_roll_trajectory_.clear();
  hip_pitch_trajectory_.clear();
  knee_trajectory_.clear();
  ankle_pitch_trajectory_.clear();
  ankle_roll_trajectory_.clear();

  total_duration_ = 0.0;
  trajectory_generated_ = false;
}

}  // namespace op3_kick_module
