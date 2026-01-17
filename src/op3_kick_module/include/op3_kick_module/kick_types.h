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

#ifndef OP3_KICK_MODULE__KICK_TYPES_H_
#define OP3_KICK_MODULE__KICK_TYPES_H_

#include <string>
#include <map>

namespace op3_kick_module
{

// =============================================================================
// Enumerations
// =============================================================================

enum class KickFoot
{
  LEFT = 0,
  RIGHT = 1
};

enum class KickType
{
  SHORT_PASS = 0,   // Umpan pendek, kekuatan rendah
  LONG_PASS = 1,    // Umpan jauh, kekuatan sedang
  SHOOT = 2,        // Tendangan ke gawang, kekuatan maksimal
  SIDE_KICK = 3,    // Tendangan samping
  BACK_KICK = 4     // Tendangan ke belakang (opsional)
};

enum class KickPhase
{
  IDLE = 0,           // Tidak ada tendangan
  ALIGNING = 1,       // Menyesuaikan posisi
  STABILIZING = 2,    // Menunggu IMU stabil
  LIFT_LEG = 3,       // Mengangkat kaki tendang
  SWING_BACK = 4,     // Ayunan ke belakang
  SWING_FORWARD = 5,  // Ayunan ke depan (kontak dengan bola)
  FOLLOW_THROUGH = 6, // Follow-through setelah kontak
  RECOVERY = 7,       // Kembali ke posisi normal
  COMPLETED = 8,      // Tendangan selesai
  FAILED = 9          // Tendangan gagal
};

enum class AlignmentStatus
{
  NOT_ALIGNED = 0,
  ALIGNING = 1,
  ALIGNED = 2,
  TIMEOUT = 3
};

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Parameter untuk satu tipe tendangan
 */
struct KickTypeParams
{
  double strength;          // Kekuatan tendangan (0.0 - 1.0)
  double swing_back_angle;  // Sudut ayunan ke belakang (derajat)
  double swing_time;        // Waktu total ayunan (detik)
  double contact_velocity;  // Kecepatan saat kontak (rad/s)
  double follow_through;    // Sudut follow-through (derajat)
  
  KickTypeParams()
    : strength(0.5), swing_back_angle(30.0), swing_time(0.4),
      contact_velocity(5.0), follow_through(15.0) {}
};

/**
 * @brief Parameter alignment sebelum tendangan
 */
struct AlignmentParams
{
  double distance_to_ball;      // Jarak ideal ke bola (meter)
  double distance_tolerance;    // Toleransi jarak (meter)
  double angle_tolerance;       // Toleransi sudut (derajat)
  double y_offset_tolerance;    // Toleransi offset Y (meter)
  double stability_threshold;   // Threshold stabilitas IMU (rad/s)
  double alignment_timeout;     // Timeout untuk alignment (detik)
  
  AlignmentParams()
    : distance_to_ball(0.18), distance_tolerance(0.02), angle_tolerance(8.0),
      y_offset_tolerance(0.03), stability_threshold(0.02), alignment_timeout(5.0) {}
};

/**
 * @brief Parameter balance control saat tendangan
 */
struct KickBalanceParams
{
  double hip_roll_gain;
  double knee_gain;
  double ankle_roll_gain;
  double ankle_pitch_gain;
  double gyro_roll_gain;
  double gyro_pitch_gain;
  
  KickBalanceParams()
    : hip_roll_gain(0.35), knee_gain(0.3), ankle_roll_gain(0.7),
      ankle_pitch_gain(0.9), gyro_roll_gain(0.3), gyro_pitch_gain(0.5) {}
};

/**
 * @brief Posisi 2D
 */
struct Point2D
{
  double x;
  double y;
  
  Point2D() : x(0.0), y(0.0) {}
  Point2D(double _x, double _y) : x(_x), y(_y) {}
};

/**
 * @brief Posisi 3D dengan orientasi
 */
struct Pose2D
{
  double x;
  double y;
  double theta;  // Orientasi (radian)
  
  Pose2D() : x(0.0), y(0.0), theta(0.0) {}
  Pose2D(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
};

/**
 * @brief Status bola relatif terhadap robot
 */
struct BallState
{
  Point2D position;         // Posisi relatif ke robot
  double distance;          // Jarak ke bola
  double angle;             // Sudut ke bola (radian)
  bool is_valid;            // Apakah deteksi valid
  double confidence;        // Confidence level (0.0 - 1.0)
  
  BallState()
    : distance(0.0), angle(0.0), is_valid(false), confidence(0.0) {}
};

/**
 * @brief Parameter hasil perencanaan tendangan
 */
struct KickPlan
{
  KickFoot foot;            // Kaki yang digunakan
  KickType type;            // Tipe tendangan
  double strength;          // Kekuatan (0.0 - 1.0)
  double direction;         // Arah tendangan (radian)
  Point2D target;           // Target tendangan
  bool is_valid;            // Apakah plan valid
  std::string reason;       // Alasan jika tidak valid
  
  KickPlan()
    : foot(KickFoot::RIGHT), type(KickType::SHORT_PASS), strength(0.5),
      direction(0.0), is_valid(false), reason("") {}
};

/**
 * @brief Status IMU untuk balance
 */
struct IMUState
{
  double roll;              // Roll angle (radian)
  double pitch;             // Pitch angle (radian)
  double yaw;               // Yaw angle (radian)
  double roll_velocity;     // Roll angular velocity (rad/s)
  double pitch_velocity;    // Pitch angular velocity (rad/s)
  double yaw_velocity;      // Yaw angular velocity (rad/s)
  
  IMUState()
    : roll(0.0), pitch(0.0), yaw(0.0),
      roll_velocity(0.0), pitch_velocity(0.0), yaw_velocity(0.0) {}
  
  bool isStable(double threshold) const
  {
    return (std::abs(roll_velocity) < threshold &&
            std::abs(pitch_velocity) < threshold);
  }
};

/**
 * @brief Status joint kaki untuk tendangan
 */
struct LegJointState
{
  double hip_yaw;
  double hip_roll;
  double hip_pitch;
  double knee;
  double ankle_pitch;
  double ankle_roll;
  
  LegJointState()
    : hip_yaw(0.0), hip_roll(0.0), hip_pitch(0.0),
      knee(0.0), ankle_pitch(0.0), ankle_roll(0.0) {}
};

/**
 * @brief Konfigurasi lengkap kick module
 */
struct KickConfig
{
  std::map<KickType, KickTypeParams> kick_types;
  AlignmentParams alignment;
  KickBalanceParams balance;
  
  // Timing parameters
  double control_cycle_sec;     // Siklus kontrol (detik)
  double lift_leg_time;         // Waktu angkat kaki (detik)
  double recovery_time;         // Waktu recovery (detik)
  
  // Joint limits
  double max_hip_pitch;         // Batas maksimal hip pitch (radian)
  double max_knee_bend;         // Batas maksimal knee bend (radian)
  
  KickConfig()
    : control_cycle_sec(0.008), lift_leg_time(0.3), recovery_time(0.5),
      max_hip_pitch(1.57), max_knee_bend(2.0) {}
};

}  // namespace op3_kick_module

#endif  // OP3_KICK_MODULE__KICK_TYPES_H_
