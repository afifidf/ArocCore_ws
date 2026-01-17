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

#ifndef OP3_KICK_MODULE__KICK_PLANNER_H_
#define OP3_KICK_MODULE__KICK_PLANNER_H_

#include <cmath>
#include "op3_kick_module/kick_types.h"

namespace op3_kick_module
{

/**
 * @brief Perencana tendangan yang menentukan parameter optimal
 * 
 * Kelas ini bertanggung jawab untuk:
 * - Menentukan kaki mana yang digunakan (kiri/kanan)
 * - Menentukan tipe tendangan berdasarkan jarak target
 * - Menghitung kekuatan dan arah tendangan
 * - Memvalidasi apakah tendangan dapat dilakukan
 */
class KickPlanner
{
public:
  KickPlanner();
  ~KickPlanner();

  /**
   * @brief Inisialisasi planner dengan konfigurasi
   * @param config Konfigurasi kick module
   */
  void initialize(const KickConfig& config);

  /**
   * @brief Rencanakan tendangan berdasarkan posisi bola dan target
   * @param ball_state Status bola saat ini
   * @param target Target tendangan (posisi gawang atau rekan)
   * @param robot_pose Pose robot saat ini
   * @return Rencana tendangan
   */
  KickPlan planKick(const BallState& ball_state,
                    const Point2D& target,
                    const Pose2D& robot_pose);

  /**
   * @brief Rencanakan tendangan sederhana (tanpa target spesifik)
   * @param ball_state Status bola saat ini
   * @return Rencana tendangan
   */
  KickPlan planSimpleKick(const BallState& ball_state);

  /**
   * @brief Tentukan kaki yang akan digunakan berdasarkan posisi bola
   * @param ball_angle Sudut bola relatif terhadap robot (radian)
   * @return Kaki yang digunakan
   */
  KickFoot determineFoot(double ball_angle);

  /**
   * @brief Tentukan tipe tendangan berdasarkan jarak ke target
   * @param distance Jarak ke target (meter)
   * @return Tipe tendangan
   */
  KickType determineKickType(double distance);

  /**
   * @brief Hitung kekuatan tendangan berdasarkan jarak
   * @param distance Jarak ke target (meter)
   * @param kick_type Tipe tendangan
   * @return Kekuatan (0.0 - 1.0)
   */
  double calculateStrength(double distance, KickType kick_type);

  /**
   * @brief Hitung arah tendangan
   * @param ball_pos Posisi bola
   * @param target_pos Posisi target
   * @param robot_pose Pose robot
   * @return Arah tendangan dalam radian
   */
  double calculateDirection(const Point2D& ball_pos,
                            const Point2D& target_pos,
                            const Pose2D& robot_pose);

  /**
   * @brief Cek apakah bola dalam posisi yang bisa ditendang
   * @param ball_state Status bola
   * @return true jika bisa ditendang
   */
  bool isBallKickable(const BallState& ball_state);

  /**
   * @brief Cek apakah robot perlu alignment sebelum tendang
   * @param ball_state Status bola
   * @return Status alignment
   */
  AlignmentStatus checkAlignment(const BallState& ball_state);

  /**
   * @brief Hitung adjustment yang diperlukan untuk alignment
   * @param ball_state Status bola
   * @param adjustment_x Output: adjustment forward/backward
   * @param adjustment_y Output: adjustment left/right
   * @param adjustment_theta Output: adjustment rotasi
   */
  void calculateAlignmentAdjustment(const BallState& ball_state,
                                     double& adjustment_x,
                                     double& adjustment_y,
                                     double& adjustment_theta);

  /**
   * @brief Update konfigurasi
   * @param config Konfigurasi baru
   */
  void updateConfig(const KickConfig& config);

private:
  /**
   * @brief Hitung jarak antara dua titik
   */
  double calculateDistance(const Point2D& p1, const Point2D& p2);

  /**
   * @brief Hitung sudut antara dua titik
   */
  double calculateAngle(const Point2D& from, const Point2D& to);

  /**
   * @brief Normalisasi sudut ke range [-pi, pi]
   */
  double normalizeAngle(double angle);

  // Konfigurasi
  KickConfig config_;
  
  // Parameter jarak untuk menentukan tipe tendangan
  double short_pass_max_distance_;   // Jarak maksimal untuk short pass (meter)
  double long_pass_max_distance_;    // Jarak maksimal untuk long pass (meter)
  
  // Parameter kekuatan
  double min_strength_;
  double max_strength_;
  
  // Flag
  bool is_initialized_;
};

}  // namespace op3_kick_module

#endif  // OP3_KICK_MODULE__KICK_PLANNER_H_
