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

#ifndef OP3_KICK_MODULE__KICK_TRAJECTORY_H_
#define OP3_KICK_MODULE__KICK_TRAJECTORY_H_

#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>
#include "op3_kick_module/kick_types.h"

namespace op3_kick_module
{

/**
 * @brief Generator trajectory untuk tendangan menggunakan minimum jerk
 * 
 * Kelas ini menghasilkan trajectory yang smooth untuk gerakan tendangan
 * dengan meminimalkan jerk (turunan ke-3 dari posisi).
 * 
 * Trajectory dibagi menjadi beberapa fase:
 * 1. Lift leg - Angkat kaki dari tanah
 * 2. Swing back - Ayunan ke belakang
 * 3. Swing forward - Ayunan ke depan (kontak dengan bola)
 * 4. Follow through - Lanjutan setelah kontak
 * 5. Recovery - Kembali ke posisi awal
 */
class KickTrajectory
{
public:
  KickTrajectory();
  ~KickTrajectory();

  /**
   * @brief Inisialisasi trajectory generator
   * @param control_cycle_sec Waktu siklus kontrol dalam detik
   */
  void initialize(double control_cycle_sec);

  /**
   * @brief Generate trajectory untuk tendangan
   * @param kick_params Parameter tendangan (strength, swing angle, dll)
   * @param initial_joint_state Posisi joint awal
   * @param foot Kaki yang digunakan (LEFT/RIGHT)
   * @return true jika berhasil generate trajectory
   */
  bool generateKickTrajectory(const KickTypeParams& kick_params,
                               const LegJointState& initial_joint_state,
                               KickFoot foot);

  /**
   * @brief Dapatkan posisi joint pada waktu tertentu
   * @param time Waktu dalam detik sejak mulai tendangan
   * @return State joint kaki pada waktu tersebut
   */
  LegJointState getJointPosition(double time) const;

  /**
   * @brief Dapatkan kecepatan joint pada waktu tertentu
   * @param time Waktu dalam detik sejak mulai tendangan
   * @return Kecepatan joint kaki pada waktu tersebut
   */
  LegJointState getJointVelocity(double time) const;

  /**
   * @brief Dapatkan fase tendangan pada waktu tertentu
   * @param time Waktu dalam detik
   * @return Fase tendangan saat ini
   */
  KickPhase getPhase(double time) const;

  /**
   * @brief Dapatkan total durasi trajectory
   * @return Durasi dalam detik
   */
  double getTotalDuration() const { return total_duration_; }

  /**
   * @brief Cek apakah trajectory sudah selesai
   * @param time Waktu dalam detik
   * @return true jika sudah selesai
   */
  bool isFinished(double time) const { return time >= total_duration_; }

  /**
   * @brief Reset trajectory
   */
  void reset();

private:
  /**
   * @brief Hitung koefisien polynomial orde-5 untuk minimum jerk
   * @param t0 Waktu awal
   * @param tf Waktu akhir
   * @param p0 Posisi awal
   * @param pf Posisi akhir
   * @param v0 Kecepatan awal
   * @param vf Kecepatan akhir
   * @param a0 Akselerasi awal
   * @param af Akselerasi akhir
   * @return Vector koefisien [a0, a1, a2, a3, a4, a5]
   */
  Eigen::VectorXd computeMinimumJerkCoeffs(double t0, double tf,
                                            double p0, double pf,
                                            double v0, double vf,
                                            double a0, double af);

  /**
   * @brief Evaluasi polynomial pada waktu t
   * @param coeffs Koefisien polynomial
   * @param t Waktu
   * @return Nilai pada waktu t
   */
  double evaluatePolynomial(const Eigen::VectorXd& coeffs, double t) const;

  /**
   * @brief Evaluasi turunan pertama polynomial pada waktu t
   * @param coeffs Koefisien polynomial
   * @param t Waktu
   * @return Nilai turunan pada waktu t
   */
  double evaluatePolynomialDerivative(const Eigen::VectorXd& coeffs, double t) const;

  /**
   * @brief Generate trajectory untuk satu fase
   */
  void generatePhaseTrajectory(KickPhase phase,
                                double start_time, double end_time,
                                const LegJointState& start_state,
                                const LegJointState& end_state,
                                const LegJointState& start_vel,
                                const LegJointState& end_vel);

  // Control cycle
  double control_cycle_sec_;

  // Trajectory storage untuk setiap joint
  struct JointTrajectorySegment
  {
    double start_time;
    double end_time;
    KickPhase phase;
    Eigen::VectorXd coeffs;  // Koefisien polynomial
  };

  std::vector<JointTrajectorySegment> hip_yaw_trajectory_;
  std::vector<JointTrajectorySegment> hip_roll_trajectory_;
  std::vector<JointTrajectorySegment> hip_pitch_trajectory_;
  std::vector<JointTrajectorySegment> knee_trajectory_;
  std::vector<JointTrajectorySegment> ankle_pitch_trajectory_;
  std::vector<JointTrajectorySegment> ankle_roll_trajectory_;

  // Timing untuk setiap fase
  double lift_leg_start_, lift_leg_end_;
  double swing_back_start_, swing_back_end_;
  double swing_forward_start_, swing_forward_end_;
  double follow_through_start_, follow_through_end_;
  double recovery_start_, recovery_end_;

  double total_duration_;

  // Foot being used
  KickFoot kick_foot_;

  // Initial state
  LegJointState initial_state_;

  // Flag
  bool is_initialized_;
  bool trajectory_generated_;
};

}  // namespace op3_kick_module

#endif  // OP3_KICK_MODULE__KICK_TRAJECTORY_H_
