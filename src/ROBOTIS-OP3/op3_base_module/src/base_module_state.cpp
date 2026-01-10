// =============================================================================
// base_module_state.cpp
// =============================================================================
// Part of src package
// =============================================================================

/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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

/* Author: SCH */

// ROBOTIS-OP3/op3_base_module/src/base_module_state.cpp

// Menyertakan header file untuk kelas BaseModuleState
#include "op3_base_module/base_module_state.h"

// Namespace untuk modul robot OP3
namespace robotis_op
{

// Konstruktor kelas BaseModuleState - menginisialisasi semua variabel state
BaseModuleState::BaseModuleState()
{
  // Flag untuk menandakan apakah robot sedang bergerak
  is_moving_ = false;

  // Counter untuk melacak langkah trajectory saat ini
  cnt_ = 0;

  // Waktu total pergerakan dalam detik (1 detik)
  mov_time_ = 1.0;
  // Waktu sampling dalam detik (8 milidetik = 125 Hz)
  smp_time_ = 0.008;
  // Total langkah waktu untuk trajectory = waktu_total / waktu_sampling + 1
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  // Matriks untuk menyimpan trajectory joint yang dihitung
  // Ukuran: jumlah_langkah x (jumlah_joint + 1)
  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);

  // Matriks untuk menyimpan pose awal joint (posisi inisial)
  joint_ini_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);
  // Matriks untuk menyimpan pose target joint
  joint_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  // Jumlah via-point (titik antara) untuk trajectory
  via_num_ = 1;

  // Matriks untuk menyimpan pose via-point (posisi titik antara)
  joint_via_pose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  // Matriks untuk menyimpan kecepatan di via-point (turunan pertama)
  joint_via_dpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  // Matriks untuk menyimpan akselerasi di via-point (turunan kedua)
  joint_via_ddpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);

  // Matriks untuk menyimpan waktu pada setiap via-point
  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);
}

// Destruktor kelas BaseModuleState - tidak ada resource yang perlu dibebaskan
BaseModuleState::~BaseModuleState()
{
}

}
