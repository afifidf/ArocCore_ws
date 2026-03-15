// =============================================================================
// link_data.cpp
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

/* Authors: SCH, Jay Song */

// ROBOTIS-OP3/op3_kinematics_dynamics/src/link_data.cpp

// Menyertakan header file untuk kelas LinkData
#include "op3_kinematics_dynamics/link_data.h"

// Namespace untuk modul robot OP3
namespace robotis_op
{

// Konstruktor kelas LinkData - menginisialisasi data link/joint robot
LinkData::LinkData()
{
  // Nama link (kosong secara default)
  name_ = "";

  // ID parent link dalam kinematic tree (-1 = tidak ada parent)
  parent_ = -1;
  // ID sibling link (link sejajar dalam tree, -1 = tidak ada)
  sibling_ = -1;
  // ID child link (link anak, -1 = tidak ada)
  child_ = -1;

  // Massa link dalam kilogram
  mass_ = 0.0;

  // Posisi relatif link terhadap parent frame (x, y, z) dalam meter
  relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  // Sumbu rotasi joint (vektor unit untuk arah rotasi)
  joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  // Posisi center of mass relatif terhadap link frame
  center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  // Tensor inersia link (Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
  inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Batas maksimum sudut joint dalam radian (100 rad = hampir tidak terbatas)
  joint_limit_max_ = 100.0;
  // Batas minimum sudut joint dalam radian
  joint_limit_min_ = -100.0;

  // Sudut joint saat ini dalam radian
  joint_angle_ = 0.0;
  // Kecepatan angular joint dalam rad/s
  joint_velocity_ = 0.0;
  // Akselerasi angular joint dalam rad/s^2
  joint_acceleration_ = 0.0;

  // Posisi absolut link dalam world frame (x, y, z)
  position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  // Matriks rotasi 3x3 untuk orientasi link dalam world frame
  orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
  // Matriks transformasi homogen 4x4 (posisi + orientasi)
  transformation_ = robotis_framework::getTransformationXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Destruktor kelas LinkData - tidak ada resource yang perlu dibebaskan
LinkData::~LinkData()
{
}

}
