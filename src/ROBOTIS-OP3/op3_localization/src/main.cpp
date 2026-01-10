// =============================================================================
// main.cpp
// =============================================================================
// Part of src package
// =============================================================================

/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

// ROBOTIS-OP3/op3_direct_control_module/src/main.cpp

// Menyertakan header file untuk kelas OP3Localization
#include "op3_localization/op3_localization.h"
// Menyertakan library ROS2 untuk node dan komunikasi
#include "rclcpp/rclcpp.hpp"

// Fungsi utama program - entry point untuk node lokalisasi OP3
int main(int argc, char **argv)
{
  // Inisialisasi ROS2 dengan argumen command line
  rclcpp::init(argc, argv);

  // Membuat shared pointer ke objek OP3Localization (node utama)
  auto node = std::make_shared<robotis_op::OP3Localization>();

  // Mengatur rate loop utama (10 Hz = 10 kali per detik)
  rclcpp::Rate loop_rate(10);

  // Loop utama node - berjalan selama ROS2 masih aktif
  while ( rclcpp::ok() )
  {
    // Menjalankan proses utama lokalisasi
    node->process();

    // Memproses callback yang tertunda (subscription, service, dll)
    rclcpp::spin_some(node);

    // Menunggu hingga waktu loop berikutnya untuk menjaga rate 10 Hz
    loop_rate.sleep();
  }

  // Menutup ROS2 dan membersihkan resource
  rclcpp::shutdown();
  // Mengembalikan 0 menandakan program berakhir dengan sukses
  return 0;
}
