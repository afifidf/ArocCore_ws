// =============================================================================
// base_module.cpp
// =============================================================================
// Base Module untuk Robot OP3
// - Module dasar untuk pose control
// - Initial pose dan idle pose
// - Direct joint control
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

/* Author: Kayman, SCH */

// ROBOTIS-OP3/op3_base_module/src/base_module.cpp

// Menyertakan library ROS2 untuk node dan komunikasi
#include <rclcpp/rclcpp.hpp>
// Menyertakan message type String untuk komunikasi ROS
#include <std_msgs/msg/string.hpp>
// Menyertakan message type StatusMsg untuk status robot
#include <robotis_controller_msgs/msg/status_msg.hpp>
// Menyertakan service type SetModule untuk mengatur modul kontrol
#include <robotis_controller_msgs/srv/set_module.hpp>
// Menyertakan library YAML untuk parsing file konfigurasi
#include <yaml-cpp/yaml.h>
// Menyertakan library Eigen untuk operasi matriks dan aljabar linear
#include <Eigen/Dense>
// Menyertakan library untuk multithreading
#include <thread>
// Menyertakan library untuk operasi waktu
#include <chrono>
// Menyertakan header file kelas BaseModule
#include "op3_base_module/base_module.h"

// Namespace untuk modul robot OP3
namespace robotis_op
{

// Konstruktor kelas BaseModule - menginisialisasi node ROS2 dan variabel member
BaseModule::BaseModule()
  : Node("op3_base_module"),          // Inisialisasi node dengan nama "op3_base_module"
    control_cycle_msec_(0),            // Waktu siklus kontrol (milidetik)
    has_goal_joints_(false),           // Flag apakah sudah menerima goal joint
    ini_pose_only_(false),             // Flag untuk mode initial pose saja
    init_pose_file_path_("")           // Path file konfigurasi pose awal
{
  // Modul belum diaktifkan
  enable_ = false;
  // Nama modul ini
  module_name_ = "base_module";
  // Mode kontrol menggunakan kontrol posisi
  control_mode_ = robotis_framework::PositionControl;

  // Membuat objek state modul untuk menyimpan kondisi pergerakan
  base_module_state_ = new BaseModuleState();
  // Membuat objek joint state untuk menyimpan kondisi joint
  joint_state_ = new BaseJointState();
}

// Destruktor kelas BaseModule - membersihkan resource
BaseModule::~BaseModule()
{
  // Menunggu thread queue selesai jika masih berjalan
  if (queue_thread_ && queue_thread_->joinable())
    queue_thread_->join();

  // Menghapus semua objek DynamixelState yang dialokasikan
  for (auto& [joint_name, state] : result_)
    delete state;

  // Menghapus objek state modul
  delete base_module_state_;
  // Menghapus objek joint state
  delete joint_state_;
}

// Fungsi inisialisasi modul - dipanggil saat modul pertama kali dimuat
void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  // Menyimpan waktu siklus kontrol
  control_cycle_msec_ = control_cycle_msec;

  // Menginisialisasi result dan tabel ID joint
  // Iterasi semua Dynamixel yang terdaftar di robot
  for (const auto& [joint_name, dxl_info] : robot->dxls_)
  {
    // Membuat mapping nama joint ke ID
    joint_name_to_id_[joint_name] = dxl_info->id_;
    // Membuat objek DynamixelState baru untuk setiap joint
    result_[joint_name] = new robotis_framework::DynamixelState();
    // Menyalin posisi goal dari state Dynamixel
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  // Memuat parameter ROS untuk path file pose awal
  this->declare_parameter<std::string>("init_pose_file_path", ament_index_cpp::get_package_share_directory("op3_base_module") + "/data/ini_pose.yaml");
  // Mengambil nilai parameter
  init_pose_file_path_ = this->get_parameter("init_pose_file_path").as_string();

  // Membuat publisher untuk mengirim pesan status
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  // Membuat publisher untuk mengaktifkan modul kontrol
  set_ctrl_module_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 1);

  // Membuat thread baru untuk menangani callback ROS
  queue_thread_ = std::make_unique<std::thread>(&BaseModule::queueThread, this);
}

// Fungsi untuk memparse data pose awal dari file YAML
void BaseModule::parseInitPoseData(const std::string &path)
{
  // Objek untuk menyimpan data YAML
  YAML::Node doc;
  try
  {
    // Memuat file YAML dari path yang diberikan
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    // Menampilkan error jika gagal memuat file
    RCLCPP_ERROR_STREAM(this->get_logger(), "Fail to load yaml file.");
    return;
  }

  // Memparse waktu pergerakan dari file YAML
  double mov_time;
  mov_time = doc["mov_time"].as<double>();

  // Menyimpan waktu pergerakan ke state modul
  base_module_state_->mov_time_ = mov_time;

  // Memparse jumlah via-point dari file YAML
  int via_num;
  via_num = doc["via_num"].as<int>();

  // Menyimpan jumlah via-point ke state modul
  base_module_state_->via_num_ = via_num;

  // Memparse waktu untuk setiap via-point
  std::vector<double> via_time;
  via_time = doc["via_time"].as<std::vector<double> >();

  // Mengubah ukuran matriks via_time sesuai jumlah via-point
  base_module_state_->via_time_.resize(via_num, 1);
  // Menyalin waktu via-point ke matriks
  for (int num = 0; num < via_num; num++)
    base_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

  // Mengubah ukuran matriks pose via-point
  base_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
  // Mengubah ukuran matriks kecepatan via-point
  base_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
  // Mengubah ukuran matriks akselerasi via-point
  base_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

  // Mengisi matriks pose dengan nilai nol
  base_module_state_->joint_via_pose_.fill(0.0);
  // Mengisi matriks kecepatan dengan nilai nol
  base_module_state_->joint_via_dpose_.fill(0.0);
  // Mengisi matriks akselerasi dengan nilai nol
  base_module_state_->joint_via_ddpose_.fill(0.0);

  // Mendapatkan node via_pose dari YAML
  YAML::Node via_pose_node = doc["via_pose"];
  // Iterasi setiap entry dalam via_pose
  for (YAML::iterator yaml_it = via_pose_node.begin(); yaml_it != via_pose_node.end(); ++yaml_it)
  {
    int id;
    std::vector<double> value;

    // Mendapatkan ID joint
    id = yaml_it->first.as<int>();
    // Mendapatkan nilai pose untuk setiap via-point
    value = yaml_it->second.as<std::vector<double> >();

    // Menyimpan pose via-point (konversi dari derajat ke radian)
    for (int num = 0; num < via_num; num++)
      base_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
  }

  // Mendapatkan node tar_pose (target pose) dari YAML
  YAML::Node tar_pose_node = doc["tar_pose"];
  // Iterasi setiap entry dalam tar_pose
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    int id;
    double value;

    // Mendapatkan ID joint
    id = yaml_it->first.as<int>();
    // Mendapatkan nilai pose target
    value = yaml_it->second.as<double>();

    // Menyimpan pose target (konversi dari derajat ke radian)
    base_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
  }

  // Menghitung total langkah waktu berdasarkan waktu pergerakan dan waktu sampling
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;
  // Mengubah ukuran matriks trajectory sesuai total langkah
  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

// Thread untuk menangani callback ROS dan subscription
void BaseModule::queueThread()
{
  // Membuat executor single-threaded untuk memproses callback
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  // Menambahkan node ini ke executor
  executor.add_node(this->get_node_base_interface());

  // Membuat subscription untuk menerima perintah initial pose
  auto ini_pose_msg_sub = this->create_subscription<std_msgs::msg::String>("/robotis/base/ini_pose", 5, 
                                    std::bind(&BaseModule::initPoseMsgCallback, this, std::placeholders::_1));
  // Membuat client untuk service pengaturan modul
  set_module_client_ = this->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");

  // Menghitung rate loop berdasarkan waktu siklus kontrol
  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  // Loop utama untuk memproses callback ROS
  while (rclcpp::ok())
  {
    // Memproses callback yang tersedia
    executor.spin_some();
    // Menunggu hingga waktu siklus berikutnya
    rate.sleep();
  }
}

// Callback saat menerima pesan initial pose dari topic ROS
void BaseModule::initPoseMsgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Cek apakah robot sedang tidak bergerak
  if (base_module_state_->is_moving_ == false)
  {
    // Cek apakah pesan adalah perintah "ini_pose"
    if (msg->data == "ini_pose")
    {
      // Mengatur modul semua joint ke modul ini
      callServiceSettingModule(module_name_);

      // Menunggu hingga modul aktif dan goal joint tersedia
      rclcpp::Rate wait_rate(125);
      while (enable_ == false || has_goal_joints_ == false)
        wait_rate.sleep();

      // Memparse data pose awal dari file konfigurasi
      parseInitPoseData(init_pose_file_path_);

      // Membuat thread untuk generate trajectory
      // Menunggu thread sebelumnya selesai jika masih berjalan
      if (tra_gene_thread_ && tra_gene_thread_->joinable())
        tra_gene_thread_->join();
      // Membuat thread baru untuk generate trajectory pose awal
      tra_gene_thread_ = std::make_unique<std::thread>(&BaseModule::initPoseTrajGenerateProc, this);
    }
  }
  else
    // Menampilkan info jika task sebelumnya masih berjalan
    RCLCPP_INFO(this->get_logger(), "previous task is alive");

  return;
}

// Prosedur untuk generate trajectory pose awal menggunakan minimum jerk
void BaseModule::initPoseTrajGenerateProc()
{
  // Iterasi setiap joint untuk menghitung trajectory
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    // Mendapatkan posisi awal dari goal state saat ini
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    // Mendapatkan posisi target dari file konfigurasi
    double tar_value = base_module_state_->joint_ini_pose_.coeff(id, 0);

    // Matriks untuk menyimpan hasil trajectory
    Eigen::MatrixXd tra;

    // Jika tidak ada via-point, gunakan trajectory langsung
    if (base_module_state_->via_num_ == 0)
    {
      // Menghitung trajectory minimum jerk dari posisi awal ke target
      // Parameter: posisi_awal, kecepatan_awal, akselerasi_awal,
      //            posisi_target, kecepatan_target, akselerasi_target,
      //            waktu_sampling, waktu_total
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  base_module_state_->smp_time_, base_module_state_->mov_time_);
    }
    else
    {
      // Jika ada via-point, gunakan trajectory dengan via-point
      // Mengambil pose via-point untuk joint ini
      Eigen::MatrixXd via_value = base_module_state_->joint_via_pose_.col(id);
      // Mengambil kecepatan via-point untuk joint ini
      Eigen::MatrixXd d_via_value = base_module_state_->joint_via_dpose_.col(id);
      // Mengambil akselerasi via-point untuk joint ini
      Eigen::MatrixXd dd_via_value = base_module_state_->joint_via_ddpose_.col(id);

      // Menghitung trajectory minimum jerk dengan via-point
      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(base_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value, 0.0,
                                                               0.0, base_module_state_->smp_time_,
                                                               base_module_state_->via_time_,
                                                               base_module_state_->mov_time_);
    }

    // Menyimpan trajectory ke matriks calc_joint_tra_ pada kolom sesuai ID joint
    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  // Mengaktifkan flag bahwa robot sedang bergerak
  base_module_state_->is_moving_ = true;
  // Mereset counter langkah trajectory
  base_module_state_->cnt_ = 0;
  // Menampilkan info bahwa trajectory mulai dikirim
  RCLCPP_INFO(this->get_logger(), "[start] send trajectory");
}

// Prosedur untuk generate trajectory pose dari matriks sudut joint (overload 1)
void BaseModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  // Memanggil service untuk mengatur modul kontrol
  callServiceSettingModule(module_name_);

  // Menunggu hingga modul aktif dan goal joint tersedia
  while (enable_ == false || has_goal_joints_ == false)
    std::this_thread::sleep_for(std::chrono::milliseconds(8));

  // Mengatur waktu pergerakan menjadi 5 detik
  base_module_state_->mov_time_ = 5.0;
  // Menghitung total langkah waktu berdasarkan waktu pergerakan dan sampling
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  // Mengubah ukuran matriks trajectory sesuai total langkah
  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  // Menyimpan pose target dari parameter
  base_module_state_->joint_pose_ = joint_angle_pose;

  // Iterasi setiap joint untuk menghitung trajectory
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    // Mendapatkan posisi awal dari goal state saat ini
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    // Mendapatkan posisi target dari matriks pose
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    // Menampilkan info posisi awal dan target untuk debugging
    RCLCPP_INFO(this->get_logger(), "[ID : %d] ini_value : %f  tar_value : %f", id, ini_value, tar_value);

    // Menghitung trajectory minimum jerk dari posisi awal ke target
    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_,
                                                                base_module_state_->mov_time_);

    // Menyimpan trajectory ke matriks calc_joint_tra_
    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  // Mengaktifkan flag bahwa robot sedang bergerak
  base_module_state_->is_moving_ = true;
  // Mereset counter langkah trajectory
  base_module_state_->cnt_ = 0;
  // Mengaktifkan flag untuk mode initial pose saja
  ini_pose_only_ = true;
  // Menampilkan info bahwa trajectory mulai dikirim
  RCLCPP_INFO(this->get_logger(), "[start] send trajectory");
}

// Prosedur untuk generate trajectory pose dari map nama joint ke sudut (overload 2)
void BaseModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
{
  // Memanggil service untuk mengatur modul kontrol
  callServiceSettingModule(module_name_);

  // Menunggu hingga modul aktif dan goal joint tersedia
  while (enable_ == false || has_goal_joints_ == false)
    std::this_thread::sleep_for(std::chrono::milliseconds(8));

  // Membuat matriks target pose dengan nilai nol
  Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  // Mengkonversi map nama joint ke matriks pose berdasarkan ID
  for (const auto& [joint_name, joint_angle_rad] : joint_angle_pose)
  {
    // Mencari ID joint dari nama joint
    auto it = joint_name_to_id_.find(joint_name);
    if (it != joint_name_to_id_.end())
    {
      // Menyimpan sudut joint ke matriks target_pose
      target_pose.coeffRef(it->second, 0) = joint_angle_rad;
    }
  }

  // Menyimpan target pose ke state modul
  base_module_state_->joint_pose_ = target_pose;

  // Mengatur waktu pergerakan menjadi 5 detik
  base_module_state_->mov_time_ = 5.0;
  // Menghitung total langkah waktu
  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

  // Mengubah ukuran matriks trajectory
  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  // Iterasi setiap joint untuk menghitung trajectory
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    // Mendapatkan posisi awal dari goal state saat ini
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    // Mendapatkan posisi target dari matriks pose
    double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

    // Menampilkan info posisi awal dan target untuk debugging
    RCLCPP_INFO(this->get_logger(), "[ID : %d] ini_value : %f  tar_value : %f", id, ini_value, tar_value);

    // Menghitung trajectory minimum jerk
    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                base_module_state_->smp_time_,
                                                                base_module_state_->mov_time_);

    // Menyimpan trajectory ke matriks calc_joint_tra_
    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  // Mengaktifkan flag bahwa robot sedang bergerak
  base_module_state_->is_moving_ = true;
  // Mereset counter langkah trajectory
  base_module_state_->cnt_ = 0;
  // Mengaktifkan flag untuk mode initial pose saja
  ini_pose_only_ = true;
  // Menampilkan info bahwa trajectory mulai dikirim
  RCLCPP_INFO(this->get_logger(), "[start] send trajectory");
}

// Fungsi untuk mengecek apakah modul sedang menjalankan trajectory
bool BaseModule::isRunning()
{
  // Mengembalikan status apakah robot sedang bergerak
  return base_module_state_->is_moving_;
}

// Fungsi utama proses yang dipanggil setiap siklus kontrol
// Parameter: dxls = map Dynamixel, sensors = map sensor (tidak digunakan)
void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  // Jika modul tidak aktif, langsung return
  if (enable_ == false)
    return;

  // Membaca posisi saat ini dari semua Dynamixel
  for (const auto& [joint_name, dxl] : dxls)
  {
    // Skip jika joint tidak ada di result map
    if (result_.find(joint_name) == result_.end())
      continue;

    // Membaca posisi present (saat ini) dari Dynamixel
    double joint_curr_position = dxl->dxl_state_->present_position_;
    // Membaca posisi goal dari Dynamixel
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    // Menyimpan posisi saat ini ke joint state
    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    // Menyimpan posisi goal ke joint state
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
  }

  // Menandai bahwa goal joint sudah tersedia
  has_goal_joints_ = true;

  // Mengirim trajectory jika robot sedang bergerak
  if (base_module_state_->is_moving_ == true)
  {
    // Mengirim status "Start Init Pose" pada langkah pertama
    if (base_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Start Init Pose");

    // Mengupdate goal position setiap joint dari trajectory yang sudah dihitung
    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = base_module_state_->calc_joint_tra_(base_module_state_->cnt_, id);

    // Increment counter langkah trajectory
    base_module_state_->cnt_++;
  }

  // Mengatur data joint ke result untuk dikirim ke controller
  for (const auto& [joint_name, state] : result_)
  {
    // Menyalin goal position dari joint state ke result
    state->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
  }

  // Mengecek apakah trajectory sudah selesai
  if ((base_module_state_->cnt_ >= base_module_state_->all_time_steps_) && (base_module_state_->is_moving_ == true))
  {
    // Menampilkan info bahwa trajectory selesai
    RCLCPP_INFO(this->get_logger(), "[end] send trajectory");

    // Mengirim status "Finish Init Pose"
    publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Finish Init Pose");

    // Menonaktifkan flag pergerakan
    base_module_state_->is_moving_ = false;
    // Mereset counter langkah
    base_module_state_->cnt_ = 0;

    // Jika mode initial pose saja, set modul ke "none"
    if (ini_pose_only_ == true)
    {
      // Menonaktifkan kontrol modul
      setCtrlModule("none");
      // Mereset flag mode initial pose
      ini_pose_only_ = false;
    }
  }
}

// Fungsi untuk menghentikan modul (tidak ada implementasi khusus)
void BaseModule::stop()
{
  return;
}

// Callback saat modul diaktifkan
void BaseModule::onModuleEnable()
{
  // Menampilkan info bahwa modul telah diaktifkan
  RCLCPP_INFO(this->get_logger(), "Base Module is enabled");
}

// Callback saat modul dinonaktifkan
void BaseModule::onModuleDisable()
{
  // Mereset flag goal joint
  has_goal_joints_ = false;
}

// Fungsi untuk mengatur modul kontrol melalui publisher
void BaseModule::setCtrlModule(std::string module)
{
  // Membuat pesan String untuk dikirim
  std_msgs::msg::String control_msg;
  // Mengisi data dengan nama modul
  control_msg.data = module_name_;

  // Mempublish pesan ke topic
  set_ctrl_module_pub_->publish(control_msg);
}

// Fungsi untuk memanggil service pengaturan modul
void BaseModule::callServiceSettingModule(const std::string &module_name)
{
  // Membuat request untuk service SetModule
  auto request = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
  // Mengisi nama modul yang diinginkan
  request->module_name = module_name;

  // Menunggu service tersedia (timeout 1 detik)
  if (!set_module_client_->wait_for_service(std::chrono::seconds(1)))
  {
    // Menampilkan error jika service tidak tersedia
    RCLCPP_ERROR(this->get_logger(), "Service not available");
    return;
  }

  // Mengirim request secara asynchronous dengan callback
  auto future = set_module_client_->async_send_request(request,
      [this](rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedFuture result)
      {
        // Menampilkan hasil dari service call
        RCLCPP_INFO(this->get_logger(), "callServiceSettingModule : result : %d", result.get()->result);
      });
}

// Fungsi untuk mempublish pesan status
void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  // Membuat objek pesan StatusMsg
  auto status_msg = robotis_controller_msgs::msg::StatusMsg();
  // Mengisi timestamp dengan waktu saat ini
  status_msg.header.stamp = rclcpp::Clock().now();
  // Mengisi tipe status (INFO, WARNING, ERROR, dll)
  status_msg.type = type;
  // Mengisi nama modul pengirim
  status_msg.module_name = "Base";
  // Mengisi pesan status
  status_msg.status_msg = msg;

  // Mempublish pesan status ke topic
  status_msg_pub_->publish(status_msg);
}
}
