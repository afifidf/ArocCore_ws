cmake_minimum_required(VERSION 3.8)
project(op3_kick_module)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_kinematics_dynamics REQUIRED)
find_package(op3_balance_control REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(op3_walking_module_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robotis_controller_msgs
  robotis_framework_common
  robotis_math
  op3_kinematics_dynamics
  op3_balance_control
  op3_action_module_msgs
  op3_walking_module_msgs
  ament_index_cpp
)

# Build kick_controller library
add_library(${PROJECT_NAME}_lib SHARED
  src/kick_controller.cpp
  src/kick_trajectory.cpp
  src/kick_planner.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_lib ${YAML_CPP_LIBRARIES})

# Build kick_module node
add_executable(kick_module_node src/kick_module_node.cpp)
target_link_libraries(kick_module_node ${PROJECT_NAME}_lib)
ament_target_dependencies(kick_module_node ${DEPENDENCIES})

# Install
install(TARGETS
  ${PROJECT_NAME}_lib
  kick_module_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
