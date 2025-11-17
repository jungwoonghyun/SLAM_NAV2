cmake_minimum_required(VERSION 3.8)
project(safeway_robot_controll_packet_rs485)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_interface REQUIRED)

# Build
add_executable(packet_rs485_publisher src/packet_rs485_publisher.cpp src/keyboard_setting_function.cpp)
include_directories(include/)
ament_target_dependencies(packet_rs485_publisher rclcpp std_msgs geometry_msgs)

add_executable(packet_rs485_publisher_rotate src/packet_rs485_publisher_rotate.cpp src/keyboard_setting_function.cpp)
include_directories(include/)
ament_target_dependencies(packet_rs485_publisher_rotate rclcpp nav_msgs std_msgs geometry_msgs message_interface)

add_executable(packet_rs485_subscriber_wagon src/packet_rs485_subscriber_wagon.cpp src/rs485_comm.cpp src/rs485_recv.cpp src/rs485_send.cpp src/rs485_crc16.cpp)
include_directories(include/)
ament_target_dependencies(packet_rs485_subscriber_wagon rclcpp std_msgs geometry_msgs message_interface)

add_executable(packet_rs485_subscriber_robot src/packet_rs485_subscriber_robot.cpp src/rs485_comm.cpp src/rs485_recv.cpp src/rs485_send.cpp src/rs485_crc16.cpp)
include_directories(include/)
ament_target_dependencies(packet_rs485_subscriber_robot rclcpp std_msgs geometry_msgs message_interface)

add_executable(packet_rs485_subscriber_robot_mod src/packet_rs485_subscriber_robot_mod.cpp src/rs485_comm.cpp src/rs485_recv.cpp src/rs485_send.cpp src/rs485_crc16.cpp)
include_directories(include/)
ament_target_dependencies(packet_rs485_subscriber_robot_mod rclcpp std_msgs geometry_msgs message_interface)

add_executable(esp32_robot_light src/packet_esp32_robot_light.cpp src/esp32_comm.cpp)
include_directories(include/)
ament_target_dependencies(esp32_robot_light rclcpp std_msgs message_interface)

add_executable(esp32_robot_switch_8bit src/packet_esp32_robot_switch_8bit.cpp src/esp32_comm.cpp)
include_directories(include/)
ament_target_dependencies(esp32_robot_switch_8bit rclcpp std_msgs message_interface)

add_executable(esp32_robot_light_8bit src/packet_esp32_robot_light_8bit.cpp src/esp32_comm.cpp)
include_directories(include/)
ament_target_dependencies(esp32_robot_light_8bit rclcpp std_msgs message_interface)

# add_executable(safeway_robot_controller_base src/safeway_robot_controller_base.cpp src/rs485_comm.cpp src/rs485_recv.cpp src/rs485_send.cpp src/rs485_crc16.cpp)
# include_directories(include/)
# ament_target_dependencies(safeway_robot_controller_base rclcpp tf2 tf2_ros nav_msgs std_msgs geometry_msgs sensor_msgs message_interface)

# add_executable(safeway_wheelchair_controller_base src/safeway_wheelchair_controller_base.cpp src/rs485_comm.cpp src/rs485_recv.cpp src/rs485_send.cpp src/rs485_crc16.cpp)
# include_directories(include/)
# ament_target_dependencies(safeway_wheelchair_controller_base rclcpp std_msgs geometry_msgs message_interface)

#add_executable(packet_uwb_dwm1001_publisher src/packet_uwb_dwm1001_publisher.cpp src/uwb_setting.cpp)
#include_directories(include/)
#ament_target_dependencies(packet_uwb_dwm1001_publisher rclcpp std_msgs geometry_msgs)

add_executable(packet_tf2_tracing src/packet_tf2_tracing.cpp)
include_directories(include/)
ament_target_dependencies(packet_tf2_tracing rclcpp std_msgs geometry_msgs tf2 tf2_ros visualization_msgs)

add_executable(esp32_controller src/packet_esp32_controller.cpp src/esp32_comm.cpp)
include_directories(include/)
ament_target_dependencies(esp32_controller rclcpp std_msgs message_interface)

add_executable(esp32_robot_switch src/packet_esp32_robot_switch.cpp src/esp32_comm.cpp)
include_directories(include/)
ament_target_dependencies(esp32_robot_switch rclcpp std_msgs message_interface)

add_executable(esp32_softwareexample src/packet_esp32_softwareexample.cpp)
include_directories(include/)
ament_target_dependencies(esp32_softwareexample rclcpp std_msgs message_interface)

# add_executable(imu_controller src/packet_imu_controller.cpp src/imu_comm.cpp)
# include_directories(include/)
# ament_target_dependencies(imu_controller rclcpp std_msgs message_interface)

#add_executable(packet_uwb_dwm1000_publisher src/packet_uwb_dwm1000_publisher.cpp src/dwm1000_setting.cpp)
#include_directories(include/)
#ament_target_dependencies(packet_uwb_dwm1000_publisher rclcpp std_msgs geometry_msgs message_interface visualization_msgs)

#add_executable(packet_rtls_system src/packet_rtls_system.cpp src/dwm1000_setting.cpp)
#include_directories(include/)
#ament_target_dependencies(packet_rtls_system rclcpp std_msgs geometry_msgs message_interface visualization_msgs)

# add_executable(esp32_loadcell_controller src/packet_esp32_loadcell_controller.cpp src/esp32_comm.cpp)
# include_directories(include/)
# ament_target_dependencies(esp32_loadcell_controller rclcpp std_msgs message_interface geometry_msgs)

# add_executable(esp32_distance_sensor src/packet_esp32_distance_sensor.cpp src/esp32_comm.cpp)
# include_directories(include/)
# ament_target_dependencies(esp32_distance_sensor rclcpp std_msgs message_interface geometry_msgs)

# add_executable(packet_rs485_test src/packet_rs485_test.cpp src/rs485_comm.cpp src/rs485_recv.cpp src/rs485_send.cpp src/rs485_crc16.cpp)
# include_directories(include/)
# ament_target_dependencies(packet_rs485_test rclcpp std_msgs)


#target_link_libraries(packet_RS485_publisher PRIVATE m)

# Install
install(TARGETS
  packet_rs485_publisher
  packet_rs485_publisher_rotate
  packet_rs485_subscriber_robot
  packet_rs485_subscriber_wagon
  packet_rs485_subscriber_robot_mod
  # safeway_robot_controller_base
  # safeway_wheelchair_controller_base
  # packet_rs485_test
#  packet_uwb_dwm1001_publisher
  packet_tf2_tracing
  esp32_controller
  esp32_robot_switch
  esp32_robot_switch_8bit
  esp32_robot_light
  esp32_robot_light_8bit
  # imu_controller
  # esp32_loadcell_controller
  # esp32_distance_sensor
#  esp32_softwareexample
#  packet_uwb_dwm1000_publisher
#  packet_rtls_system
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
