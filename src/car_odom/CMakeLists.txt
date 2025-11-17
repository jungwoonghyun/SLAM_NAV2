cmake_minimum_required(VERSION 3.8)
project(car_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp  REQUIRED)
find_package(rclcpp_action  REQUIRED)
find_package(message_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(car_odom src/car_odom.cpp)
ament_target_dependencies(car_odom rclcpp tf2 tf2_ros nav_msgs message_interface geometry_msgs)

add_executable(basefootprint_listner src/basefootprint_listner.cpp)
ament_target_dependencies(basefootprint_listner rclcpp tf2 tf2_ros message_interface geometry_msgs)

add_executable(navigation_cancel_goal src/nav2_robot.cpp)
ament_target_dependencies(navigation_cancel_goal rclcpp tf2 tf2_ros rclcpp_action nav2_msgs message_interface geometry_msgs)

install(TARGETS
  car_odom
  basefootprint_listner
  navigation_cancel_goal
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
