cmake_minimum_required(VERSION 3.8)
project(clustering_cpp)

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
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_interface REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(SFML 2.5 COMPONENTS audio REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Build
# add_executable(clustering_cpp src/clustering_wagon)
# ament_target_dependencies(clustering_cpp
#   rclcpp 
#   std_msgs
#   sensor_msgs
#   visualization_msgs
#   geometry_msgs
#   message_interface
# )
# target_link_libraries(clustering_cpp ${PCL_LIBRARIES})

add_executable(clustering_robot src/clustering_robot.cpp)
include_directories(include/)
ament_target_dependencies(clustering_robot 
rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs message_interface tf2 tf2_ros)
target_link_libraries(clustering_robot ${PCL_LIBRARIES} sfml-audio sfml-system)

add_executable(clustering_wagon src/clustering_wagon.cpp)
include_directories(include/)
ament_target_dependencies(clustering_wagon 
rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs message_interface)
target_link_libraries(clustering_wagon ${PCL_LIBRARIES} sfml-audio sfml-system)

add_executable(clustering_nav_robot src/clustering_nav_robot.cpp)
include_directories(include/)
ament_target_dependencies(clustering_nav_robot 
rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs message_interface tf2 tf2_ros)
target_link_libraries(clustering_nav_robot ${PCL_LIBRARIES} sfml-audio sfml-system)

# Install
install(TARGETS
  clustering_wagon
  clustering_robot
  clustering_nav_robot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params launch
 DESTINATION share/${PROJECT_NAME})

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

ament_package()
