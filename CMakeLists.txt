cmake_minimum_required(VERSION 3.8)
project(dexi_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(dexi_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# PX4 Offboard Manager
add_executable(px4_offboard_manager src/px4_offboard_manager.cpp src/mission_controller.cpp)
target_include_directories(px4_offboard_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(px4_offboard_manager PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  px4_offboard_manager
  "rclcpp"
  "px4_msgs"
  "dexi_interfaces"
)

# PX4 Pose Publisher
add_executable(px4_pose_publisher src/px4_pose_publisher.cpp)
target_include_directories(px4_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(px4_pose_publisher PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  px4_pose_publisher
  "rclcpp"
  "px4_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2"
)

# Keyboard Teleop
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_include_directories(keyboard_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(keyboard_teleop PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  keyboard_teleop
  "rclcpp"
  "dexi_interfaces"
)

install(TARGETS px4_offboard_manager px4_pose_publisher keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()