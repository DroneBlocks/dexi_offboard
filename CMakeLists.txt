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
find_package(apriltag_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# PX4 Offboard Manager
add_executable(px4_offboard_manager src/px4_offboard_manager.cpp)
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

# AprilTag Follower
add_executable(apriltag_follower src/apriltag_follower.cpp)
target_include_directories(apriltag_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(apriltag_follower PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  apriltag_follower
  "rclcpp"
  "px4_msgs"
  "apriltag_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2"
)

# AprilTag Visualizer
add_executable(apriltag_visualizer src/apriltag_visualizer.cpp)
target_include_directories(apriltag_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(apriltag_visualizer PUBLIC c_std_99 cxx_std_17)
target_link_libraries(apriltag_visualizer ${OpenCV_LIBS})
ament_target_dependencies(
  apriltag_visualizer
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "apriltag_msgs"
  "cv_bridge"
  "image_transport"
)

install(TARGETS px4_offboard_manager px4_pose_publisher keyboard_teleop apriltag_follower apriltag_visualizer
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