cmake_minimum_required(VERSION 3.8)
project(imu_and_image_to_rosbag2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rcutils REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 1. 编译可执行文件
add_executable(imu_and_image_to_rosbag2_tool src/imu_and_image_to_rosbag2.cc)
# 2. 链接所有依赖库
ament_target_dependencies(imu_and_image_to_rosbag2_tool rclcpp sensor_msgs rosbag2_cpp rosbag2_storage rcutils OpenCV cv_bridge)
# 3. 安装可执行文件（让 ros2 run 能找到）
install(TARGETS imu_and_image_to_rosbag2_tool DESTINATION lib/${PROJECT_NAME})

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
