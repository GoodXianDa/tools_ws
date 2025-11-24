# tools_ws

ROS2 数据转换工具集，用于将各种格式的数据转换为 ROS2 rosbag2 格式。

## 工具包

本工作空间包含以下三个工具包：

### 1. txt_to_rosbag2
将 IMU 数据从 txt 格式转换为 ROS2 rosbag2 格式。

**功能**：
- 读取 IMU txt 文件（时间戳、角速度、加速度）
- 转换为 `sensor_msgs/msg/Imu` 消息
- 输出到 rosbag2 文件

**详细文档**：参见 [src/txt_to_rosbag2/README.md](src/txt_to_rosbag2/README.md)

### 2. image_to_rosbag2
将图像数据从 PNG 格式转换为 ROS2 rosbag2 格式。

**功能**：
- 读取左右目图像文件（`left_<timestamp>.png` / `right_<timestamp>.png`）
- 支持单目/双目自动识别
- 转换为 `sensor_msgs/msg/Image` 消息
- 输出到 rosbag2 文件

**详细文档**：参见 [src/image_to_rosbag2/README.md](src/image_to_rosbag2/README.md)

### 3. imu_and_image_to_rosbag2
将 IMU 和图像数据合并转换为 ROS2 rosbag2 格式。

**功能**：
- 读取 IMU txt 文件和图像文件
- 按时间戳合并排序
- 同时输出 IMU 和图像数据到同一个 rosbag2 文件

**详细文档**：参见 [src/imu_and_image_to_rosbag2/README.md](src/imu_and_image_to_rosbag2/README.md)

## 编译

```bash
cd /home/saylor/workspace/tools_ws
colcon build
```

编译特定包：
```bash
colcon build --packages-select <package_name>
```

## 使用

编译完成后，source 工作空间：
```bash
source install/setup.bash
```

然后使用 `ros2 run` 命令运行工具，例如：
```bash
# 转换 IMU 数据
ros2 run txt_to_rosbag2 txt_to_rosbag2_tool imu_data_default.txt output_name

# 转换图像数据
ros2 run image_to_rosbag2 image_to_rosbag2_tool image_data_default output_test

# 合并转换 IMU 和图像数据
ros2 run imu_and_image_to_rosbag2 imu_and_image_to_rosbag2_tool imu_test_1 stereo_test_1 output_test
```

## ROS2 rosbag2 转换为 ROS1 rosbag

如果需要将 ROS2 的 rosbag2 格式转换为 ROS1 的 rosbag 格式，可以使用 `rosbags` 工具。

### 安装 rosbags

```bash
pip3 install rosbags
```

### 转换方法

rosbag2 文件通常是目录格式（包含 `.db3` 文件和 `metadata.yaml`），转换时可以直接指定目录或 `.db3` 文件：

**方法1：指定 rosbag2 目录**
```bash
rosbags-convert --src ./output_data/test_1.bag --dst /data/test_ros1.bag --dst-typestore ros1_noetic
```

**方法2：指定 .db3 文件**
```bash
rosbags-convert --src ./output_data/test_1.bag/test_1.bag_0.db3 --dst /data/test_ros1.bag --dst-typestore ros1_noetic
```

**参数说明**：
- `--src`：源 rosbag2 文件路径（目录或 .db3 文件）
- `--dst`：目标 ROS1 rosbag 文件路径
- `--dst-typestore`：目标类型存储格式，常用值：
  - `ros1_noetic`：ROS1 Noetic
  - `ros1_melodic`：ROS1 Melodic
  - `ros1_kinetic`：ROS1 Kinetic

**示例**：
```bash
# 转换 IMU rosbag2 到 ROS1
rosbags-convert --src ./src/txt_to_rosbag2/output_imu_rosbag2/test/test_0.db3 --dst /data/imu_ros1.bag --dst-typestore ros1_noetic

# 转换合并的 IMU+图像 rosbag2 到 ROS1
rosbags-convert --src ./src/imu_and_image_to_rosbag2/output_data/test_1.bag --dst /data/imu_image_ros1.bag --dst-typestore ros1_noetic
```

## 依赖

- ROS2 (Humble/Iron/Rolling)
- ament_cmake
- rclcpp
- sensor_msgs
- rosbag2_cpp
- rosbag2_storage
- OpenCV
- cv_bridge
- Python 3（用于 rosbags 工具）

## 许可证

各包遵循 Apache-2.0 许可证。

