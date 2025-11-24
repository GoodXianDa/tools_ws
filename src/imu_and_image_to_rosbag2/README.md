# imu_and_image_to_rosbag2

将IMU数据和图像数据合并转换为ROS2 rosbag2格式的C++工具。支持将IMU和双目/单目相机数据按时间戳合并到同一个rosbag2文件中。

## 数据格式

### IMU数据格式

输入txt文件格式（与 `txt_to_rosbag2` 相同）：
- 第一行：时间戳（微秒）
- 第二行：角速度（rad/s），三个值用空格分隔 [x, y, z]
- 第三行：加速度（m/s²），三个值用空格分隔 [x, y, z]
- 重复以上模式

示例：
```
116281007
0.021305 -0.026632 0.172573
1.039083 -0.804452 9.586384
116291082
-0.031958 -0.028762 0.191747
0.976834 -0.919373 9.562442
```

### 图像数据格式

输入图像文件格式（与 `image_to_rosbag2` 相同）：
- 文件名格式：`left_<timestamp>.png` 或 `right_<timestamp>.png`
- `timestamp`：时间戳（微秒），例如 `175819809`
- 支持的图像格式：PNG
- 支持的图像编码：mono8（单通道）、bgr8（三通道）、bgra8（四通道）

示例文件名：
```
left_175819809.png
right_175819809.png
```

## 目录结构

工具会自动在包目录下查找以下目录：
- IMU输入目录：`input_data/<imu_dir>/`（自动查找其中的txt文件）
- 图像输入目录：`input_data/<image_dir>/`
- 输出目录：`output_data/`

## 依赖

- ROS2 (Humble/Iron/Rolling)
- ament_cmake
- rclcpp
- sensor_msgs
- rosbag2_cpp
- rosbag2_storage
- rcutils
- OpenCV
- cv_bridge

## 编译

```bash
cd /tools_ws
colcon build --packages-select imu_and_image_to_rosbag2
```

## 使用方法

```bash
cd /tools_ws
source install/setup.bash
ros2 run imu_and_image_to_rosbag2 imu_and_image_to_rosbag2_tool <imu_dir> <image_dir> <output_name>
```

参数说明：
- `imu_dir`：IMU数据的子目录名（位于 `input_data/` 下，工具会自动查找其中的txt文件）
- `image_dir`：图像数据的子目录名（位于 `input_data/` 下）
- `output_name`：输出rosbag2文件名（不含.bag扩展名，位于 `output_data/` 下）

示例：

```bash
# 使用colcon编译后
ros2 run imu_and_image_to_rosbag2 imu_and_image_to_rosbag2_tool imu_test_1 stereo_test_1 output_test

# 或直接运行可执行文件
./build/imu_and_image_to_rosbag2/imu_and_image_to_rosbag2_tool imu_test_1 stereo_test_1 output_test
```

## 功能特性

- **时间戳合并排序**：自动将所有IMU和图像数据按时间戳（微秒）排序，确保数据按时间顺序写入rosbag2
- **自动识别单目/双目**：如果同时存在左右目图像，自动创建双目话题；如果只有单目，只创建对应的话题
- **自动查找IMU文件**：自动在IMU目录中查找txt文件（通常只有一个）
- **自动创建输出目录**：如果输出目录不存在，会自动创建

## 输出

生成的rosbag2文件包含以下话题：
- `/imu/data` (sensor_msgs/msg/Imu) - IMU数据
- `/left/image_raw` (sensor_msgs/msg/Image) - 左目图像（如果存在）
- `/right/image_raw` (sensor_msgs/msg/Image) - 右目图像（如果存在）

时间戳：
- IMU和图像的时间戳均为微秒单位
- 自动转换为ROS2时间戳格式（纳秒）
- 所有数据按时间戳顺序写入，确保时间同步

## 验证

可以使用以下命令验证生成的rosbag2：

```bash
ros2 bag info output_data/<output_name>.bag
ros2 bag play output_data/<output_name>.bag
```

## 相关工具

- `txt_to_rosbag2`：仅转换IMU数据
- `image_to_rosbag2`：仅转换图像数据
- `imu_and_image_to_rosbag2`：合并转换IMU和图像数据（本工具）

