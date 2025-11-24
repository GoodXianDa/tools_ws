# image_to_rosbag2

将图像数据从PNG格式转换为ROS2 rosbag2格式的C++工具。支持单目和双目相机数据。

## 数据格式

输入图像文件格式：
- 文件名格式：`left_<timestamp>.png` 或 `right_<timestamp>.png`
- `timestamp`：时间戳（微秒），例如 `175819809`
- 支持的图像格式：PNG
- 支持的图像编码：mono8（单通道）、bgr8（三通道）、bgra8（四通道）

示例文件名：
```
left_175819809.png
right_175819809.png
left_175784317.png
right_175784317.png
```

## 目录结构

工具会自动在包目录下查找以下目录：
- 输入目录：`input_image/<input_subdir>/`
- 输出目录：`output_rosbag2/<output_subdir>/`

## 依赖

- ROS2 (Humble/Iron/Rolling)
- ament_cmake
- rclcpp
- sensor_msgs
- rosbag2_cpp
- rosbag2_storage
- OpenCV
- cv_bridge

## 编译

```bash
cd /tools_ws
colcon build --packages-select image_to_rosbag2
```

## 使用方法

```bash
cd /tools_ws
source install/setup.bash
ros2 run image_to_rosbag2 image_to_rosbag2_tool <input_subdir> <output_subdir>
```

参数说明：
- `input_subdir`：输入图像的子目录名（位于 `input_image/` 下）
- `output_subdir`：输出rosbag2的子目录名（位于 `output_rosbag2/` 下）

示例：

```bash
# 使用colcon编译后
ros2 run image_to_rosbag2 image_to_rosbag2_tool image_data_default output_test

# 或直接运行可执行文件
./build/image_to_rosbag2/image_to_rosbag2_tool image_data_default output_test
```

## 功能特性

- **自动识别单目/双目**：如果同时存在左右目图像，自动创建双目话题；如果只有单目，只创建对应的话题
- **按时间戳排序**：自动按文件名中的时间戳对图像进行排序
- **自动创建输出目录**：如果输出目录不存在，会自动创建

## 输出

生成的rosbag2文件包含以下话题：
- `/left/image_raw` (sensor_msgs/msg/Image) - 左目图像（如果存在）
- `/right/image_raw` (sensor_msgs/msg/Image) - 右目图像（如果存在）

时间戳：
- 从文件名中提取的时间戳（微秒）转换为ROS2时间戳格式

## 验证

可以使用以下命令验证生成的rosbag2：

```bash
ros2 bag info output_rosbag2/<output_subdir>/<output_subdir>.bag
ros2 bag play output_rosbag2/<output_subdir>/<output_subdir>.bag
```

