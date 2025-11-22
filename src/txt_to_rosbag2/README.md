# txt_to_rosbag2

将IMU数据从txt格式转换为ROS2 rosbag2格式的C++工具。

## 数据格式

输入txt文件格式：
- 第一行：时间戳（微秒）
- 第二行：角速度（rad/s），三个值用空格分隔 [x, y, z]
- 第三行：加速度（m/s²），三个值用空格分隔 [x, y, z]
- 重复以上模式

示例：
```
345499737070
-0.011718 -0.006392 -0.004261
0.129287 -0.201113 9.634268
345499742148
-0.011718 -0.005326 -0.003196
0.129287 -0.201113 9.634268
```

## 依赖

- ROS2 (Humble/Iron/Rolling)
- ament_cmake
- rclcpp
- sensor_msgs
- rosbag2_cpp
- rosbag2_storage

## 编译

```bash
cd /tools_ws
colcon build --packages-select txt_to_rosbag2
```

## 使用方法

```bash
cd /tools_ws
source install.setup.bash
ros2 run txt_to_rosbag2 txt_to_rosbag2_tool input_name.txt output_name
```

示例：

```bash
# 使用colcon编译后
ros2 run txt_to_rosbag2 txt_to_rosbag2_tool imu_data_default.txt default_text

# 或直接运行可执行文件
./build/txt_to_rosbag2 txt_to_rosbag2_tool default_text
```

## 输出

- 生成的rosbag2文件包含 `/imu/data` 话题
- 消息类型：`sensor_msgs/msg/Imu`
- 时间戳基于输入文件中的第一个时间戳进行相对计算

## 验证

可以使用以下命令验证生成的rosbag2：

```bash
ros2 bag info output_bag
ros2 bag play output_bag
```
