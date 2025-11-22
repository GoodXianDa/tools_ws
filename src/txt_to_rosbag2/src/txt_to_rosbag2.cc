#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rmw/rmw.h>
#include <rcutils/allocator.h>
#include <rcutils/types/uint8_array.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <vector>
#include <cstring>

namespace fs = std::filesystem;

// 包内的输入输出目录名（固定不变）
const std::string INPUT_DIR_NAME = "input_imu_txt";
const std::string OUTPUT_DIR_NAME = "output_imu_rosbag2";
// 包名（必须和你的包名一致！）
const std::string PACKAGE_NAME = "txt_to_rosbag2";

struct IMUData {
    int64_t timestamp_us;  // 时间戳（微秒）
    double angular_velocity[3];  // 角速度 (rad/s) [x, y, z]
    double linear_acceleration[3];  // 加速度 (m/s²) [x, y, z]
};

class TxtToRosbag2Converter {
public:
    TxtToRosbag2Converter() {
        rclcpp::init(0, nullptr);
        auto node = rclcpp::Node::make_shared("temp_node");
    }

    ~TxtToRosbag2Converter() {
        rclcpp::shutdown();
    }

    // 接收：包绝对路径 + 输入文件名 + 输出bag名
    bool convertFile(const std::string& package_abs_path, const std::string& input_filename, const std::string& output_bag_name) {
        // 精准拼接绝对路径：包路径/input_imu_txt/文件名
        fs::path input_file_full = fs::path(package_abs_path) / INPUT_DIR_NAME / input_filename;
        // 精准拼接绝对路径：包路径/output_imu_rosbag2/bag名
        fs::path output_bag_full = fs::path(package_abs_path) / OUTPUT_DIR_NAME / output_bag_name;

        // 打印路径信息（方便调试）
        std::cout << "=== 路径信息 ===" << std::endl;
        std::cout << "包绝对路径: " << package_abs_path << std::endl;
        std::cout << "实际读取文件: " << input_file_full << std::endl;
        std::cout << "实际输出路径: " << output_bag_full << std::endl;
        std::cout << "================" << std::endl;

        // 读取txt文件
        std::vector<IMUData> imu_data_list;
        if (!readTxtFile(input_file_full.string(), imu_data_list)) {
            std::cerr << "读取txt文件失败: " << input_file_full << std::endl;
            return false;
        }

        std::cout << "成功读取 " << imu_data_list.size() << " 条IMU数据" << std::endl;

        // 写入rosbag2
        if (!writeRosbag2(output_bag_full.string(), imu_data_list)) {
            std::cerr << "写入rosbag2失败: " << output_bag_full << std::endl;
            return false;
        }

        std::cout << "成功转换到: " << output_bag_full << std::endl;
        return true;
    }

private:
    bool readTxtFile(const std::string& filepath, std::vector<IMUData>& imu_data_list) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "错误：无法打开文件！" << std::endl;
            std::cerr << "当前文件路径: " << filepath << std::endl;
            std::cerr << "请检查：1. 文件是否存在 2. 文件名是否正确 3. 权限是否足够" << std::endl;
            return false;
        }

        std::string line;
        IMUData current_data;
        int line_count = 0;

        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::istringstream iss(line);
            
            if (line_count % 3 == 0) {
                if (!(iss >> current_data.timestamp_us)) {
                    std::cerr << "解析时间戳失败，行号: " << line_count + 1 << std::endl;
                    continue;
                }
            } else if (line_count % 3 == 1) {
                if (!(iss >> current_data.angular_velocity[0] 
                      >> current_data.angular_velocity[1] 
                      >> current_data.angular_velocity[2])) {
                    std::cerr << "解析角速度失败，行号: " << line_count + 1 << std::endl;
                    continue;
                }
            } else if (line_count % 3 == 2) {
                if (!(iss >> current_data.linear_acceleration[0] 
                      >> current_data.linear_acceleration[1] 
                      >> current_data.linear_acceleration[2])) {
                    std::cerr << "解析加速度失败，行号: " << line_count + 1 << std::endl;
                    continue;
                }
                imu_data_list.push_back(current_data);
            }

            line_count++;
        }

        file.close();
        return !imu_data_list.empty();
    }

    bool writeRosbag2(const std::string& bag_path, const std::vector<IMUData>& imu_data_list) {
        // 自动创建输出目录（如果不存在）
        fs::path bag_dir(bag_path);
        if (bag_dir.has_parent_path()) {
            if (!fs::exists(bag_dir.parent_path())) {
                fs::create_directories(bag_dir.parent_path());
                std::cout << "已创建输出目录: " << bag_dir.parent_path() << std::endl;
            }
        }

        // 配置rosbag2
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";

        std::string serialization_format = "cdr";
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = serialization_format;
        converter_options.output_serialization_format = serialization_format;

        rosbag2_cpp::writers::SequentialWriter writer;
        writer.open(storage_options, converter_options);

        // 创建话题
        const std::string topic_name = "/imu/data";
        const std::string topic_type = "sensor_msgs/msg/Imu";
        writer.create_topic({topic_name, topic_type, serialization_format, ""});

        // 写入数据
        for (const auto& imu_data : imu_data_list) {
            sensor_msgs::msg::Imu imu_msg;
            
            // 时间戳转换：微秒 -> 纳秒
            int64_t timestamp_ns = imu_data.timestamp_us * 1000LL;
            rclcpp::Time timestamp(timestamp_ns);
            imu_msg.header.stamp = timestamp;
            imu_msg.header.frame_id = "imu_frame";

            // 角速度
            imu_msg.angular_velocity.x = imu_data.angular_velocity[0];
            imu_msg.angular_velocity.y = imu_data.angular_velocity[1];
            imu_msg.angular_velocity.z = imu_data.angular_velocity[2];

            // 加速度
            imu_msg.linear_acceleration.x = imu_data.linear_acceleration[0];
            imu_msg.linear_acceleration.y = imu_data.linear_acceleration[1];
            imu_msg.linear_acceleration.z = imu_data.linear_acceleration[2];

            // 协方差矩阵（未知）
            imu_msg.angular_velocity_covariance[0] = -1.0;
            imu_msg.angular_velocity_covariance[4] = -1.0;
            imu_msg.angular_velocity_covariance[8] = -1.0;
            imu_msg.linear_acceleration_covariance[0] = -1.0;
            imu_msg.linear_acceleration_covariance[4] = -1.0;
            imu_msg.linear_acceleration_covariance[8] = -1.0;

            // 序列化
            rclcpp::SerializedMessage serialized_msg;
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
            serialization.serialize_message(&imu_msg, &serialized_msg);
            
            // 创建bag消息
            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->topic_name = topic_name;
            bag_message->time_stamp = timestamp.nanoseconds();
            
            // 复制序列化数据
            const rcutils_uint8_array_t& rcl_serialized = serialized_msg.get_rcl_serialized_message();
            bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
            *bag_message->serialized_data = rcutils_get_zero_initialized_uint8_array();
            rcutils_allocator_t allocator = rcutils_get_default_allocator();
            rcutils_ret_t ret = rcutils_uint8_array_init(
                bag_message->serialized_data.get(),
                rcl_serialized.buffer_length,
                &allocator
            );
            if (ret != RCUTILS_RET_OK) {
                std::cerr << "分配序列化数据内存失败" << std::endl;
                continue;
            }
            std::memcpy(
                bag_message->serialized_data->buffer,
                rcl_serialized.buffer,
                rcl_serialized.buffer_length
            );
            bag_message->serialized_data->buffer_length = rcl_serialized.buffer_length;
            
            writer.write(bag_message);
        }

        writer.close();
        return true;
    }
};

int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <输入txt文件名> <输出rosbag2名称>" << std::endl;
        std::cerr << "说明: " << std::endl;
        std::cerr << "  - 输入文件必须放在 " << PACKAGE_NAME << "/" << INPUT_DIR_NAME << " 目录下" << std::endl;
        std::cerr << "  - 输出bag会保存到 " << PACKAGE_NAME << "/" << OUTPUT_DIR_NAME << " 目录下" << std::endl;
        std::cerr << "示例: " << argv[0] << " imu_data_default.txt imu_data_rosbag" << std::endl;
        return 1;
    }

    // 1. 获取输入文件名和输出bag名
    std::string input_filename = argv[1];
    std::string output_bag_name = argv[2];

    // 2. 关键：获取 txt_to_rosbag2 包的绝对路径！
    std::string package_abs_path;
    try {
        // 通过 ament_index_cpp 获取包的安装前缀，再推导源码目录（或直接用源码目录路径）
        // 注意：如果是开发模式（colcon build 未安装），直接用 src 下的包路径
        package_abs_path = fs::path(__FILE__).parent_path().parent_path().string();
        // 验证包路径是否正确（检查 input_imu_txt 目录是否存在）
        if (!fs::exists(fs::path(package_abs_path) / INPUT_DIR_NAME)) {
            // 若开发模式失败，尝试从安装目录获取
            package_abs_path = ament_index_cpp::get_package_prefix(PACKAGE_NAME);
            // 若仍失败，提示用户
            if (!fs::exists(fs::path(package_abs_path) / INPUT_DIR_NAME)) {
                throw std::runtime_error("无法找到包目录，请确保包已正确编译");
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "错误：获取包路径失败！" << std::endl;
        std::cerr << "原因：" << e.what() << std::endl;
        return 1;
    }

    // 3. 执行转换
    TxtToRosbag2Converter converter;
    if (converter.convertFile(package_abs_path, input_filename, output_bag_name)) {
        std::cout << "\n转换完成！" << std::endl;
        return 0;
    } else {
        std::cerr << "\n转换失败！" << std::endl;
        return 1;
    }
}