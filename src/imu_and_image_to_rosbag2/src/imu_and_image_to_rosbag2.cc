#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <regex>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rcutils/allocator.h>
#include <rcutils/types/uint8_array.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>
#include <cstring>
#include <memory>

namespace fs = std::filesystem;
using namespace std;

// 包内的输入输出目录名（固定不变）
const string INPUT_ROOT = "input_data";
const string OUTPUT_ROOT = "output_data";

// IMU数据结构
struct IMUData {
    int64_t timestamp_us;  // 时间戳（微秒）
    double angular_velocity[3];  // 角速度 (rad/s) [x, y, z]
    double linear_acceleration[3];  // 加速度 (m/s²) [x, y, z]
};

// 图像文件结构体
struct ImageFile {
    string path;
    int64_t timestamp_us;
    string type; // left/right
};

// 统一的消息结构（用于时间戳排序）
struct TimestampedMessage {
    int64_t timestamp_us;
    enum Type { IMU, IMAGE_LEFT, IMAGE_RIGHT };
    Type type;
    IMUData imu_data;
    ImageFile image_file;
};

// 从IMU目录中找到txt文件
string findImuTxtFile(const string& imu_dir) {
    if (!fs::exists(imu_dir) || !fs::is_directory(imu_dir)) {
        return "";
    }
    
    for (const auto& entry : fs::directory_iterator(imu_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            return entry.path().string();
        }
    }
    return "";
}

// 读取IMU txt文件
bool readImuTxtFile(const string& filepath, vector<IMUData>& imu_data_list) {
    ifstream file(filepath);
    if (!file.is_open()) {
        cerr << "错误：无法打开IMU文件：" << filepath << endl;
        return false;
    }

    string line;
    IMUData current_data;
    int line_count = 0;

    while (getline(file, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        
        if (line_count % 3 == 0) {
            if (!(iss >> current_data.timestamp_us)) {
                cerr << "解析时间戳失败，行号: " << line_count + 1 << endl;
                continue;
            }
        } else if (line_count % 3 == 1) {
            if (!(iss >> current_data.angular_velocity[0] 
                  >> current_data.angular_velocity[1] 
                  >> current_data.angular_velocity[2])) {
                cerr << "解析角速度失败，行号: " << line_count + 1 << endl;
                continue;
            }
        } else if (line_count % 3 == 2) {
            if (!(iss >> current_data.linear_acceleration[0] 
                  >> current_data.linear_acceleration[1] 
                  >> current_data.linear_acceleration[2])) {
                cerr << "解析加速度失败，行号: " << line_count + 1 << endl;
                continue;
            }
            imu_data_list.push_back(current_data);
        }

        line_count++;
    }

    file.close();
    return !imu_data_list.empty();
}

// 合并时间戳和类型提取
pair<int64_t, string> extractTsAndType(const string& filename) {
    regex pattern(R"((left|right)_(\d+)\.png)");
    smatch matches;
    if (regex_search(filename, matches, pattern) && matches.size() >= 3) {
        return {stoll(matches[2].str()), matches[1].str()};
    }
    return {0, ""};
}

// 读取指定目录下的有效图片
vector<ImageFile> getImageFiles(const string& input_dir) {
    vector<ImageFile> files;
    if (!fs::exists(input_dir) || !fs::is_directory(input_dir)) {
        cerr << "错误：输入目录不存在 -> " << input_dir << endl;
        return files;
    }

    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (entry.is_regular_file()) {
            auto [ts, type] = extractTsAndType(entry.path().filename().string());
            if (ts > 0 && !type.empty()) {
                files.push_back({entry.path().string(), ts, type});
            }
        }
    }

    // 按时间戳排序
    sort(files.begin(), files.end(), [](const ImageFile& a, const ImageFile& b) {
        return a.timestamp_us < b.timestamp_us;
    });
    return files;
}

// OpenCV图像转ROS消息
sensor_msgs::msg::Image cvToRosImg(const cv::Mat& cv_img, const std_msgs::msg::Header& header) {
    sensor_msgs::msg::Image ros_img;
    ros_img.header = header;
    ros_img.height = cv_img.rows;
    ros_img.width = cv_img.cols;
    
    if (cv_img.channels() == 1) {
        ros_img.encoding = "mono8";
        ros_img.is_bigendian = false;
        ros_img.step = cv_img.cols;
    } else if (cv_img.channels() == 3) {
        ros_img.encoding = "bgr8";
        ros_img.is_bigendian = false;
        ros_img.step = cv_img.cols * 3;
    } else if (cv_img.channels() == 4) {
        ros_img.encoding = "bgra8";
        ros_img.is_bigendian = false;
        ros_img.step = cv_img.cols * 4;
    } else {
        cerr << "不支持的图像通道数: " << cv_img.channels() << endl;
        return ros_img;
    }
    
    ros_img.data.resize(ros_img.step * ros_img.height);
    memcpy(ros_img.data.data(), cv_img.data, ros_img.data.size());
    return ros_img;
}

// 序列化IMU消息
shared_ptr<rosbag2_storage::SerializedBagMessage> serializeImu(
    const sensor_msgs::msg::Imu& imu_msg, const string& topic, rclcpp::Time ts) {
    
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    auto serialized_msg = make_shared<rosbag2_storage::SerializedBagMessage>();
    serialized_msg->topic_name = topic;
    serialized_msg->time_stamp = ts.nanoseconds();

    rclcpp::SerializedMessage ser_data;
    serializer.serialize_message(&imu_msg, &ser_data);
    auto rcl_msg = ser_data.get_rcl_serialized_message();

    serialized_msg->serialized_data = make_shared<rcutils_uint8_array_t>();
    serialized_msg->serialized_data->buffer = static_cast<uint8_t*>(malloc(rcl_msg.buffer_length));
    if (!serialized_msg->serialized_data->buffer) throw runtime_error("内存分配失败");
    
    memcpy(serialized_msg->serialized_data->buffer, rcl_msg.buffer, rcl_msg.buffer_length);
    serialized_msg->serialized_data->buffer_length = rcl_msg.buffer_length;
    serialized_msg->serialized_data->buffer_capacity = rcl_msg.buffer_length;
    serialized_msg->serialized_data->allocator = rcl_get_default_allocator();

    return serialized_msg;
}

// 序列化图像消息
shared_ptr<rosbag2_storage::SerializedBagMessage> serializeImg(
    const sensor_msgs::msg::Image& ros_img, const string& topic, rclcpp::Time ts) {
    
    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    auto serialized_msg = make_shared<rosbag2_storage::SerializedBagMessage>();
    serialized_msg->topic_name = topic;
    serialized_msg->time_stamp = ts.nanoseconds();

    rclcpp::SerializedMessage ser_data;
    serializer.serialize_message(&ros_img, &ser_data);
    auto rcl_msg = ser_data.get_rcl_serialized_message();

    serialized_msg->serialized_data = make_shared<rcutils_uint8_array_t>();
    serialized_msg->serialized_data->buffer = static_cast<uint8_t*>(malloc(rcl_msg.buffer_length));
    if (!serialized_msg->serialized_data->buffer) throw runtime_error("内存分配失败");
    
    memcpy(serialized_msg->serialized_data->buffer, rcl_msg.buffer, rcl_msg.buffer_length);
    serialized_msg->serialized_data->buffer_length = rcl_msg.buffer_length;
    serialized_msg->serialized_data->buffer_capacity = rcl_msg.buffer_length;
    serialized_msg->serialized_data->allocator = rcl_get_default_allocator();

    return serialized_msg;
}

int main(int argc, char** argv) {
    // 1. 初始化ROS节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_and_image_to_rosbag2");

    // 2. 解析命令行参数（必须传入3个参数：IMU目录、图像目录、输出名字）
    if (argc != 4) {
        cerr << "用法错误！正确格式：" << endl;
        cerr << "ros2 run imu_and_image_to_rosbag2 imu_and_image_to_rosbag2 <imu_dir> <image_dir> <output_name>" << endl;
        cerr << "示例: ros2 run imu_and_image_to_rosbag2 imu_and_image_to_rosbag2 imu_test_1 stereo_test_1 output_test" << endl;
        rclcpp::shutdown();
        return 1;
    }

    // 3. 获取包路径
    string package_abs_path = fs::path(__FILE__).parent_path().parent_path().string();
    if (!fs::exists(fs::path(package_abs_path) / INPUT_ROOT)) {
        cerr << "错误：无法找到包目录或input_data目录" << endl;
        rclcpp::shutdown();
        return 1;
    }

    // 4. 拼接输入输出路径
    string imu_dir = package_abs_path + "/" + INPUT_ROOT + "/" + argv[1];
    string image_dir = package_abs_path + "/" + INPUT_ROOT + "/" + argv[2];
    string output_dir = package_abs_path + "/" + OUTPUT_ROOT;
    string bag_path = output_dir + "/" + string(argv[3]) + ".bag";

    // 5. 创建输出目录
    if (!fs::exists(output_dir)) {
        if (fs::create_directories(output_dir)) {
            cout << "创建输出目录：" << output_dir << endl;
        } else {
            cerr << "创建输出目录失败：" << output_dir << endl;
            rclcpp::shutdown();
            return 1;
        }
    }

    // 6. 读取IMU数据
    cout << "读取IMU目录：" << imu_dir << endl;
    string imu_file = findImuTxtFile(imu_dir);
    if (imu_file.empty()) {
        cerr << "错误：在IMU目录中未找到txt文件" << endl;
        rclcpp::shutdown();
        return 1;
    }
    
    vector<IMUData> imu_data_list;
    if (!readImuTxtFile(imu_file, imu_data_list)) {
        cerr << "读取IMU文件失败" << endl;
        rclcpp::shutdown();
        return 1;
    }
    cout << "成功读取 " << imu_data_list.size() << " 条IMU数据" << endl;

    // 7. 读取图像数据
    cout << "读取图像目录：" << image_dir << endl;
    auto all_image_files = getImageFiles(image_dir);
    if (all_image_files.empty()) {
        cerr << "未找到有效图片（需符合 left_xxx.png / right_xxx.png 格式）" << endl;
        rclcpp::shutdown();
        return 1;
    }

    vector<ImageFile> left_files, right_files;
    for (const auto& file : all_image_files) {
        if (file.type == "left") left_files.push_back(file);
        else if (file.type == "right") right_files.push_back(file);
    }
    cout << "识别结果: left=" << left_files.size() << " 张, right=" << right_files.size() << " 张" << endl;

    // 8. 合并所有数据并按时间戳排序
    vector<TimestampedMessage> all_messages;
    
    // 添加IMU数据
    for (const auto& imu : imu_data_list) {
        TimestampedMessage msg;
        msg.timestamp_us = imu.timestamp_us;
        msg.type = TimestampedMessage::IMU;
        msg.imu_data = imu;
        all_messages.push_back(msg);
    }
    
    // 添加左目图像
    for (const auto& img : left_files) {
        TimestampedMessage msg;
        msg.timestamp_us = img.timestamp_us;
        msg.type = TimestampedMessage::IMAGE_LEFT;
        msg.image_file = img;
        all_messages.push_back(msg);
    }
    
    // 添加右目图像
    for (const auto& img : right_files) {
        TimestampedMessage msg;
        msg.timestamp_us = img.timestamp_us;
        msg.type = TimestampedMessage::IMAGE_RIGHT;
        msg.image_file = img;
        all_messages.push_back(msg);
    }
    
    // 按时间戳排序
    sort(all_messages.begin(), all_messages.end(), 
         [](const TimestampedMessage& a, const TimestampedMessage& b) {
             return a.timestamp_us < b.timestamp_us;
         });
    
    cout << "总共 " << all_messages.size() << " 条消息，已按时间戳排序" << endl;

    // 9. 初始化rosbag写入器
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_opt;
    rosbag2_cpp::ConverterOptions conv_opt;
    storage_opt.uri = bag_path;
    storage_opt.storage_id = "sqlite3";
    conv_opt.input_serialization_format = "cdr";
    conv_opt.output_serialization_format = "cdr";
    writer.open(storage_opt, conv_opt);

    // 创建话题
    writer.create_topic({"/imu/data", "sensor_msgs/msg/Imu", "cdr", ""});
    if (!left_files.empty()) {
        writer.create_topic({"/left/image_raw", "sensor_msgs/msg/Image", "cdr", ""});
    }
    if (!right_files.empty()) {
        writer.create_topic({"/right/image_raw", "sensor_msgs/msg/Image", "cdr", ""});
    }

    // 10. 按时间戳顺序写入所有数据
    cout << "开始写入数据到：" << bag_path << endl;
    int imu_count = 0, left_count = 0, right_count = 0;
    
    for (size_t i = 0; i < all_messages.size(); ++i) {
        const auto& msg = all_messages[i];
        int64_t timestamp_ns = msg.timestamp_us * 1000LL;  // 微秒转纳秒
        rclcpp::Time timestamp(timestamp_ns);
        
        if (msg.type == TimestampedMessage::IMU) {
            // 写入IMU数据
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = timestamp;
            imu_msg.header.frame_id = "imu_frame";
            
            imu_msg.angular_velocity.x = msg.imu_data.angular_velocity[0];
            imu_msg.angular_velocity.y = msg.imu_data.angular_velocity[1];
            imu_msg.angular_velocity.z = msg.imu_data.angular_velocity[2];
            
            imu_msg.linear_acceleration.x = msg.imu_data.linear_acceleration[0];
            imu_msg.linear_acceleration.y = msg.imu_data.linear_acceleration[1];
            imu_msg.linear_acceleration.z = msg.imu_data.linear_acceleration[2];
            
            // 协方差矩阵（未知）
            imu_msg.angular_velocity_covariance[0] = -1.0;
            imu_msg.angular_velocity_covariance[4] = -1.0;
            imu_msg.angular_velocity_covariance[8] = -1.0;
            imu_msg.linear_acceleration_covariance[0] = -1.0;
            imu_msg.linear_acceleration_covariance[4] = -1.0;
            imu_msg.linear_acceleration_covariance[8] = -1.0;
            
            auto ser_msg = serializeImu(imu_msg, "/imu/data", timestamp);
            writer.write(ser_msg);
            imu_count++;
            
        } else if (msg.type == TimestampedMessage::IMAGE_LEFT || msg.type == TimestampedMessage::IMAGE_RIGHT) {
            // 写入图像数据
            cv::Mat img = cv::imread(msg.image_file.path, cv::IMREAD_UNCHANGED);
            if (img.empty()) {
                cerr << "跳过无效图片：" << msg.image_file.path << endl;
                continue;
            }
            
            std_msgs::msg::Header header;
            header.frame_id = (msg.type == TimestampedMessage::IMAGE_LEFT) ? "left_camera_frame" : "right_camera_frame";
            header.stamp = timestamp;
            
            auto ros_img = cvToRosImg(img, header);
            string topic = (msg.type == TimestampedMessage::IMAGE_LEFT) ? "/left/image_raw" : "/right/image_raw";
            auto ser_msg = serializeImg(ros_img, topic, timestamp);
            writer.write(ser_msg);
            
            if (msg.type == TimestampedMessage::IMAGE_LEFT) left_count++;
            else right_count++;
        }
        
        if ((i + 1) % 100 == 0) {
            cout << "已处理 " << i + 1 << "/" << all_messages.size() << " 条消息" << endl;
        }
    }

    // 11. 完成
    cout << "\n完成！bag文件路径: " << bag_path << endl;
    cout << "统计: IMU=" << imu_count << " 条, 左目=" << left_count << " 张, 右目=" << right_count << " 张" << endl;
    rclcpp::shutdown();
    return 0;
}

