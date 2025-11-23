#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <regex>
#include <map>
#include <set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>
#include <cstring>

namespace fs = std::filesystem;
using namespace std;

// 图像文件结构体
struct ImageFile {
    string path;
    int64_t timestamp;
    string type; // left/right
};

// 合并时间戳和类型提取（减少正则调用，保留核心逻辑）
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
        return a.timestamp < b.timestamp;
    });
    return files;
}

// OpenCV图像转ROS消息（保留你原来的编码判断逻辑）
sensor_msgs::msg::Image cvToRosImg(const cv::Mat& cv_img, const std_msgs::msg::Header& header) {
    sensor_msgs::msg::Image ros_img;
    ros_img.header = header;
    ros_img.height = cv_img.rows;
    ros_img.width = cv_img.cols;
    
    // 保留原编码格式判断（完全还原你的逻辑）
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
        std::cerr << "不支持的图像通道数: " << cv_img.channels() << std::endl;
        return ros_img;
    }
    
    ros_img.data.resize(ros_img.step * ros_img.height);
    memcpy(ros_img.data.data(), cv_img.data, ros_img.data.size());
    return ros_img;
}

// 序列化图像消息（精简冗余注释）
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

// 通用rosbag写入函数（合并重复逻辑，减少冗余）
void writeRosbag(rosbag2_cpp::Writer& writer, const vector<ImageFile>& files, const string& topic, const string& frame_id) {
    rosbag2_storage::TopicMetadata topic_meta = {topic, "sensor_msgs/msg/Image", "cdr", ""};
    writer.create_topic(topic_meta);

    for (size_t i = 0; i < files.size(); ++i) {
        cv::Mat img = cv::imread(files[i].path, cv::IMREAD_UNCHANGED);
        if (img.empty()) {
            cerr << "跳过无效图片：" << files[i].path << endl;
            continue;
        }

        std_msgs::msg::Header header;
        header.frame_id = frame_id;
        header.stamp.sec = files[i].timestamp / 1000000;
        header.stamp.nanosec = (files[i].timestamp % 1000000) * 1000;

        auto ros_img = cvToRosImg(img, header);
        auto ser_msg = serializeImg(ros_img, topic, rclcpp::Time(header.stamp));
        writer.write(ser_msg);

        if ((i + 1) % 10 == 0) cout << "已处理 " << i + 1 << "/" << files.size() << " 张" << endl;
    }
}

int main(int argc, char** argv) {
    // 1. 初始化ROS节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("image_to_rosbag2");

    // 2. 解析命令行参数（必须传入2个参数：输入子目录、输出子目录）
    if (argc != 3) {
        cerr << "用法错误！正确格式：" << endl;
        cerr << "ros2 run image_to_rosbag2 image_to_rosbag2 <input_subdir> <output_subdir>" << endl;
        cerr << "示例:ros2 run image_to_rosbag2 image_to_rosbag2 image_data_default output_test" << endl;
        rclcpp::shutdown();
        return 1;
    }

    // 3. 拼接输入输出路径（固定input_image和output_rosbag2根目录）
    const string INPUT_ROOT = "input_image";   // 固定输入根目录
    const string OUTPUT_ROOT = "output_rosbag2"; // 固定输出根目录
    string input_dir = INPUT_ROOT + "/" + argv[1];  // 输入路径：input_image/输入子目录
    string output_dir = OUTPUT_ROOT + "/" + argv[2];// 输出目录：output_rosbag2/输出子目录
    string bag_path = output_dir + "/" + argv[2] + ".bag"; // bag路径：输出目录/输出子目录.bag

    // 4. 创建输出目录（不存在则自动创建）
    if (!fs::exists(output_dir)) {
        if (fs::create_directories(output_dir)) {
            cout << "创建输出目录：" << output_dir << endl;
        } else {
            cerr << "创建输出目录失败：" << output_dir << endl;
            rclcpp::shutdown();
            return 1;
        }
    }

    // 5. 读取并分类图片
    cout << "读取图片目录：" << input_dir << endl;
    auto all_files = getImageFiles(input_dir);
    if (all_files.empty()) {
        cerr << "未找到有效图片（需符合 left_xxx.png / right_xxx.png 格式）" << endl;
        rclcpp::shutdown();
        return 1;
    }

    vector<ImageFile> left_files, right_files;
    for (const auto& file : all_files) {
        if (file.type == "left") left_files.push_back(file);
        else if (file.type == "right") right_files.push_back(file);
    }
    cout << "识别结果:left=" << left_files.size() << " 张,right=" << right_files.size() << " 张" << endl;

    // 6. 初始化rosbag写入器
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_opt;
    rosbag2_cpp::ConverterOptions conv_opt;
    storage_opt.uri = bag_path;
    storage_opt.storage_id = "sqlite3";
    conv_opt.input_serialization_format = "cdr";
    conv_opt.output_serialization_format = "cdr";
    writer.open(storage_opt, conv_opt);

    // 7. 写入数据（双目/单目自动适配）
    if (!left_files.empty() && !right_files.empty()) {
        cout << "开始写入双目数据到：" << bag_path << endl;
        writeRosbag(writer, left_files, "/left/image_raw", "left_camera_frame");
        writeRosbag(writer, right_files, "/right/image_raw", "right_camera_frame");
    } else {
        if (!left_files.empty()) {
            cout << "开始写入左目数据到：" << bag_path << endl;
            writeRosbag(writer, left_files, "/left/image_raw", "left_camera_frame");
        }
        if (!right_files.empty()) {
            cout << "开始写入右目数据到：" << bag_path << endl;
            writeRosbag(writer, right_files, "/right/image_raw", "right_camera_frame");
        }
    }

    // 8. 结束
    cout << "完成!bag文件路径:" << bag_path << endl;
    rclcpp::shutdown();
    return 0;
}