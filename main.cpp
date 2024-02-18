// STL
#include <iostream>
#include <unordered_map>
// External
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// yaml-cpp
#include <yaml-cpp/yaml.h>
// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Internal Utilities
#include "Thirdparty/DLib/FileFunctions.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "BagFromImages");

    if (argc != 2) {
        std::cerr << "Enter a Yaml file: \n"
                  << "for example:- \n"
                  << "================\n"
                  << " left_imgs_path: \"~/left_image_dir\"\n"
                  << " left_imgs_topic: \"/camera/left\"\n"
                  << " right_imgs_path: \"~/right_image_dir\"\n"
                  << " right_imgs_topic: \"/camera/right\"\n"
                  << " imgs_extention: \"bgr8\"(default_value=20.0)\n"
                  << " imgs_extention: \".png\"(default_value=.png)\n"
                  << " frequency: 20.0(default_value=20.0)\n"
                  << " output_bag_path: \"./output.bag\"(default_value=./output.bag)" << '\n';
        return EXIT_FAILURE;
    }

    ros::start();

    YAML::Node node = YAML::LoadFile((std::string)argv[1]);

    // image encoding
    std::string image_encoding{"bgr8"};
    if(node["imgs_encoding"].IsDefined())
        image_encoding = node["imgs_encoding"].as<std::string>();

    if (!node["left_imgs_path"].IsDefined()  ||
        !node["right_imgs_path"].IsDefined() ||
        !node["left_imgs_topic"].IsDefined() ||
        !node["right_imgs_topic"].IsDefined()) {
        std::cout << "Enter the topic names for left and right images and images paths for left and right images\n";
        return EXIT_FAILURE;
    }
    const auto left_imgs_Path  = node["left_imgs_path"].as<std::string>();
    const auto right_imgs_Path = node["right_imgs_path"].as<std::string>();
    const auto left_topic      = node["left_imgs_topic"].as<std::string>();
    const auto right_topic     = node["right_imgs_topic"].as<std::string>();
    
    std::string imgs_extention{".png"};
    if (node["imgs_extention"].IsDefined())
        imgs_extention = node["imgs_extention"].as<std::string>();

    std::vector<std::string> left_filenames =
        DUtils::FileFunctions::Dir(left_imgs_Path.c_str(), imgs_extention.c_str(), true);
    std::vector<std::string> right_filenames = 
        DUtils::FileFunctions::Dir(right_imgs_Path.c_str(), imgs_extention.c_str(), true);
    std::vector<std::string> topic_names{left_topic, right_topic};

    if(left_filenames.size() != right_filenames.size()) {
        std::cout << "left and right images have different sizes\n";
    }
    std::cout << "Left_Images: " << left_filenames.size() << '\n';
    std::cout << "Right_Images: " << right_filenames.size() << '\n';

    std::unordered_map<std::string, std::vector<std::string>> images{{left_topic, left_filenames}, {right_topic, right_filenames}};

    // Frequency
    double freq{20.0};
    if(node["frequency"].IsDefined())
        freq = node["frequency"].as<double>();

    // Output bag
    std::string output_bag_path{"./output.bag"};
    if(node["output_bag_path"].IsDefined())
        output_bag_path = node["output_bag_path"].as<std::string>();
    
    rosbag::Bag bag_out(output_bag_path, rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T = 1.0f / freq;
    ros::Duration d(T);

    for (const auto &[topic_name, file_names] : images) {
        auto i{1};
        for (const auto& path : file_names) {
            if (!ros::ok())
                break;

            cv::Mat im = cv::imread(path, cv::IMREAD_COLOR);
            cv_bridge::CvImage cvImage;
            cvImage.image = im;
            cvImage.encoding = image_encoding;
            cvImage.header.stamp = t;
            bag_out.write(topic_name, ros::Time(t), cvImage.toImageMsg());
            t += d;
            std::cout << i++ << " / " << file_names.size() << '\n';
        }
    }
    bag_out.close();
    
    ros::shutdown();

    return 0;
}