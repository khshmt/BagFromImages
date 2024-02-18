# BagFromImages

ROS package to generate a rosbag from a collection of images from stereo camera setup. Images are ordered alphabetically. The timestamp for each image is assigned according to the specified frequency. 

Tested in ROS noetic.

## Installation

In your ROS_PACKAGE_PATH (check your environment variable ROS_PACKAGE_PATH):

    git clone https://github.com/raulmur/BagFromImages.git BagFromImages

    cd BagFromImages
    source /opt/ros/${ROS_DISTRO}/setup.bash
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PWD}
    mkdir build
    cd build
    cmake ..
    make

## Usage:

    rosrun BagFromImages BagFromImages parameters.yaml

    parameters:-
        left_imgs_path: "~/images/imgs/left_images"
        right_imgs_path: "~/images/imgs/right_images"
        left_imgs_topic: "/camera/left/image_raw"
        right_imgs_topic: "/camera/right/image_raw"
        imgs_extention: ".png"
        imgs_encoding: "bgr8" # Image encoding ("mono8", "bgr8", "rgb8", etc.)
        frequency: 20.0
        output_bag_path: "~/images/output.bag"

