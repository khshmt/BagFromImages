cmake_minimum_required(VERSION 3.10.0)
project(BagFromImages)

set(CMAKE_CXX_STANDARD 17)
set(BUILD_SHARED_LIBS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

include(FetchContent)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master 
)
FetchContent_MakeAvailable(yaml-cpp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  rosbag
)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME}
  main.cpp
  ${PROJECT_SOURCE_DIR}/Thirdparty/DLib/FileFunctions.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  yaml-cpp::yaml-cpp
)