cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- ROS2 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ORB-SLAM3 is built outside colcon; link against its shared library.
# Workspace layout:
#   project/ORB_SLAM3
#   project/ros2_21_tutorials-master/orbslam3_ros2  (this package)
set(ORB_SLAM3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ORB_SLAM3")

# Reuse ORB-SLAM3's CMake find-modules (e.g. FindPangolin.cmake).
list(APPEND CMAKE_MODULE_PATH "${ORB_SLAM3_DIR}/cmake_modules")

# ---- Third-party ----
# IMPORTANT: ORB-SLAM3 in this workspace is linked against OpenCV 4.8 installed in
#   /home/aaa/opt/opencv-4.8.0
# Mixing that with ROS2 Humble's cv_bridge OpenCV (4.5) can segfault at runtime.
# We therefore avoid cv_bridge and force-find the same OpenCV as ORB-SLAM3.
if(NOT OpenCV_DIR AND EXISTS "/home/aaa/opt/opencv-4.8.0/lib/cmake/opencv4")
  set(OpenCV_DIR "/home/aaa/opt/opencv-4.8.0/lib/cmake/opencv4" CACHE PATH "OpenCV config directory" FORCE)
endif()

find_package(OpenCV REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(orbslam3_mono_node
  src/orbslam3_mono_node.cpp
)

target_include_directories(orbslam3_mono_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

ament_target_dependencies(orbslam3_mono_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Link ORB-SLAM3 and the libs it expects at runtime.
target_link_libraries(orbslam3_mono_node
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  boost_system
  boost_serialization
  crypto
)

install(TARGETS orbslam3_mono_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
