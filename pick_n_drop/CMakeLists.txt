cmake_minimum_required(VERSION 3.10.2)  # CMake version in Ubuntu 18.04 LTS
project(pick_n_drop VERSION 2.8.1 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()
find_package(moveit_ros_planning_interface REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

message(STATUS "------------------------------------------")
message(STATUS "Pick and Drop app is being built using AMENT.")
message(STATUS "------------------------------------------")

find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)



find_package(rclpy REQUIRED)

set(dependencies
  camera_info_manager
  cv_bridge
  rclcpp
  rclpy
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  stereo_msgs
  std_msgs
  vision_msgs
  moveit_ros_planning_interface
  depthai_ros_msgs
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)


macro(app_add_node_ros1 node_name node_src1)
  add_executable("${node_name}" "${node_src1}")

  target_link_libraries("${node_name}"
                        opencv_imgproc
                        opencv_highgui
                        jsoncpp)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

macro(app_add_node_ros2 node_name node_src1 node_src2)
  add_executable("${node_name}" "${node_src1}" "${node_src2}")

  target_link_libraries("${node_name}"
                        opencv_imgproc
                        opencv_highgui
                        jsoncpp)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

macro(app_add_node_ros3 node_name node_src1 node_src2 node_src3)
  add_executable("${node_name}" "${node_src1}" "${node_src2}" "${node_src3}")

  target_link_libraries("${node_name}"
                        opencv_imgproc
                        opencv_highgui
                        jsoncpp)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

macro(app_add_node_ros4 node_name node_src1 node_src2 node_src3 node_src4)
  add_executable("${node_name}" "${node_src1}" "${node_src2}" "${node_src3}" "${node_src4}")

  target_link_libraries("${node_name}"
                        opencv_imgproc
                        opencv_highgui
                        jsoncpp)

  ament_target_dependencies("${node_name}"
                            ${dependencies})

endmacro()

app_add_node_ros3(pick_n_drop_node 
                  src/pick_n_drop_app.cpp 
                  src/ur_control_class.cpp
                  src/object_detection_class.cpp)



install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})


install(TARGETS
        pick_n_drop_node
        DESTINATION lib/${PROJECT_NAME})


install(PROGRAMS
        #scripts/publisch_bouding_boxes.py
        scripts/publisch_dummy_detections.py
        DESTINATION lib/${PROJECT_NAME})


ament_package()

