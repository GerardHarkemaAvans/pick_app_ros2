#pragma once
#ifndef OBJECT_DETECTION_CLASS
#define OBJECT_DETECTION_CLASS

#include <memory>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "depthai_ros_msgs/msg/spatial_detection.hpp"
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "jsoncpp/json/json.h"


using namespace std::chrono_literals;



class ObjectDetectionClass
{
  std::shared_ptr<rclcpp::Node> _node;
  public:
    ObjectDetectionClass(std::shared_ptr<rclcpp::Node> node, std::string nn_config);
    bool getNearestObjectPosition(std::string &class_name);
  private:

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr detections_subscription_;
    void detections_callback(depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) const;
    mutable  depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr detections_array = nullptr;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    int NumClasses;
    std::vector<std::string> class_names;
};

#endif