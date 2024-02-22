#pragma once
#ifndef OBJECT_DETECTION_CLASS
#define OBJECT_DETECTION_CLASS

#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//#include <pcl/common/transforms.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>

using namespace std::chrono_literals;



class ObjectDetectionClass
{
  std::shared_ptr<rclcpp::Node> _node;
  public:
    ObjectDetectionClass(std::shared_ptr<rclcpp::Node> node);
  private:

    rclcpp::Subscription<depthai_ros_msgs::msg::SpatialDetectionArray>::SharedPtr detections_subscription_;
    void detections_callback(depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) const;
    depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr detections;
    mutable  depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr detections_;
};

#endif