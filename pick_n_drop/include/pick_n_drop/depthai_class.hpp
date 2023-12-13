#pragma once
#ifndef DEPTHAI_CLASS
#define DEPTHAI_CLASS

#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

//#include "pick_n_drop/depthai_class.hpp"

class DepthaiClass
{
  std::shared_ptr<rclcpp::Node> _node;
  public:
    DepthaiClass(std::shared_ptr<rclcpp::Node> node);

  private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

};

#endif