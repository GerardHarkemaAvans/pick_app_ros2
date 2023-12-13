#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/executors.hpp"
// #include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include "pick_n_drop/depthai_class.hpp"


DepthaiClass::DepthaiClass(std::shared_ptr<rclcpp::Node> node) : _node(node)
{
      pointcloud_subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>("name", 10, std::bind(&DepthaiClass::pointcloud_callback, this, std::placeholders::_1));
      image_subscription_ = node->create_subscription<sensor_msgs::msg::Image>("name", 10, std::bind(&DepthaiClass::image_callback, this, std::placeholders::_1));
      detections_subscription_ = node->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>("name", 10, std::bind(&DepthaiClass::detections_callback, this, std::placeholders::_1));
}

void DepthaiClass::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void DepthaiClass::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}


void DepthaiClass::detections_callback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) const
{

}
