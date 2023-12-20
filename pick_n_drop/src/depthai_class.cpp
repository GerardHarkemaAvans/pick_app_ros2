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

      pointcloud_photo_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_photo", 10);

      tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());


}

void DepthaiClass::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
      #if 0
      //pcl::PointCloud<pcl::PointXYZ> cloud_in;
      //pcl::PointCloud<pcl::PointXYZ> cloud_out;


      geometry_msgs::msg::TransformStamped transform;

      transform = tfBuffer_->lookupTransform("world", msg->header.frame_id, tf2::TimePointZero);
      pcl_ros::transformPointCloud(msg, pc2_msg_, transform);
      //pointcloud = msg;
      #endif
}

void DepthaiClass::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}


void DepthaiClass::detections_callback(const depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) const
{

}

int DepthaiClass::TakePCLPhoto(){

      // do some stuff

      pointcloud_photo_publisher->publish(pointcloud);

      return 0;
}
