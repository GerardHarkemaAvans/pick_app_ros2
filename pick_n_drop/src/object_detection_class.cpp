#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/executors.hpp"
// #include "rclcpp/node.hpp"
#include <string.h>
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include "pick_n_drop/object_detection_class.hpp"


ObjectDetectionClass::ObjectDetectionClass(std::shared_ptr<rclcpp::Node> node) : _node(node)
{
      detections_subscription_ = node->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>("/color/detections", 10, std::bind(&ObjectDetectionClass::detections_callback, this, std::placeholders::_1));

}


void ObjectDetectionClass::detections_callback(depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) const
{
//      detections = std::make_shared<depthai_ros_msgs::msg::SpatialDetectionArray_<std::allocator<void>>>(msg);

      //detections = &msg;
      //RCLCPP_INFO(_node->get_logger(), "I received the message , height is: %s", msg->header.frame_id);
      detections_ = msg;

}

