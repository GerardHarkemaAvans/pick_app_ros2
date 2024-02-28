#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <string.h>
using std::placeholders::_1;

#include "pick_n_drop/object_detection_class.hpp"


ObjectDetectionClass::ObjectDetectionClass(std::shared_ptr<rclcpp::Node> node) : _node(node)
{
      printf("ObjectDetectionClass constructor\n");

      detections_subscription_ = node->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>("DummyDetections", 10, std::bind(&ObjectDetectionClass::detections_callback, this, std::placeholders::_1));

}


void ObjectDetectionClass::detections_callback(depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) const
{
//      detections = std::make_shared<depthai_ros_msgs::msg::SpatialDetectionArray_<std::allocator<void>>>(msg);

      //detections = &msg;
      RCLCPP_INFO(_node->get_logger(), "I received the message , ID: %s", msg->header.frame_id.c_str());
      //printf("Received message Id: %s\n", msg->header.frame_id.c_str());
      detections_ = msg;

}

geometry_msgs::msg::Point ObjectDetectionClass::getNearestObjectPosition(){
      geometry_msgs::msg::Point pnt;

      if(detections_){
            RCLCPP_INFO(_node->get_logger(), "I current the message , ID: %s", detections_->header.frame_id.c_str());  
            detections_ = nullptr;
      }    

      return pnt;
      
}