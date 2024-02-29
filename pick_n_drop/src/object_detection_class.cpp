#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

#include "pick_n_drop/object_detection_class.hpp"


ObjectDetectionClass::ObjectDetectionClass(std::shared_ptr<rclcpp::Node> node) : _node(node), tf_broadcaster(_node)
{
      printf("ObjectDetectionClass constructor\n");

      detections_subscription_ = node->create_subscription<depthai_ros_msgs::msg::SpatialDetectionArray>("DummyDetections", 10, std::bind(&ObjectDetectionClass::detections_callback, this, std::placeholders::_1));
      //tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

}


void ObjectDetectionClass::detections_callback(depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr msg) const
{
      detections_array = msg;
}

//geometry_msgs::msg::TransformStamped  
bool ObjectDetectionClass::getNearestObjectPosition(){
      geometry_msgs::msg::TransformStamped transform;

      bool found = false;
      // detection shold not be older than 3 seconds
      rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(3.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      rclcpp::Time time_now = _node->now();

      if(detections_array){
            rclcpp::Time stamp = detections_array->header.stamp;
            if(stamp > (time_now - timeout_duration)){
                  //RCLCPP_INFO(_node->get_logger(), "I current the message , ID: %s", detections_array->header.frame_id.c_str());  
                  std::vector<depthai_ros_msgs::msg::SpatialDetection>::iterator it;
                  std::vector<depthai_ros_msgs::msg::SpatialDetection>::iterator nearest_detection = detections_array->detections.begin();
                  for (it = detections_array->detections.begin(); it != detections_array->detections.end(); ++it){
                        if(it->position.z < nearest_detection->position.z){
                              nearest_detection = it;
                        }
                        //RCLCPP_INFO(_node->get_logger(), "I current z position: %f", it->position.z);
                  }
                  RCLCPP_INFO(_node->get_logger(), "Nearest z position: %f",nearest_detection->position.z);

                  transform.header.stamp = _node->now();
                  transform.header.frame_id = "oak_rgb_camera_optical_frame";
                  transform.child_frame_id = "nearest_object";

                  transform.transform.translation.x = nearest_detection->position.x;
                  transform.transform.translation.y = -nearest_detection->position.y;
                  transform.transform.translation.z = nearest_detection->position.z;

                  transform.transform.rotation.x = 0.0;
                  transform.transform.rotation.y = 0.0;
                  transform.transform.rotation.z = 0.0;
                  transform.transform.rotation.w = 1.0;

                  detections_array = nullptr;
                  //_node->TransformBroadcaster(transform);
                  tf_broadcaster.sendTransform(transform);
                  found = true;
            }

      }    

      return found;
      
}