#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/executors.hpp"
//#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include "pick_n_drop/ur_control_class.hpp"



UrControlClass::UrControlClass(std::shared_ptr<rclcpp::Node> node): _node(node){

}

void UrControlClass::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}
