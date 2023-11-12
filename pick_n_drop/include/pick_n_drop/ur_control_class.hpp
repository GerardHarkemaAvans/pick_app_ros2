#pragma once
#ifndef UR_CONTROL_CLASS
#define UR_CONTROL_CLASS

#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include "pick_n_drop/ur_control_class.hpp"

class UrControlClass
{
  std::shared_ptr<rclcpp::Node> _node;
  public:
    UrControlClass(std::shared_ptr<rclcpp::Node> node);//: _node(node);

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
};

#endif