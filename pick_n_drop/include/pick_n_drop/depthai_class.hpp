#pragma once
#ifndef DEPTHAI_CLASS
#define DEPTHAI_CLASS

#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include "pick_n_drop/depthai_class.hpp"

class DepthaiClass
{
  std::shared_ptr<rclcpp::Node> _node;
  public:
    DepthaiClass(std::shared_ptr<rclcpp::Node> node);//: _node(node);

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
};

#endif