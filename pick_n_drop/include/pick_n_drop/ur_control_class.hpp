#pragma once
#ifndef UR_CONTROL_CLASS
#define UR_CONTROL_CLASS

#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>


#include "pick_n_drop/ur_control_class.hpp"

class UrControlClass
{
  std::shared_ptr<rclcpp::Node> _node;
  public:

    UrControlClass(std::shared_ptr<rclcpp::Node> node);
    int movePose(const char *Posename);
  private:
    moveit::planning_interface::MoveGroupInterface *move_group;

};

#endif