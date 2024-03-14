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
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"


class UrControlClass
{
  std::shared_ptr<rclcpp::Node> _node;
  public:

    UrControlClass(std::shared_ptr<rclcpp::Node> node);
    int movePose(const char *Posename);
    int moveFrame();//geometry_msgs::msg::TransformStamped transform);
  private:
    moveit::planning_interface::MoveGroupInterface *move_group;
    const moveit::core::JointModelGroup* joint_model_group;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster;

};

#endif