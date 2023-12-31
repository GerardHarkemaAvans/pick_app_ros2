#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/executors.hpp"
// #include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include <cstdio>
#include <iostream>
using namespace std;
// #include <pluginlib/class_loader.hpp>

#include "pick_n_drop/ur_control_class.hpp"

UrControlClass::UrControlClass(std::shared_ptr<rclcpp::Node> node) : _node(node)
{

      static const std::string PLANNING_GROUP = "arm";

      // The
      // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
      // class can be easily set up using just the name of the planning group you would like to control and plan for.
      // moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
      move_group = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);
}

int UrControlClass::movePose(const char *Posename)
{

      std::map<std::string, double> target = move_group->getNamedTargetValues(Posename);

      // We will use the
      // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
      // class to add and rem    std::cout << "1" << std::endl;ove collision objects in our "virtual world" scene
      //  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      moveit_msgs::msg::Constraints constraints;
      std::map<std::string, double>::iterator it = target.begin();
      while (it != target.end())
      {
            moveit_msgs::msg::JointConstraint joint_constraint;

            std::cout << it->first << " = " << it->second << std::endl;
            // Constrain the position of a joint to be within a certain bound
            joint_constraint.joint_name = it->first;

            // the bound to be achieved is [position - tolerance_below, position + tolerance_above]
            joint_constraint.position = it->second;
            joint_constraint.tolerance_above = 0.1;
            joint_constraint.tolerance_below = 0.1;

            // A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
            joint_constraint.weight = 1.0;

            constraints.joint_constraints.push_back(joint_constraint);

            it++;
      }

      move_group->setJointValueTarget(target);
      move_group->setPlanningTime(10.0);

      move_group->setPathConstraints(constraints);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      bool succes = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (succes)
      {
            // printf("Execute\n");
            move_group->execute(my_plan);
            // move_group->asyncExecute(my_plan);
      }

      return 0;
}
