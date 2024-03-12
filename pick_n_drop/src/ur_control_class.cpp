#include <memory>
#include <chrono>

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

UrControlClass::UrControlClass(std::shared_ptr<rclcpp::Node> node) : _node(node), tf_broadcaster(_node)
{

      static const std::string PLANNING_GROUP = "arm";

      // The
      // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
      // class can be easily set up using just the name of the planning group you would like to control and plan for.
      move_group = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      printf("Move group ready\n");
}

int UrControlClass::movePose(const char *Posename)
{
      printf("Move to %s\n", Posename);
      std::map<std::string, double> target = move_group->getNamedTargetValues(Posename);


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
            joint_constraint.tolerance_above = 0.3;
            joint_constraint.tolerance_below = 0.3;

            // A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
            joint_constraint.weight = 1.0;

            constraints.joint_constraints.push_back(joint_constraint);

            it++;
      }


      //move_group->setJointValueTarget(target);
      //move_group->setPlanningTime(100.0);
      //move_group->setNumPlanningAttempts (10);

      //move_group->setPathConstraints(constraints);

      //move_group->move();

      move_group->setJointValueTarget(move_group->getNamedTargetValues(Posename));
      
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

      bool success = (move_group->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
            printf("Execute plan\n");
            move_group->move();
      }
      else{
            printf("Faild to create plan\n");
      }




#if 0
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      bool succes = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (succes)
      {
            printf("Execute plan\n");
            move_group->execute(my_plan);
            // move_group->asyncExecute(my_plan);
      }
      else{
            printf("Faild to create plan\n");

      }
#endif
      return 0;
}

int UrControlClass::moveFrame(){
      //std::cout << transform.header.frame_id << std::endl;

      geometry_msgs::msg::Pose target_pose;

      // Store frame names in variables that will be used to
      // compute transformations
      std::string fromFrameRel = "nearest_object";
      std::string toFrameRel = "world";//"base";//_link";

      geometry_msgs::msg::TransformStamped t;

      // Look up for the transformation between target_frame and turtle2 frames
      // and send velocity commands for turtle2 to reach target_frame
      try {
            t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            //_node->get_clock()->now());//
            tf2::TimePointZero);//,
            
            //rclcpp::Duration::from_seconds(2.0));
      } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
            _node->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return 0;
      }

      tf2::Quaternion myQuaternion;

      myQuaternion.setRPY(3.14 ,0, 3.14/2);

      myQuaternion=myQuaternion.normalize();

      target_pose.orientation.x = myQuaternion.x();
      target_pose.orientation.y = myQuaternion.y();
      target_pose.orientation.z = myQuaternion.z();
      target_pose.orientation.w = myQuaternion.w();
      target_pose.position.x = t.transform.translation.x;
      target_pose.position.y = t.transform.translation.y;
      target_pose.position.z = t.transform.translation.z;//+0.05;

      {
            geometry_msgs::msg::TransformStamped transform;
            //tf2_ros::TransformBroadcaster tf_broadcaster;
            transform.header.stamp = _node->now();
            transform.header.frame_id = "world";
            transform.child_frame_id = "pick_point";

            transform.transform.translation.x = target_pose.position.x;
            transform.transform.translation.y = target_pose.position.y;
            transform.transform.translation.z = target_pose.position.z;

            transform.transform.rotation.x = target_pose.orientation.x;
            transform.transform.rotation.y = target_pose.orientation.y;
            transform.transform.rotation.z = target_pose.orientation.z;
            transform.transform.rotation.w = target_pose.orientation.w;
            tf_broadcaster.sendTransform(transform);
      }


      //return 0;
      move_group->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

      bool success = (move_group->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
            printf("Execute plan\n");
            move_group->move();
      }
      else{
            printf("Faild to create plan\n");
            return 1;
      }



      return 0;
    }
