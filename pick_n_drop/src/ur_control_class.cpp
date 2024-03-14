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

//#include <moveit_visual_tools/moveit_visual_tools.h>

//namespace rvt = rviz_visual_tools;


UrControlClass::UrControlClass(std::shared_ptr<rclcpp::Node> node) : _node(node), tf_broadcaster(_node)
{

      static const std::string PLANNING_GROUP = "arm";

      // The
      // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
      // class can be easily set up using just the name of the planning group you would like to control and plan for.
      move_group = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP);

      joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);


      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
#if 0
      moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "move_group_tutorial",
                                                      move_group->getRobotModel());

      visual_tools.deleteAllMarkers();

      /* Remote control is an introspection tool that allows users to step through a high level script */
      /* via buttons and keyboard shortcuts in RViz */
      visual_tools.loadRemoteControl();

#endif

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
      move_group->setNumPlanningAttempts(10);

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

      geometry_msgs::msg::PoseStamped target_pose;

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

      target_pose.pose.orientation.x = myQuaternion.x();
      target_pose.pose.orientation.y = myQuaternion.y();
      target_pose.pose.orientation.z = myQuaternion.z();
      target_pose.pose.orientation.w = myQuaternion.w();
      target_pose.pose.position.x = t.transform.translation.x;
      target_pose.pose.position.y = t.transform.translation.y;
      double offsets[] = {0.05, 0, 0.05};

      printf("w(req): %f\n", target_pose.pose.orientation.w);
      printf("x(req): %f\n", target_pose.pose.orientation.x);
      printf("y(req): %f\n", target_pose.pose.orientation.y);
      printf("z(req): %f\n", target_pose.pose.orientation.z);
      target_pose.header.frame_id = toFrameRel;

      int i=0;
      for (double offset : offsets) {
            target_pose.pose.position.z = t.transform.translation.z + offset;
            target_pose.header.stamp = _node->now();

#if 1
            {
                  geometry_msgs::msg::TransformStamped transform;
                  //tf2_ros::TransformBroadcaster tf_broadcaster;
                  transform.header.stamp = _node->now();
                  transform.header.frame_id = "world";
                  transform.child_frame_id = "pick_point";

                  transform.transform.translation.x = target_pose.pose.position.x;
                  transform.transform.translation.y = target_pose.pose.position.y;
                  transform.transform.translation.z = target_pose.pose.position.z;

                  transform.transform.rotation.x = target_pose.pose.orientation.x;
                  transform.transform.rotation.y = target_pose.pose.orientation.y;
                  transform.transform.rotation.z = target_pose.pose.orientation.z;
                  transform.transform.rotation.w = target_pose.pose.orientation.w;
                  tf_broadcaster.sendTransform(transform);
            }
#endif

            moveit::core::RobotState start_state(*move_group->getCurrentState());
  #if 0
  geometry_msgs::msg::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  #endif
            move_group->setStartState(start_state);

            //return 0;
            move_group->setPlanningTime(10.0);
            move_group->setNumPlanningAttempts(10);
            move_group->setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

            bool success = (move_group->plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success)
            {
                  printf("Execute plan\n");
                  move_group->move();
                  geometry_msgs::msg::PoseStamped 	pose = move_group->getCurrentPose();
                  printf("w%i: %f\n", i, pose.pose.orientation.w);
                  printf("x%i: %f\n", i, pose.pose.orientation.x);
                  printf("y%i: %f\n", i, pose.pose.orientation.y);
                  printf("z%i: %f\n", i++, pose.pose.orientation.z);


            }
            else{
                  printf("Faild to create plan\n");
                  return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }



      return 0;
    }
