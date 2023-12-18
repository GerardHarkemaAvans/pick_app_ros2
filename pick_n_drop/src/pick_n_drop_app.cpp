#include <cstdio>
#include "pick_n_drop/depthai_class.hpp"
#include "pick_n_drop/ur_control_class.hpp"

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  printf("hello world pick_n_drop package\n");

  rclcpp::init(argc, argv);
  auto app_node = rclcpp::Node::make_shared("pick_n_drop_node");

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(app_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  DepthaiClass Depthai(app_node);
  UrControlClass UrControl(app_node);

  typedef enum states
  {
    idle,
    start,
    robot_go_photo_pos,
    camera_take_pcl_photo,
    robot_go_resting_pos,
    end,
  } STATES;

  STATES state = idle;

  //string group_states[] = {"home", "left", "right", "home", "resting", "photo"};

  bool break_flag = false;

  for (;;)
  {
    switch (state)
    {
      case idle:
        state = start;
        break;
      case start:
        break;
      case robot_go_photo_pos:
        UrControl.movePose("photo");
        state = camera_take_pcl_photo;
        break;
      case camera_take_pcl_photo:
        Depthai.TakePCLPhoto();
        state = robot_go_resting_pos;
        break;
      case robot_go_resting_pos:
        UrControl.movePose("resting");
        state = end;
        break;
      case end:
        break_flag = true;
        break;
    }
    if(break_flag) break;
  }
  rclcpp::shutdown();

  return 0;
}
