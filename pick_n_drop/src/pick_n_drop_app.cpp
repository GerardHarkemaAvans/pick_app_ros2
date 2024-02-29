#include <cstdio>
#include "pick_n_drop/depthai_class.hpp"
#include "pick_n_drop/ur_control_class.hpp"
#include "pick_n_drop/object_detection_class.hpp"

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
 
  printf("Hello pick_n_drop node\n");

  rclcpp::init(argc, argv);
  auto app_node = rclcpp::Node::make_shared("pick_n_drop_node");

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(app_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  //DepthaiClass Depthai(app_node);
  UrControlClass UrControl(app_node);
  ObjectDetectionClass ObjectDetection(app_node);

  typedef enum states
  {
    idle,
    start,
    robot_go_photo_pos,
    camera_detect_objects,
    robot_go_picking_pos,
    robot_go_resting_pos,
    end,
  } STATES;

  STATES state = idle;
  printf("Start state sequencer\n");

  //string group_states[] = {"home", "left", "right", "home", "resting", "photo", "robot_go_picking_pos"};

  bool break_flag = false;

  for (;;)
  {
    switch (state)
    {
      case idle:
        printf("state: idle\n");
        state = start;
        break;
      case start:
        printf("state: start\n");
        state = robot_go_photo_pos;
        break;
      case robot_go_photo_pos:
        printf("state: robot_go_photo_pos\n");

        UrControl.movePose("photo");
        state = camera_detect_objects;
        break;
      case camera_detect_objects:
        printf("state: camera_detect_objects\n");
        if(ObjectDetection.getNearestObjectPosition()){
          state = robot_go_picking_pos;
        }
        else{
          state = robot_go_resting_pos;
        }
        //Depthai.TakePCLPhoto();
        break;
      case robot_go_picking_pos:
        printf("state: robot_go_picking_pos\n");
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        state = robot_go_resting_pos;
        break;
      case robot_go_resting_pos:
        printf("state: robot_go_resting_pos\n");
        UrControl.movePose("resting");
        state = end;
        break;
      case end:
        printf("state: end\n");
        break_flag = true;
        break;
    }
    if(break_flag) break;
    //rclcpp::sleep_for(1000ms);
  }
  rclcpp::shutdown();
  
  printf("Ready\n");
  
  return 0;
}
