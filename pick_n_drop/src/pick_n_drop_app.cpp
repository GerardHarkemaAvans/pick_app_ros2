#include <cstdio>
#include "pick_n_drop/depthai_class.hpp"
#include "pick_n_drop/ur_control_class.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world pick_n_drop package\n");

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_n_drop_node");
  
  DepthaiClass Depthai(node);
  UrControlClass UrControl(node);

  typedef enum states{idle, start}STATES;

  STATES state = idle;

  for(;;){
    switch(state){
      case idle:
        state = start;
        break;
      case start:
        break;
    }

    rclcpp::shutdown();
  }


  return 0;

}
