#include <cstdio>
#include "pick_n_drop/depthai_class.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world pick_n_drop package\n");

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_n_drop_node");
  //rclcpp::spin(std::make_shared<DepthaiClass>());
  
  DepthaiClass test(node);

  rclcpp::shutdown();
  return 0;

}
