#include <cstdio>
#include "jaka_zu5_driver/jaka_zu5_driver.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto jaka_zu5_node = std::make_shared<jaka_zu5_driver::jaka_robot_driver>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &jaka_zu5_node]() {
    executor.add_node(jaka_zu5_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(jaka_zu5_node->getNodeBaseInterface());
  });

  //   // 示例：主线程做别的事
  // rclcpp::sleep_for(std::chrono::seconds(5));
  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  printf("hello world jaka_zu5_driver package\n");
  return 0;
}
