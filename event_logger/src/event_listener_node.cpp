#include <event_logger/event_listener.hpp>

int main(int argc, char* argv[])
{
  // Initialize the ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Create a shared pointer to the CapabilitiesFabricClient
  auto listener_node = std::make_shared<EventListener>();
  
  // Spin the node to process callbacks
  rclcpp::spin(listener_node);

  rclcpp::shutdown();

  return 0;
}
