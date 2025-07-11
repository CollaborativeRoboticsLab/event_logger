cmake_minimum_required(VERSION 3.8)
project(event_logger)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(event_logger_msgs REQUIRED)

include_directories(
  include
)

############################################################################
# node implementation that compiles as a executable 
############################################################################

add_executable(${PROJECT_NAME}_node
  src/event_listener_node.cpp
)

# target_link_libraries(${PROJECT_NAME}_node)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  event_logger_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

############################################################################
# miscellaneous
############################################################################

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
