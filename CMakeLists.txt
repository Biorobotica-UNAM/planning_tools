cmake_minimum_required(VERSION 3.5)
project(planning_tools)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(clips_interfaces REQUIRED)
find_package(semantic_parser_interfaces REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_library(planning_tools SHARED
  src/Planning.cpp
)

ament_target_dependencies(planning_tools
  rclcpp
  std_srvs
  clips_interfaces
  semantic_parser_interfaces
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS planning_tools
  DESTINATION lib
)

install(
  PROGRAMS scripts/qwen_to_plan.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_srvs
  clips_interfaces
  semantic_parser_interfaces
)

ament_export_include_directories(include)
ament_export_libraries(planning_tools)

ament_package()
