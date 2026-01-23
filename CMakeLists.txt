cmake_minimum_required(VERSION 3.5)
project(planning_tools)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(service REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
#  ${geometry_msgs_INCLUDE_DIRS}
#  ${robotino_msgs_INCLUDE_DIRS}
)

add_library(planning_tools SHARED
  src/Planning.cpp
)

ament_target_dependencies(planning_tools
  rclcpp
  std_srvs
  service
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS planning_tools
  DESTINATION lib
)
 # AQUI AGREGUE LO DEL LAUNCH
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(planning_tools)

ament_package()
