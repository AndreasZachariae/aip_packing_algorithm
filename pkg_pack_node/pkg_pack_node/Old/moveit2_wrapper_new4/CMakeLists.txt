cmake_minimum_required(VERSION 3.5)
project(moveit_wrapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(iras_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_library(
  moveit_wrapper
  SHARED
  src/moveit_wrapper.cpp
)
target_include_directories(
  moveit_wrapper
  PRIVATE
  include
)
ament_target_dependencies(
  moveit_wrapper
  rclcpp
  rclcpp_lifecycle
  moveit_core
  moveit_ros_planning_interface
  iras_srvs
  geometry_msgs
  moveit_msgs
  shape_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  moveit_wrapper
)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  moveit_core
  moveit_ros_planning_interface
  iras_srvs
  geometry_msgs
  moveit_msgs
  shape_msgs
)
install(
  TARGETS moveit_wrapper
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)


add_executable(moveit_wrapper_node src/moveit_wrapper_node.cpp src/moveit_wrapper.cpp)
add_executable(attach_move_detach_clients_node src/attach_move_detach_clients_node.cpp src/moveit_wrapper.cpp)

target_include_directories(
  moveit_wrapper_node
  PRIVATE
  include)
target_include_directories(
  attach_move_detach_clients_node
  PRIVATE
  include)

ament_target_dependencies(moveit_wrapper_node rclcpp rclcpp_lifecycle moveit_core moveit_ros_planning_interface iras_srvs geometry_msgs moveit_msgs shape_msgs)
ament_target_dependencies(attach_move_detach_clients_node rclcpp rclcpp_lifecycle moveit_core moveit_ros_planning_interface iras_srvs geometry_msgs moveit_msgs shape_msgs )

install(TARGETS
  moveit_wrapper_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  attach_move_detach_clients_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

