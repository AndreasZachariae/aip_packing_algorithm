cmake_minimum_required(VERSION 3.5)
project(aip_packing_planning_interfaces)

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

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  )

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Package.msg"
  "msg/PackageSequence.msg"
  "srv/PackSequence.srv"
  DEPENDENCIES ${DEPENDENCIES})

#file (GLOB MSGS msg/*.msg)
#file (GLOB ACTIONS action/*.action)
#file (GLOB SERVICES srv/*.srv)



ament_export_dependencies(rosidl_default_runtime ${DEPENDENCIES})
ament_package()