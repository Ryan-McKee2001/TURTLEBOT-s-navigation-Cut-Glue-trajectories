cmake_minimum_required(VERSION 3.8)
project(custom_planner_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ADDED dependencies ------------------------------
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ADDED dependencies END --------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CustomPlannerInterface.srv"
  DEPENDENCIES geometry_msgs
  DEPENDENCIES nav_msgs
  DEPENDENCIES std_msgs
)

ament_package()
