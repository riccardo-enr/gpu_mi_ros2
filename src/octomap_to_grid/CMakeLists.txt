cmake_minimum_required(VERSION 3.16)
project(octomap_to_grid)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OccupancyGrid3D.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(octomap_to_grid_node src/octomap_to_grid_node.cpp)
ament_target_dependencies(octomap_to_grid_node
  rclcpp
  std_msgs
  geometry_msgs
  octomap_msgs
  octomap
)
target_include_directories(octomap_to_grid_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_to_grid_node ${OCTOMAP_LIBRARIES})

# Link the generated interfaces (this package's own msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(octomap_to_grid_node "${cpp_typesupport_target}")

install(TARGETS octomap_to_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
