cmake_minimum_required(VERSION 3.8)
project(behavior_trees_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(final_approach_controller_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(THIS_PACKAGE_DEPS
  behaviortree_ros2
  btcpp_ros2_interfaces
  final_approach_controller_msgs
)

# Build BTExecutionServer
add_executable(bt_executor
  src/bt_executor.cpp
)
ament_target_dependencies(
  bt_executor
  ${THIS_PACKAGE_DEPS}
)

# Build action client that calls controller clients (static version)
add_executable(final_approach_controller_client
  src/final_approach_controller_action.cpp
  src/final_approach_controller_client.cpp
)
ament_target_dependencies(
  final_approach_controller_client
  ${THIS_PACKAGE_DEPS}
)

# Build a client that calls controller clients (plugin version)
add_library(final_approach_controller_plugin SHARED src/final_approach_controller_action.cpp)
target_compile_definitions(final_approach_controller_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(final_approach_controller_plugin ${THIS_PACKAGE_DEPS})

add_executable(final_approach_controller_client_dyn src/final_approach_controller_client.cpp)
target_compile_definitions(final_approach_controller_client_dyn PRIVATE USE_SLEEP_PLUGIN)
target_link_libraries(final_approach_controller_client_dyn final_approach_controller_plugin)
ament_target_dependencies(final_approach_controller_client_dyn ${THIS_PACKAGE_DEPS})

# # Build server
# add_executable(final_approach_controller_server )

# Install
install(TARGETS
  bt_executor
  final_approach_controller_client
  final_approach_controller_client_dyn
  final_approach_controller_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugins for other packages to load
install(TARGETS
  final_approach_controller_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

# install behavior.xml's, configs, launch files
install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces final_approach_controller_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
