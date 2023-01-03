cmake_minimum_required(VERSION 3.8)
project(minimal_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  example_interfaces
  generate_parameter_library
)

foreach(dependency ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(minimal_action_server_params
  "src/minimal_action_server_parameters.yaml"
)


add_library(minimal_action_server SHARED)# src/minimal_action_server.cpp)
target_sources(minimal_action_server PRIVATE src/minimal_action_server.cpp)

target_compile_features(minimal_action_server PUBLIC c_std_11 cxx_std_20)  # Require C99 and C++17
target_include_directories(minimal_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(minimal_action_server ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(minimal_action_server minimal_action_server_params)

add_executable(minimal_action_server_node)
target_sources(minimal_action_server_node PRIVATE src/minimal_action_server_node.cpp)
ament_target_dependencies(minimal_action_server_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(minimal_action_server_node minimal_action_server)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(minimal_action_server PRIVATE "MINIMAL_ACTION_SERVER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS 
    minimal_action_server
    minimal_action_server_params
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS minimal_action_server_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  minimal_action_server
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
