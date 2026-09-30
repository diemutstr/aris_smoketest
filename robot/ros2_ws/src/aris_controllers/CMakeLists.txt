cmake_minimum_required(VERSION 3.8)
project(aris_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(aris_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(Franka REQUIRED)

add_library(${PROJECT_NAME} SHARED src/joint_impedance_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${PROJECT_NAME}
  controller_interface hardware_interface rclcpp rclcpp_lifecycle pluginlib realtime_tools
  std_srvs aris_msgs franka_semantic_components)
target_link_libraries(${PROJECT_NAME} Franka::Franka)

pluginlib_export_plugin_description_file(controller_interface aris_controllers.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# The core (law, reference, holds) has no ROS in it; this test runs without the robot.
if(BUILD_TESTING)
  add_executable(core_test test/core_test.cpp)
  target_include_directories(core_test PRIVATE include ${EIGEN3_INCLUDE_DIR})
  add_test(NAME core_test COMMAND core_test)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface hardware_interface rclcpp rclcpp_lifecycle
  pluginlib realtime_tools std_srvs aris_msgs franka_semantic_components)
ament_package()
