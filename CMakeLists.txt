cmake_minimum_required(VERSION 3.8)
project(synapse_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(synapse_protobuf REQUIRED)
find_package(synapse_tinyframe REQUIRED)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/clients/tcp_client.cpp
  src/clients/ros_client.cpp
  )
ament_target_dependencies(${PROJECT_NAME}
  synapse_tinyframe synapse_protobuf sensor_msgs actuator_msgs rclcpp)

#==========================================================
# install
#==========================================================

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

#==========================================================
# testing
#==========================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
