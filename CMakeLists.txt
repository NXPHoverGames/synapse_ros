cmake_minimum_required(VERSION 3.8)
project(synapse_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(synapse_protobuf REQUIRED)
find_package(synapse_tinyframe REQUIRED)
find_package(synapse_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME}
  src/synapse_ros.cpp
  src/proto/udp_link.cpp
  )

ament_target_dependencies(${PROJECT_NAME}
  synapse_tinyframe synapse_protobuf sensor_msgs actuator_msgs rclcpp nav_msgs builtin_interfaces synapse_msgs geometry_msgs)

#==========================================================
# install
#==========================================================

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
