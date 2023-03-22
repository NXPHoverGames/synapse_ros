cmake_minimum_required(VERSION 3.8)
project(synaptic_gap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(synapse_protobuf REQUIRED)
find_package(synapse_tinyframe REQUIRED)

set(DEPENDENCIES rclcpp Protobuf Boost synapse_protobuf synapse_tinyframe)

# tcp_echo_server
add_executable(tcp_echo_server src/tcp_echo_server.cpp)
ament_target_dependencies(tcp_echo_server synapse_protobuf synapse_tinyframe)

add_executable(tcp_echo_client src/tcp_echo_client.cpp)
ament_target_dependencies(tcp_echo_client synapse_protobuf synapse_tinyframe)

#==========================================================
# install
#==========================================================

install(TARGETS
  tcp_echo_server
  tcp_echo_client
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
