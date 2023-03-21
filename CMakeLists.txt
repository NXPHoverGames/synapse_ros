cmake_minimum_required(VERSION 3.8)
project(synaptic_gap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost COMPONENTS asio)

# cmake macros
include(FetchContent)
include(FindProtobuf)

#==========================================================
# protobuf serialization, tinyframe framing
#==========================================================

FetchContent_Populate(
  tinyframe
  GIT_REPOSITORY https://github.com/MightyPork/TinyFrame.git
  GIT_TAG  a29167a69f052975b0e0134a73b4d31d03afa8fa
  SOURCE_DIR tinyframe
  QUIET
)

FetchContent_Populate(
  synapse_pb
  GIT_REPOSITORY https://github.com/CogniPilot/synapse_pb.git
  GIT_TAG 0e2a88c65dfacb16478cd2963489f8ed288183ab
  SOURCE_DIR synapse_pb
  QUIET
)

file(GLOB PROTO_FILES "${CMAKE_BINARY_DIR}/synapse_pb/*.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS DESCRIPTORS PROTO_DESCS ${PROTO_FILES})  

add_library(synapse
  ${PROTO_SRCS} ${PROTO_HDRS}
  ${CMAKE_BINARY_DIR}/tinyframe/TinyFrame.c
)
target_include_directories(synapse PRIVATE
  ${Protobuf_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tinyframe
  include/tinyframe)

#==========================================================
# executables
#==========================================================


# tcp_echo_server
add_executable(tcp_echo_server
  src/tcp_echo_server.cpp
)
ament_target_dependencies(tcp_echo_server rclcpp)
target_link_libraries(tcp_echo_server
  synapse
  ${Boost_Libraries}
)
target_include_directories(tcp_echo_server PRIVATE
  ${Protobuf_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(tcp_echo_client src/tcp_echo_client.cpp)
target_link_libraries(tcp_echo_client
  synapse
  ${Boost_Libraries}
)
ament_target_dependencies(tcp_echo_client rclcpp)

#==========================================================
# install
#==========================================================

install(TARGETS
  tcp_echo_server
  tcp_echo_client
  synapse
  DESTINATION lib/${PROJECT_NAME}
)

#==========================================================
# testing
#==========================================================

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
