cmake_minimum_required(VERSION 3.8)
project(x500_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport14 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs11 REQUIRED)

include_directories(
  ${gz-transport14_INCLUDE_DIRS}
  ${gz-msgs11_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller.cpp)

target_link_libraries(motor_controller PUBLIC
  rclcpp::rclcpp
  protobuf
  ${gz-transport14_LIBRARIES}
  ${gz-msgs11_LIBRARIES}
)

install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
