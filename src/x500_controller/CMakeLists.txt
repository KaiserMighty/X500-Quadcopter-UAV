cmake_minimum_required(VERSION 3.8)
project(x500_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport14 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(flight_controller src/flight_controller.cpp)

target_link_libraries(flight_controller PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  protobuf
  ${gz-transport14_LIBRARIES}
  ${gz-msgs11_LIBRARIES}
  tf2::tf2
)

install(TARGETS
  flight_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
