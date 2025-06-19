cmake_minimum_required(VERSION 3.5)
project(x500_model)
find_package(ament_cmake REQUIRED)
install(DIRECTORY models launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
