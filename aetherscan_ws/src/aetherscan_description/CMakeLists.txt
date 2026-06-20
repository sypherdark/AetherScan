cmake_minimum_required(VERSION 3.8)
project(aetherscan_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  sdf
  meshes
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
