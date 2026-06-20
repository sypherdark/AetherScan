cmake_minimum_required(VERSION 3.8)
project(aetherscan_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/indoor_simulator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME indoor_simulator
)

ament_package()
