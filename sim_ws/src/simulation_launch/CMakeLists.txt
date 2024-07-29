cmake_minimum_required(VERSION 3.8)
project(simulation_launch)

install(DIRECTORY launch
  maps
  robots
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()