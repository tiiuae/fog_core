cmake_minimum_required(VERSION 3.5)
project(fog_core)

find_package(ament_cmake REQUIRED)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
