cmake_minimum_required(VERSION 3.5)

project(irobot_create_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
    launch
    rviz
    map
    param
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    src/rs_to_scan.py
  DESTINATION
    lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
