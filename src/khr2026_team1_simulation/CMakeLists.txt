cmake_minimum_required(VERSION 3.8)
project(khr2026_team1_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/invert_cmd_vel.py
  scripts/generate_field_sdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
