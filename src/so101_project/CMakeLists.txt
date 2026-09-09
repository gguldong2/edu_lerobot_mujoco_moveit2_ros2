cmake_minimum_required(VERSION 3.16)
project(so101_project NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf mjcf
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes in an ament_cmake package must be installed as programs.
install(PROGRAMS
  scripts/group_goal_client.py
  scripts/joint_command.py
  scripts/leader_follow.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
