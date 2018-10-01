cmake_minimum_required(VERSION 2.8.3)
project(sesame_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_service_files(
  FILES
  Command.srv
  Status.srv
  )

generate_messages(
  DEPENDENCIES
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories()

install(PROGRAMS
  node_scripts/sesame_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
