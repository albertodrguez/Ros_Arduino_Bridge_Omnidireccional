cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_python)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
)
catkin_package(DEPENDS)
catkin_python_setup()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_to_speed src/joy_to_speed.cpp)
target_link_libraries(joy_to_speed ${catkin_LIBRARIES})
