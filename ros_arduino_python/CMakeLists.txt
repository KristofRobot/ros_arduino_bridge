cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_python)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)
catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/PID.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS nodes/arduino_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
