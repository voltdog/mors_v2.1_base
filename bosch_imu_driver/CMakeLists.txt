cmake_minimum_required(VERSION 2.8.3)
project(bosch_imu_driver)

# set(CMAKE_CXX_FLAGS "-Wno-dev" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  rospy)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
	nodes/bosch_imu_node.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
