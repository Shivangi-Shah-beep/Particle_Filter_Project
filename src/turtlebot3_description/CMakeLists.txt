cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
