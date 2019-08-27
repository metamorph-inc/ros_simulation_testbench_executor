cmake_minimum_required(VERSION 2.8.3)
project(openmeta_testbench_simulation_executor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rosgraph
  rospy
  rostopic
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

# catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

