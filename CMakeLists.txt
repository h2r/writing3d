cmake_minimum_required(VERSION 2.8.3)
project(writing3d)

find_package(catkin REQUIRED
  genmsg
  actionlib_msgs
  actionlib
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  )


catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES PlanMoveEE.action ExecMoveEE.action
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

