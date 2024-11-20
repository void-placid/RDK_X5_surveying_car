# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.0.2)
project(ros_base_learning)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare a catkin package
catkin_package(
  LIBRARIES ros_base_learning
  CATKIN_DEPENDS roscpp rospy std_msgs
  INCLUDE_DIRS include
  DEPENDS system_lib
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(middle src/middle.cpp)
target_link_libraries(middle ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%

