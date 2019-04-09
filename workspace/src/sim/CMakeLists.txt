cmake_minimum_required(VERSION 2.8.3)
project(sim)

find_package(catkin REQUIRED COMPONENTS
  barc
  keyboard
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(keyboard_manual src/keyboard_manual.cpp)
target_link_libraries(keyboard_manual ${catkin_LIBRARIES})
#add_dependencies(keyboard_manual sim_generate_messages_cpp)
