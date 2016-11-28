cmake_minimum_required(VERSION 2.8.3)
project(teamMEGA)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(teamMEGA src/main.cpp)
target_link_libraries(teamMEGA ${catkin_LIBRARIES})
add_dependencies(teamMEGA teamMEGA_messages_cpp)

install(
	TARGETS teamMEGA
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
