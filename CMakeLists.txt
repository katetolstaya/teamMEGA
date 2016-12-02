cmake_minimum_required(VERSION 2.8.3)
project(teamMEGA)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /home/brent/catkin_ws/src/teamMEGA)

## Find OMPL
# find_package(OMPL)
 MESSAGE( STATUS "OMPL_FOUND" ${OMPL_FOUND} )
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(teamMEGA src/main.cpp)
target_link_libraries(teamMEGA ${catkin_LIBRARIES} )


#add_executable(QuadrotorPlanning src/QuadrotorPlanning.cpp)
#target_link_libraries(QuadrotorPlanning ${catkin_LIBRARIES} OMPL_LIBRARIES)


install(
	TARGETS teamMEGA 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
