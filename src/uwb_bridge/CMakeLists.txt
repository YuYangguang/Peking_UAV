cmake_minimum_required(VERSION 2.8.3)
project(uwb_bridge)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

#set(CMAKE_BUILD_TYPE "Release")
#if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
#else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_BUILD_TYPE "Release")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
#endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )
## Add support for C++11, supported in ROS Kinetic and newer
#add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 message_filters
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs

)


add_message_files(DIRECTORY msg/
FILES
uwbMsg.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_generation

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/commander.cpp
# )
add_library(uwb
        src/uwb_main.cpp
	
)

target_link_libraries(uwb ${catkin_LIBRARIES}
)
add_dependencies(uwb ${catkin_EXPORTED_TARGETS})

add_executable(uwb_node src/uwb_main.cpp
			 )

target_link_libraries(uwb_node
${catkin_LIBRARIES} 
 )



