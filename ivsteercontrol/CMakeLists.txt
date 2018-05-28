cmake_minimum_required(VERSION 2.8.3)
project(ivsteercontrol)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

link_directories(../../../../devel/lib)   

 add_message_files(
   FILES
   ivsteercontrol.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ivsteercontrol 
               src/ivsteercontrol.cpp 
               src/wsteercontrol.cpp
               src/filter.cpp)

add_dependencies(ivsteercontrol ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(ivsteercontrol ${catkin_LIBRARIES}
 )


