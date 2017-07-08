cmake_minimum_required(VERSION 2.8.3)
project(ros_can_nodes)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra")

find_package(catkin REQUIRED COMPONENTS
  cpp_common message_generation rosconsole roscpp_serialization roscpp_traits rosgraph_msgs rostime std_msgs xmlrpcpp
)

find_package(Boost REQUIRED COMPONENTS signals filesystem system)

include_directories(include ${Boost_INCLUDE_DIRS})
    #${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_libs
  CATKIN_DEPENDS cpp_common message_runtime rosconsole roscpp_serialization roscpp_traits rosgraph_msgs rostime std_msgs xmlrpcpp
  DEPENDS Boost
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_libs
  src/libs/xmlrpc_manager.cpp
  )

add_executable(ros_can_nodes src/main.cpp)
add_dependencies(ros_can_nodes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_can_nodes
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

