cmake_minimum_required(VERSION 2.8.3)
project(ros_can_nodes)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  cpp_common message_generation rosconsole roscpp_serialization roscpp_traits rosgraph_msgs rostime std_msgs xmlrpcpp
)

catkin_package_xml()
# split version in parts and pass to extra file
string(REPLACE "." ";" ros_can_nodes_VERSION_LIST "${ros_can_nodes_VERSION}")
list(LENGTH ros_can_nodes_VERSION_LIST _count)
if(NOT _count EQUAL 3)
  message(FATAL_ERROR "ros_can_nodes version '${ros_can_nodes_VERSION}' does not match 'MAJOR.MINOR.PATCH' pattern")
endif()
list(GET ros_can_nodes_VERSION_LIST 0 ros_can_nodes_VERSION_MAJOR)
list(GET ros_can_nodes_VERSION_LIST 1 ros_can_nodes_VERSION_MINOR)
list(GET ros_can_nodes_VERSION_LIST 2 ros_can_nodes_VERSION_PATCH)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ros/common.h.in ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros/common.h)

find_package(Boost REQUIRED COMPONENTS signals filesystem system)

include_directories(include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES Logger.msg
)

add_service_files(
  DIRECTORY srv
  FILES Empty.srv GetLoggers.srv SetLoggerLevel.srv
)

generate_messages()

set(PTHREAD_LIB "")
find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
  string(LENGTH ${CMAKE_THREAD_LIBS_INIT} _length)
  if(_length GREATER 2)
    string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 0 2 _prefix)
    if(${_prefix} STREQUAL "-l")
      math(EXPR _rest_len "${_length} - 2")
      string(SUBSTRING ${CMAKE_THREAD_LIBS_INIT} 2 ${_rest_len} PTHREAD_LIB)
    endif()
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ros
  LIBRARIES ${PROJECT_NAME}_lib ${PTHREAD_LIB}
  CATKIN_DEPENDS cpp_common message_runtime rosconsole roscpp_serialization roscpp_traits rosgraph_msgs rostime std_msgs xmlrpcpp
  DEPENDS Boost
)

include(CheckIncludeFiles)
include(CheckFunctionExists)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
# Not everybody has trunc (e.g., Windows, embedded arm-linux)
CHECK_FUNCTION_EXISTS(trunc HAVE_TRUNC)

# Output test results to config.h
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/libros/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME}_lib
  src/libros/master.cpp
  src/libros/network.cpp
  src/libros/subscriber.cpp
  src/libros/common.cpp
  src/libros/publisher_link.cpp
  src/libros/connection.cpp
  src/libros/single_subscriber_publisher.cpp
  src/libros/param.cpp
  src/libros/wall_timer.cpp
  src/libros/xmlrpc_manager.cpp
  src/libros/publisher.cpp
  src/libros/timer.cpp
  src/libros/io.cpp
  src/libros/names.cpp
  src/libros/topic.cpp
  src/libros/topic_manager.cpp
  src/libros/poll_manager.cpp
  src/libros/publication.cpp
  src/libros/statistics.cpp
  src/libros/intraprocess_subscriber_link.cpp
  src/libros/intraprocess_publisher_link.cpp
  src/libros/callback_queue.cpp
  src/libros/node_handle.cpp
  src/libros/connection_manager.cpp
  src/libros/file_log.cpp
  src/libros/transport/transport.cpp
  src/libros/transport/transport_udp.cpp
  src/libros/transport/transport_tcp.cpp
  src/libros/subscriber_link.cpp
  src/libros/transport_publisher_link.cpp
  src/libros/transport_subscriber_link.cpp
  src/libros/rosout_appender.cpp
  src/libros/Node.cpp
  src/libros/subscription.cpp
  src/libros/subscription_queue.cpp
  src/libros/spinner.cpp
  src/libros/internal_timer_manager.cpp
  src/libros/message_deserializer.cpp
  src/libros/poll_set.cpp
  )

add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_executable(ros_can_nodes src/main.cpp)
add_dependencies(ros_can_nodes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_can_nodes
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}_lib
  )

