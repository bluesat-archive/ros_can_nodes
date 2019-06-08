cmake_minimum_required(VERSION 2.8.3)
project(ros_can_nodes)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(MESSAGE_PACKAGES
  std_msgs
  owr_messages
)
find_package(catkin REQUIRED COMPONENTS
  ros_type_introspection
  ${MESSAGE_PACKAGES}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS ros_type_introspection
#  DEPENDS Boost
)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_message_properties_map.cmake)
generate_message_properties_map(${MESSAGE_PACKAGES})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    include/ros_node_lib
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/message_properties_map/include
)

## Declare a C++ library
# add_library(ros_can_nodes
#   src/${PROJECT_NAME}/ros_can_nodes.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(ros_can_nodes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(ros_can_nodes
    src/RosCanNode.cpp
    src/TopicBuffers.cpp
    src/CANHelpers.cpp
    src/CANMsgRouter.cpp
    src/MessageBuffer.cpp
    src/RosCanNodeManager.cpp
    src/IntrospectionHelpers.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/message_properties_map/src/message_properties_map.cpp"
    src/ros_node_lib/xmlrpc_manager.cpp
    src/ros_node_lib/network.cpp
    src/ros_node_lib/poll_manager.cpp
    src/ros_node_lib/connection_manager.cpp
    src/ros_node_lib/topic_manager.cpp
    src/ros_node_lib/publication.cpp
    src/ros_node_lib/publisher.cpp
    src/ros_node_lib/publisher_link.cpp
    src/ros_node_lib/transport_publisher_link.cpp
    src/ros_node_lib/intraprocess_publisher_link.cpp
    src/ros_node_lib/subscription.cpp
    src/ros_node_lib/subscriber_link.cpp
    src/ros_node_lib/transport_subscriber_link.cpp
    src/ros_node_lib/intraprocess_subscriber_link.cpp
    src/ros_node_lib/subscription_queue.cpp
    src/ros_node_lib/callback_queue.cpp
    src/ros_node_lib/rosout_appender.cpp
    src/ros_node_lib/internal_timer_manager.cpp
    src/ros_node_lib/RosNode.cpp
    )

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(ros_can_nodes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ros_can_nodes
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS ros_can_nodes ros_can_nodes
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_can_nodes.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
