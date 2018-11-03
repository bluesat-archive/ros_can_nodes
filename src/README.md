# ROS CAN Node

Provides a proxy for ROS messages going on to the CAN bus.

## Structure

### src

Code for the proxy

#### ros_node_lib

All the files need to compile the original NodeHandle classes (modify roscpp).

Most of the time doesn't need to be edited. Handles low level interactions with ROS Master

### include 

Header files

# Nodes

- RosCanNodeTest -> testing ROSCanNode itself. Testing if publishing works, etc internally.
- CanMessageRouter.cpp -> Main Entry point for the proxy