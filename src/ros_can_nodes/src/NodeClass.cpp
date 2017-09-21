#include <ctime>
#include <iostream>
#include "NodeClass.hpp"

//array of nodes
Node *nodeList[MAX_NODES];

Node::Node(std::string callerId, int id, std::string hashName) {

    //NOTE: ONLY RUN FOLLOWING IF REGISTER TO ROS IS SUCCESSFUL
    this->id = id;
    this->callerId = callerId;
    this->hashName = hashName;

}

Node::~Node() {
    std::cout << "Successfully deallocated node" << '\n';
}

int Node::getID() {
    return id;
}

std::string Node::getName() {
    return callerId;
}

void Node::registerNode(std::string callerId, std::string hashName) {
    int index = 0;
    //get the first not set id
    while (index < MAX_NODES && nodeList[index] != NULL) {
        index++;
        if (index >= MAX_NODES) {
            std::cout << "No available ids" << '\n';
            break;
        }
    }

    if(index < MAX_NODES) {
        //TODO: call the constructor
        Node *node = new Node(callerId, index, hashName);
        nodeList[index] = node;
    }
}

void Node::deregisterNode(int atID) {
    //NOTE: ONLY RUN FOLLOWING IF DEREGISTER TO ROS IS SUCCESSFUL
    // TODO: call deregisterNode()
    nodeList[atID] = NULL;
}

//NOTE: do at a later date
void Node::heartbeat() {
    time_t t = time(0);
    struct tm * now = localtime( & t );
    std::cout   << now->tm_hour << ":"
    << now->tm_min
    << std::endl;
}

int Node::registerSubscriber(std::string topic, std::string topic_type) {

    // TODO: call ROS master registerSubscriber() with callerId as first parameter
    std::cout << "Calling subscriber" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master registerSubscriber
        //-1: ERROR: Error on the part of the caller, e.g. an invalid parameter. In general, this means that the master/slave did not attempt to execute the action.
        //0: FAILURE: Method failed to complete correctly. In general, this means that the master/slave attempted the action and failed, and there may have been side-effects as a result.
        //1: SUCCESS: Method completed successfully
    return 0;

}

int Node::unregisterSubscriber(std::string topic) {

    //TODO: call ROS master unregisterSubscriber() with callerId as first parameter
    std::cout << "Unregistering subscriber" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterSubscriber
    return 0;
}

int advertiseTopic(std::string topic, std::string topic_type) {

    //TODO: call ROS master registerPublisher(caller_id, topic, topic_type, caller_api) with callerId as first parameter
    std::cout << "Advertising topic" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterSubscriber
    return 0;
}

int unregisterPublisher(std::string topic) {

    //TODO: call ROS master unregisterSubscriber() with callerId as first parameter
    std::cout << "Unregistering publisher" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterPublisher
    return 0;
}

int setParam(std::string key) {

    //TODO: call ROS master setParam(caller_id, key, value) with callerId as first parameter
    std::cout << "Setting parameter" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterPublisher
    return 0;

}

int deleteParam(std::string key) {

    //TODO: call ROS master deleteParam(caller_id, key) with callerId as first parameter
    std::cout << "Deleting parameter" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master deleteParam
    return 0;

}

int advertiseService(std::string service) {

    //TODO: call ROS master registerService(caller_id, service, service_api, caller_api) with callerId as first parameter
    std::cout << "Registering service" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master registerService
    return 0;
}

int unregisterService(std::string service) {

    //TODO: call ROS master unregisterService(caller_id, service, service_api) with callerId as first parameter
    std::cout << "Unregistering service" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterService
    return 0;

}

int searchParam(std::string key) {

    //TODO: call ROS master searchParam(caller_id, key) with callerId as first parameter
    std::cout << "Searching for parameter key" << '\n';

    //TODO: return the CODE to see if found or not found from the ROS master searchParam
    return 0;
}

int subscribeParam(std::string key) {

    //TODO: call ROS master subscribeParam(caller_id, caller_api, key) with callerId as first parameter
    std::cout << "Subscribing to parameter key" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master subscribeParam
    return 0;
}

int unsubscribeParam(std::string key) {

    //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
    std::cout << "Unsubscribing to parameter key" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unsubscribeParam
    return 0;
}

int hasParam(std::string key) {

    //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
    std::cout << "Checking if parameter stored on server" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master hasParam
    return 0;
}

int getParamNames() {

    //TODO: call ROS master getParamNames(caller_id) with callerId as parameter
    std::cout << "Getting list of all parameter names" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master getParamNames
    return 0;
}

int getParam(std::string key) {

    //TODO: call ROS master getParam(caller_id, key) with callerId as parameter
    std::cout << "Getting a parameter" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master getParam
    return 0;
}

Node * getNode(int index) {
    return nodeList[index];
}

int main() {
    for (int i = 0; i < MAX_NODES + 5; i++) {
      Node::registerNode("Node " + std::to_string(i), "random hashID");

      if(i < MAX_NODES){
          std::cout << nodeList[i]->getID() << '\n';
          std::cout << nodeList[i]->getName();
          std::cout << "" << '\n';
      }

    }
}