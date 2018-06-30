#include "NodeClass.hpp"

Node::Node(std::string callerId, int id, uint8_t hashName) {

    //NOTE: ONLY RUN FOLLOWING IF REGISTER TO ROS IS SUCCESSFUL
    this->id = id;
    this->callerId = callerId;
    this->hashName = hashName;
}

Node::~Node() {
    std::cout << "Successfully deallocated node" << '\n';
    //TODO: remove from nodelist if not already
}

int Node::getID() {
    return id;
}

std::string Node::getName() {
    return callerId;
}

static Node * Node::getNode(int index) {
    return nodeList[index];
}

static void Node::registerNode(std::string callerId, uint8_t hashName) {
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

static void Node::deregisterNode(int atID) {
    //NOTE: ONLY RUN FOLLOWING IF DEREGISTER TO ROS IS SUCCESSFUL
    // TODO: call deregisterNode()
    nodeList[atID] = NULL;
}

static bool checkValidMsg(uint8_t id, uint8_t topic){
    int valid = false;

    if(nodeList[id] != NULL){
        //TODO: check topic is valid
        valid = true;
    }

    return valid;
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

int Node::advertiseTopic(std::string topic, std::string topic_type) {

    //TODO: call ROS master registerPublisher(caller_id, topic, topic_type, caller_api) with callerId as first parameter
    std::cout << "Advertising topic" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterSubscriber
    return 0;
}

int Node::unregisterPublisher(std::string topic) {

    //TODO: call ROS master unregisterSubscriber() with callerId as first parameter
    std::cout << "Unregistering publisher" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterPublisher
    return 0;
}

int Node::setParam(std::string key) {

    //TODO: call ROS master setParam(caller_id, key, value) with callerId as first parameter
    std::cout << "Setting parameter" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterPublisher
    return 0;

}

int Node::deleteParam(std::string key) {

    //TODO: call ROS master deleteParam(caller_id, key) with callerId as first parameter
    std::cout << "Deleting parameter" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master deleteParam
    return 0;

}

int Node::advertiseService(std::string service) {

    //TODO: call ROS master registerService(caller_id, service, service_api, caller_api) with callerId as first parameter
    std::cout << "Registering service" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master registerService
    return 0;
}

int Node::unregisterService(std::string service) {

    //TODO: call ROS master unregisterService(caller_id, service, service_api) with callerId as first parameter
    std::cout << "Unregistering service" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unregisterService
    return 0;

}

int Node::searchParam(std::string key) {

    //TODO: call ROS master searchParam(caller_id, key) with callerId as first parameter
    std::cout << "Searching for parameter key" << '\n';

    //TODO: return the CODE to see if found or not found from the ROS master searchParam
    return 0;
}

int Node::subscribeParam(std::string key) {

    //TODO: call ROS master subscribeParam(caller_id, caller_api, key) with callerId as first parameter
    std::cout << "Subscribing to parameter key" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master subscribeParam
    return 0;
}

int Node::unsubscribeParam(std::string key) {

    //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
    std::cout << "Unsubscribing to parameter key" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master unsubscribeParam
    return 0;
}

int Node::hasParam(std::string key) {

    //TODO: call ROS master unsubscribeParam(caller_id, caller_api, key) with callerId as first parameter
    std::cout << "Checking if parameter stored on server" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master hasParam
    return 0;
}

int Node::getParamNames() {

    //TODO: call ROS master getParamNames(caller_id) with callerId as parameter
    std::cout << "Getting list of all parameter names" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master getParamNames
    return 0;
}

int Node::getParam(std::string key) {

    //TODO: call ROS master getParam(caller_id, key) with callerId as parameter
    std::cout << "Getting a parameter" << '\n';

    //TODO: return the CODE to see if success or fail from the ROS master getParam
    return 0;
}
