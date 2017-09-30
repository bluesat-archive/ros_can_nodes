

#define MAX_NODES 16

class Node {
public:
    Node(std::string name, int id, std::string hashName);
    ~Node();

    int getID();
    std::string getName();
    static Node * getNode(int id);

    static void registerNode(std::string name, std::string hashId);
    static void deregisterNode(int atID);
    void heartbeat(void);
    int registerSubscriber(std::string topic, std::string topic_type);
    int unregisterSubscriber(std::string topic);
    int advertiseTopic(std::string topic, std::string topic_type);
    int unregisterPublisher(std::string topic);
    int setParam(std::string key);
    int deleteParam(std::string key);
    int advertiseService(std::string service);
    int unregisterService(std::string service);
    int searchParam(std::string key);
    int subscribeParam(std::string key);
    int unsubscribeParam(std::string key);
    int hasParam(std::string key);
    int getParamNames();
    int getParam(std::string key);

private:

    int id;
    std::string callerId;
    std::string hashName;

};
