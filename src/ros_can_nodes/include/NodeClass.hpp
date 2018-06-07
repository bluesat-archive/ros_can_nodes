

#define MAX_NODES 16

class Node {

    public:
        Node(std::string name, int id, uint hashName);
        ~Node();

        int getID();
        std::string getName();

        static Node * getNode(int id);
        static void registerNode(std::string name, uint hashName);
        static void deregisterNode(int atID);
        static bool checkValidMsg(uint id, uint topic);

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
        uint hashName;
        static Node *nodeList[MAX_NODES];

};
