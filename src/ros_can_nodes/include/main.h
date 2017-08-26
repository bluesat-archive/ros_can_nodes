#include<xmlrpc_manager.h>

class RosCanNode {
    public:
        RosCanNode();
        ~RosCanNode();
        ros::XMLRPCManagerPtr xmlrpcManager;
};
