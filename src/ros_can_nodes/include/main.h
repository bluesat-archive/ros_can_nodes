#include<xmlrpc_manager.h>

class RosCanNode {
    public:
        RosCanNode();
        ~RosCanNode();
        roscan::XMLRPCManagerPtr xmlrpcManager;
};
