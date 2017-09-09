#ifndef ROSCANNODE_H_
#define ROSCANNODE_H_

#include<xmlrpc_manager.h>
#include<poll_manager.h>
#include<connection_manager.h>
//#include<topic_manager.h>

namespace roscan
{

    class RosCanNode {
        public:
            RosCanNode();
            ~RosCanNode();
            roscan::XMLRPCManagerPtr xmlrpcManager;
            roscan::PollManagerPtr pollManager;
            roscan::ConnectionManagerPtr connectionManager;
            //roscan::TopicManagerPtr topicManager;
    };
}

#endif
