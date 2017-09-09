#ifndef ROSCANNODE_H_
#define ROSCANNODE_H_

#include<xmlrpc_manager.h>
#include<poll_manager.h>
#include<connection_manager.h>
#include<topic_manager.h>

#include <boost/enable_shared_from_this.hpp>

namespace roscan
{

    class RosCanNode {
        public:
            RosCanNode(std::string name);
            ~RosCanNode();
            XMLRPCManagerPtr xmlrpcManager;
            PollManagerPtr pollManager;
            ConnectionManagerPtr connectionManager;
            TopicManagerPtr topicManager;

            inline const std::string getName() {
                return name_;
            }

        private:
            std::string name_;

    };

    typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;
}

#endif
