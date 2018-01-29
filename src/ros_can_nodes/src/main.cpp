#include "RosCanNode.h"
#include "network.h"
#include <boost/make_shared.hpp>
//#include "ros/ros.h"
#include <unistd.h>

namespace roscan {

    RosCanNode::RosCanNode(std::string name) {
        name_ = name;

        std::cout << "  Creating xmlrpc manager\n";
        xmlrpcManager.reset(new XMLRPCManager);

        std::cout << "  Creating poll manager\n";
        pollManager.reset(new PollManager);
        std::cout << "  Starting poll manager\n";
        pollManager->start();

        std::cout << "  Creating connection manager\n";
        connectionManager.reset(new ConnectionManager);
        std::cout << "  Starting connection manager\n";
        connectionManager->start(*this);

        std::cout << "  Creating topic manager\n";
        RosCanNodePtr nodeptr = boost::make_shared<RosCanNode>(*this);
        topicManager.reset(new TopicManager);
        std::cout << "  Starting topic manager\n";
        topicManager->start(nodeptr);
        std::cout << "  Done!\n";

        // xmlrpc manager must be started _after_ all functions are bound to it
        std::cout << "  Starting xmlrpc manager\n";
        xmlrpcManager->start();

        //collection_ = new NodeBackingCollection;
        callback_queue_ = new ros::CallbackQueue;
    }

    /*
    void RosCanNode::subChatterCallback(const boost::shared_ptr<std_msgs::String const>& msg) {
        std::cout << "received " << msg->data << std::endl;
    }

    Subscriber RosCanNode::subscribe(ros::SubscribeOptions& ops) {

        if (ops.callback_queue == 0) {
            if (callback_queue_) {
                ops.callback_queue = callback_queue_;
            } else {
                std::cout << "argh no callback queue\n";
                return Subscriber();
            }
        }

        if (topicManager->subscribe(ops)) {
            Subscriber sub(ops.topic, boost::make_shared<RosCanNode>(*this), ops.helper);

            {
                boost::mutex::scoped_lock lock(collection_->mutex_);
                collection_->subs_.push_back(sub);
            }

            return sub;
        }

        return Subscriber();
    }

    void RosCanNode::spinOnce() {
        ((ros::CallbackQueue*)callback_queue_)->callAvailable(ros::WallDuration());
    }
    */

} // namespace roscan

using namespace roscan;

// main has to be in the global namespace lol
int main() {
    network::init();

    std::cout << "Creating new node\n";
    RosCanNodePtr node;
    node.reset(new RosCanNode("bob"));
    std::cout << node->xmlrpcManager->getServerURI() << std::endl;

    // mini shell for testing lel
    while (1) {
        std::cout << "$ ";
        std::string s;
        std::cin >> s;
        if (s == "exit") {
            break;
        } else if (s == "checkMaster") {
            std::cout << node->xmlrpcManager->checkMaster("yo") << std::endl;
        } else if (s == "getMasterURI") {
            std::cout << node->xmlrpcManager->getMasterURI() << std::endl;
        } else if (s == "getAllNodes") {
            std::vector<std::string> nodes;
            if (node->xmlrpcManager->getAllNodes("yo", nodes)) {
                for (unsigned i = 0; i < nodes.size(); ++i) {
                    std::cout << nodes[i] << std::endl;
                }
            } else {
                std::cout << "getAllNodes() failed!" << std::endl;
            }
        } else if (s == "getAllTopics") {
            std::vector<XMLRPCManager::TopicInfo> topics;
            if (node->xmlrpcManager->getAllTopics("yo", topics)) {
                for (unsigned i = 0; i < topics.size(); ++i) {
                    std::cout << topics[i].name << " (" << topics[i].datatype << ")" << std::endl;
                }
            } else {
                std::cout << "getAllTopics() failed!" << std::endl;
            }
        }
        /*
        if (s == "subscribe") {
            ros::SubscribeOptions ops;
            ops.template init<std_msgs::String>("/chatter", 1, boost::bind(&roscan::RosCanNode::subChatterCallback, node, _1));
            ops.transport_hints = ros::TransportHints();
            node->subscribe(ops);
            while(1) {
                // spin on callbacks
                node->spinOnce();
                sleep(1);
            }
        }*/
    }

    return 0;
}
