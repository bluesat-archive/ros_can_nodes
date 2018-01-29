#ifndef ROSCANNODE_H
#define ROSCANNODE_H

#include "connection_manager.h"
#include "poll_manager.h"
#include "topic_manager.h"
#include "xmlrpc_manager.h"
#include <boost/enable_shared_from_this.hpp>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

//#include<ros/subscribe_options.h>

//#include"subscriber.h"

namespace roscan {

    class RosCanNode;
    typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;

    //class Subscriber;
    /*
    class NodeBackingCollection
    {
        public:
            //typedef std::vector<Publisher::ImplWPtr> V_PubImpl;
            //typedef std::vector<ServiceServer::ImplWPtr> V_SrvImpl;
            typedef std::vector<Subscriber> V_Subscriber;
            //typedef std::vector<ServiceClient::ImplWPtr> V_SrvCImpl;
            //V_PubImpl pubs_;
            //V_SrvImpl srvs_;
            V_Subscriber subs_;
            //V_SrvCImpl srv_cs_;

            boost::mutex mutex_;
    };
    */

    class RosCanNode {
        public:
            RosCanNode(std::string name);
            ~RosCanNode();
            XMLRPCManagerPtr xmlrpcManager;
            PollManagerPtr pollManager;
            ConnectionManagerPtr connectionManager;
            TopicManagerPtr topicManager;

            inline const std::string getName() { return name_; }

            //Subscriber subscribe(ros::SubscribeOptions& ops);

            void spinOnce();

            //void subChatterCallback(const boost::shared_ptr<std_msgs::String const>&);

        private:
            std::string name_;
            ros::CallbackQueueInterface *callback_queue_;
            //NodeBackingCollection* collection_;
    };

} // namespace roscan

#endif // ROSCANNODE_H
