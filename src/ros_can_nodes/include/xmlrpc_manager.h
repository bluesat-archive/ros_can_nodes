/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCAN_XMLRPC_MANAGER_H
#define ROSCAN_XMLRPC_MANAGER_H

#include <cstdint>
#include <set>
#include <string>
#include <vector>
#include <map>
#include <XmlRpc.h>
#include <XmlRpcValue.h>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ros/time.h>

namespace roscan {

namespace xmlrpc {

XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response);
XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response);
XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response);

} // namespace xmlrpc

class XMLRPCCallWrapper;
typedef boost::shared_ptr<XMLRPCCallWrapper> XMLRPCCallWrapperPtr;

class ASyncXMLRPCConnection : public boost::enable_shared_from_this<ASyncXMLRPCConnection> {
    public:
        virtual ~ASyncXMLRPCConnection() {}

        virtual void addToDispatch(XmlRpc::XmlRpcDispatch* disp) = 0;
        virtual void removeFromDispatch(XmlRpc::XmlRpcDispatch* disp) = 0;

        virtual bool check() = 0;
};
typedef boost::shared_ptr<ASyncXMLRPCConnection> ASyncXMLRPCConnectionPtr;
typedef std::set<ASyncXMLRPCConnectionPtr> S_ASyncXMLRPCConnection;

class CachedXmlRpcClient {
    public:
        CachedXmlRpcClient(XmlRpc::XmlRpcClient* c) : in_use_(false), client_(c) {}

        bool in_use_;
        ros::WallTime last_use_time_; // for reaping
        XmlRpc::XmlRpcClient* client_;

        static const ros::WallDuration s_zombie_time_; // how long before it is toasted
};

class XMLRPCManager;
typedef boost::shared_ptr<XMLRPCManager> XMLRPCManagerPtr;

typedef boost::function<void(XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&)> XMLRPCFunc;

class XMLRPCManager {
    public:
        XMLRPCManager() : port_(0), shutting_down_(false), unbind_requested_(false) {}
        ~XMLRPCManager() { shutdown(); }

        // Validate an XML/RPC response
        // @param method The RPC method that was invoked.
        // @param response The resonse that was received.
        // @param payload The payload that was received.
        // Return true if validation succeeds, false otherwise.
        // @todo Consider making this private.
        bool validateXmlrpcResponse(const std::string& method, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload);

        // Get the xmlrpc server URI of this node
        inline const std::string& getServerURI() const { return uri_; }
        inline uint32_t getServerPort() const { return port_; }

        XmlRpc::XmlRpcClient* getXMLRPCClient(const std::string& host, const int port, const std::string& uri);
        void releaseXMLRPCClient(XmlRpc::XmlRpcClient* c);

        void addASyncConnection(const ASyncXMLRPCConnectionPtr& conn);
        void removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn);

        bool bind(const std::string& function_name, const XMLRPCFunc& cb);
        void unbind(const std::string& function_name);

        void start();
        void shutdown();

        bool isShuttingDown() { return shutting_down_; }

        // Execute an XMLRPC call on the master
        // @param method The RPC method to invoke
        // @param request The arguments to the RPC call
        // @param response [out] The resonse that was received.
        // @param payload [out] The payload that was received.
        // @param wait_for_master Whether or not this call should loop until it can contact the master
        // Returns true if call succeeds, false otherwise.
        // Relevant reading: http://wiki.ros.org/ROS/Master_API
        bool callMaster(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master);

        // Get the hostname where the master runs.
        const std::string& getMasterHost();

        // Get the port where the master runs.
        uint32_t getMasterPort();

        // Get the full URI to the master (eg. http://host:port/)
        const std::string& getMasterURI();

        // Check whether the master is up
        // This method tries to contact the master.  You can call it any time
        // after ros::init has been called.  The intended usage is to check
        // whether the master is up before trying to make other requests
        // (subscriptions, advertisements, etc.).
        // Returns true if the master is available, false otherwise.
        bool checkMaster();

        // Contains information retrieved from the master about a topic
        struct TopicInfo {
            TopicInfo() {}
            TopicInfo(const std::string& _name, const std::string& _datatype) : name(_name), datatype(_datatype) {}
            std::string name;     // Name of the topic
            std::string datatype; // Datatype of the topic
        };
        typedef std::vector<TopicInfo> V_TopicInfo;

        // Get the list of topics that are being published by all nodes.
        // This method communicates with the master to retrieve the list of all
        // currently advertised topics.
        // @param subgraph The name of the subgraph to restrict topic search space, "" for all topics
        // @param topics A place to store the resulting list.  Each item in the
        // list is a pair <string topic, string type>.  The type is represented
        // in the format "package_name/MessageName", and is also retrievable
        // through message.__getDataType() or MessageName::__s_getDataType().
        // Return true on success, false otherwise (topics not filled in)
        bool getAllTopics(std::string subgraph, V_TopicInfo& topics);

        // Retreives the currently-known list of nodes from the master
        bool getAllNodes(std::vector<std::string>& nodes);

        // Set the max time this node should spend looping trying to connect to the master
        // @param The timeout.  A negative value means infinite
        void setMasterRetryTimeout(ros::WallDuration timeout);

    private:
        void serverThreadFunc();

        std::string uri_;
        int port_;
        boost::thread server_thread_;

        XmlRpc::XmlRpcServer server_;
        typedef std::vector<CachedXmlRpcClient> V_CachedXmlRpcClient;
        V_CachedXmlRpcClient clients_;
        boost::mutex clients_mutex_;

        bool shutting_down_;

        ros::WallDuration master_retry_timeout_;

        S_ASyncXMLRPCConnection added_connections_;
        boost::mutex added_connections_mutex_;
        S_ASyncXMLRPCConnection removed_connections_;
        boost::mutex removed_connections_mutex_;

        S_ASyncXMLRPCConnection connections_;

        struct FunctionInfo {
            std::string name;
            XMLRPCFunc function;
            XMLRPCCallWrapperPtr wrapper;
        };
        typedef std::map<std::string, FunctionInfo> M_StringToFuncInfo;
        boost::mutex functions_mutex_;
        M_StringToFuncInfo functions_;

        volatile bool unbind_requested_;
};

} // namespace roscan

#endif // ROSCAN_XMLRPC_MANAGER_H
