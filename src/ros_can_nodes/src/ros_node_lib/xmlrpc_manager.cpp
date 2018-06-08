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

#include "common.h"
#include "xmlrpc_manager.h"
#include "network.h"
#include <XmlRpc.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/file_log.h>
#include <ros/io.h>
#include <ros/param.h>

using namespace XmlRpc;

namespace roscan {

namespace xmlrpc {

XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response) {
    XmlRpc::XmlRpcValue v;
    v[0] = code;
    v[1] = msg;
    v[2] = response;
    return v;
}

XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response) {
    XmlRpc::XmlRpcValue v;
    v[0] = int(code);
    v[1] = msg;
    v[2] = response;
    return v;
}

XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response) {
    XmlRpc::XmlRpcValue v;
    v[0] = int(code);
    v[1] = msg;
    v[2] = XmlRpc::XmlRpcValue(response);
    return v;
}

} // namespace xmlrpc

class XMLRPCCallWrapper : public XmlRpcServerMethod {
    public:
        XMLRPCCallWrapper(const std::string& function_name, const XMLRPCFunc& cb, XmlRpcServer* s)
            : XmlRpcServerMethod(function_name, s), name_(function_name), func_(cb) {}

        void execute(XmlRpcValue& params, XmlRpcValue& result) {
            func_(params, result);
        }

    private:
        std::string name_;
        XMLRPCFunc func_;
};

void getPid(const XmlRpcValue& params, XmlRpcValue& result) {
    (void)params;
    result = xmlrpc::responseInt(1, "", (int)getpid());
}

const ros::WallDuration CachedXmlRpcClient::s_zombie_time_(30.0); // reap after 30 seconds

uint32_t g_port = 0;
std::string g_host;
std::string g_uri;
ros::WallDuration g_retry_timeout;

void XMLRPCManager::start() {
    if (g_uri.empty()) {
        char* master_uri_env = getenv("ROS_MASTER_URI");
        if (!master_uri_env) {
            ROS_FATAL("ROS_MASTER_URI is not defined in the environment. Either "
                      "type the following or (preferrably) add this to your "
                      "~/.bashrc file in order set up your "
                      "local machine as a ROS master:\n\n"
                      "export ROS_MASTER_URI=http://localhost:11311\n\n"
                      "then, type 'roscore' in another shell to actually launch "
                      "the master program.");
            ROS_BREAK();
        }

        g_uri = master_uri_env;
    }

    // Split URI into
    if (!network::splitURI(g_uri, g_host, g_port)) {
        ROS_FATAL("Couldn't parse the master URI [%s] into a host:port pair.", g_uri.c_str());
        ROS_BREAK();
    }

    shutting_down_ = false;
    port_ = 0;
    bind("getPid", getPid);

    bool bound = server_.bindAndListen(0);
    (void)bound;
    ROS_ASSERT(bound);
    port_ = server_.get_port();
    ROS_ASSERT(port_ != 0);

    std::stringstream ss;
    // TODO: modify network so we can init() it so this works
    ss << "http://" << network::getHost() << ":" << port_ << "/";
    //ss << "http://localhost:" << port_ << "/";
    uri_ = ss.str();

    server_thread_ = boost::thread(boost::bind(&XMLRPCManager::serverThreadFunc, this));
}

void XMLRPCManager::shutdown() {
    if (shutting_down_) {
        return;
    }

    shutting_down_ = true;
    server_thread_.join();

    server_.close();

    // kill the last few clients that were started in the shutdown process
    {
        boost::mutex::scoped_lock lock(clients_mutex_);

        for (V_CachedXmlRpcClient::iterator i = clients_.begin(); i != clients_.end();) {
            if (!i->in_use_) {
                i->client_->close();
                delete i->client_;
                i = clients_.erase(i);
            } else {
                ++i;
            }
        }
    }

    // Wait for the clients that are in use to finish and remove themselves from clients_
    for (int wait_count = 0; !clients_.empty() && wait_count < 10; wait_count++) {
        ROSCPP_LOG_DEBUG("waiting for xmlrpc connection to finish...");
        ros::WallDuration(0.01).sleep();
    }

    boost::mutex::scoped_lock lock(functions_mutex_);
    functions_.clear();

    {
        S_ASyncXMLRPCConnection::iterator it = connections_.begin();
        S_ASyncXMLRPCConnection::iterator end = connections_.end();
        for (; it != end; ++it) {
            (*it)->removeFromDispatch(server_.get_dispatch());
        }
    }

    connections_.clear();

    {
        boost::mutex::scoped_lock lock(added_connections_mutex_);
        added_connections_.clear();
    }

    {
        boost::mutex::scoped_lock lock(removed_connections_mutex_);
        removed_connections_.clear();
    }
}

bool XMLRPCManager::validateXmlrpcResponse(const std::string& method, XmlRpcValue& response, XmlRpcValue& payload) {
    if (response.getType() != XmlRpcValue::TypeArray) {
        ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return an array", method.c_str());
        return false;
    }
    if (response.size() != 2 && response.size() != 3) {
        ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a 2 or 3-element array", method.c_str());
        return false;
    }
    if (response[0].getType() != XmlRpcValue::TypeInt) {
        ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a int as the 1st element", method.c_str());
        return false;
    }
    int status_code = response[0];
    if (response[1].getType() != XmlRpcValue::TypeString) {
        ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a string as the 2nd element", method.c_str());
        return false;
    }
    std::string status_string = response[1];
    if (status_code != 1) {
        ROSCPP_LOG_DEBUG("XML-RPC call [%s] returned an error (%d): [%s]", method.c_str(), status_code, status_string.c_str());
        return false;
    }
    if (response.size() > 2) {
        payload = response[2];
    } else {
        std::string empty_array = "<value><array><data></data></array></value>";
        int offset = 0;
        payload = XmlRpcValue(empty_array, &offset);
    }
    return true;
}

void XMLRPCManager::serverThreadFunc() {
    ros::disableAllSignalsInThisThread();

    while (!shutting_down_) {
        {
            boost::mutex::scoped_lock lock(added_connections_mutex_);
            S_ASyncXMLRPCConnection::iterator it = added_connections_.begin();
            S_ASyncXMLRPCConnection::iterator end = added_connections_.end();
            for (; it != end; ++it) {
                (*it)->addToDispatch(server_.get_dispatch());
                connections_.insert(*it);
            }

            added_connections_.clear();
        }

        // Update the XMLRPC server, blocking for at most 100ms in select()
        {
            boost::mutex::scoped_lock lock(functions_mutex_);
            server_.work(0.1);
        }

        while (unbind_requested_) {
            ros::WallDuration(0.01).sleep();
        }

        if (shutting_down_) {
            return;
        }

        {
            S_ASyncXMLRPCConnection::iterator it = connections_.begin();
            S_ASyncXMLRPCConnection::iterator end = connections_.end();
            for (; it != end; ++it) {
                if ((*it)->check()) {
                    removeASyncConnection(*it);
                }
            }
        }

        {
            boost::mutex::scoped_lock lock(removed_connections_mutex_);
            S_ASyncXMLRPCConnection::iterator it = removed_connections_.begin();
            S_ASyncXMLRPCConnection::iterator end = removed_connections_.end();
            for (; it != end; ++it) {
                (*it)->removeFromDispatch(server_.get_dispatch());
                connections_.erase(*it);
            }

            removed_connections_.clear();
        }
    }
}

XmlRpcClient* XMLRPCManager::getXMLRPCClient(const std::string& host, const int port, const std::string& uri) {
    // go through our vector of clients and grab the first available one
    XmlRpcClient* c = NULL;

    boost::mutex::scoped_lock lock(clients_mutex_);

    for (V_CachedXmlRpcClient::iterator i = clients_.begin(); !c && i != clients_.end();) {
        if (!i->in_use_) {
            // see where it's pointing
            if (i->client_->getHost() == host && i->client_->getPort() == port && i->client_->getUri() == uri) {
                // hooray, it's pointing at our destination. re-use it.
                c = i->client_;
                i->in_use_ = true;
                i->last_use_time_ = ros::WallTime::now();
                break;
            } else if (i->last_use_time_ + CachedXmlRpcClient::s_zombie_time_ < ros::WallTime::now()) {
                // toast this guy. he's dead and nobody is reusing him.
                delete i->client_;
                i = clients_.erase(i);
            } else {
                ++i; // move along. this guy isn't dead yet.
            }
        } else {
            ++i;
        }
    }

    if (!c) {
        // allocate a new one
        c = new XmlRpcClient(host.c_str(), port, uri.c_str());
        CachedXmlRpcClient mc(c);
        mc.in_use_ = true;
        mc.last_use_time_ = ros::WallTime::now();
        clients_.push_back(mc);
        //ROS_INFO("%d xmlrpc clients allocated\n", xmlrpc_clients.size());
    }
    // ONUS IS ON THE RECEIVER TO UNSET THE IN_USE FLAG
    // by calling releaseXMLRPCClient
    return c;
}

void XMLRPCManager::releaseXMLRPCClient(XmlRpcClient* c) {
    boost::mutex::scoped_lock lock(clients_mutex_);

    for (V_CachedXmlRpcClient::iterator i = clients_.begin(); i != clients_.end(); ++i) {
        if (c == i->client_) {
            if (shutting_down_) {
                // if we are shutting down we won't be re-using the client
                i->client_->close();
                delete i->client_;
                clients_.erase(i);
            } else {
                i->in_use_ = false;
            }
            break;
        }
    }
}

void XMLRPCManager::addASyncConnection(const ASyncXMLRPCConnectionPtr& conn) {
    boost::mutex::scoped_lock lock(added_connections_mutex_);
    added_connections_.insert(conn);
}

void XMLRPCManager::removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn) {
    boost::mutex::scoped_lock lock(removed_connections_mutex_);
    removed_connections_.insert(conn);
}

bool XMLRPCManager::bind(const std::string& function_name, const XMLRPCFunc& cb) {
    boost::mutex::scoped_lock lock(functions_mutex_);
    if (functions_.find(function_name) != functions_.end()) {
        return false;
    }

    FunctionInfo info;
    info.name = function_name;
    info.function = cb;
    info.wrapper.reset(new XMLRPCCallWrapper(function_name, cb, &server_));
    functions_[function_name] = info;

    return true;
}

void XMLRPCManager::unbind(const std::string& function_name) {
    unbind_requested_ = true;
    boost::mutex::scoped_lock lock(functions_mutex_);
    functions_.erase(function_name);
    unbind_requested_ = false;
}

const std::string& XMLRPCManager::getMasterHost() {
    return g_host;
}

uint32_t XMLRPCManager::getMasterPort() {
    return g_port;
}

const std::string& XMLRPCManager::getMasterURI() {
    return g_uri;
}

void XMLRPCManager::setMasterRetryTimeout(ros::WallDuration timeout) {
    if (timeout < ros::WallDuration(0)) {
        ROS_FATAL("retry timeout must not be negative.");
        ROS_BREAK();
    }
    g_retry_timeout = timeout;
}

bool XMLRPCManager::checkMaster() {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = "";
    return callMaster("getPid", args, result, payload, false);
}

bool XMLRPCManager::getAllTopics(std::string subgraph, V_TopicInfo& topics) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ""; // caller_id
    args[1] = subgraph;

    if (!callMaster("getPublishedTopics", args, result, payload, true)) {
        return false;
    }

    topics.clear();
    for (int i = 0; i < payload.size(); i++) {
        topics.push_back(TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
    }

    return true;
}

bool XMLRPCManager::getAllNodes(std::vector<std::string>& nodes) {
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ""; // caller_id

    if (!callMaster("getSystemState", args, result, payload, true)) {
        return false;
    }

    std::set<std::string> node_set;
    for (int i = 0; i < payload.size(); ++i) {
        for (int j = 0; j < payload[i].size(); ++j) {
            for (int k = 0; k < payload[i][j][1].size(); ++k) {
                node_set.insert(std::string(payload[i][j][1][k]));
            }
        }
    }

    nodes.clear();
    std::copy(node_set.begin(), node_set.end(), std::back_inserter(nodes));

    return true;
}

bool XMLRPCManager::callMaster(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master) {
    ros::WallTime start_time = ros::WallTime::now();

    std::string master_host = getMasterHost();
    uint32_t master_port = getMasterPort();
    XmlRpc::XmlRpcClient* c = getXMLRPCClient(master_host, master_port, "/");
    bool printed = false;
    bool slept = false;
    bool ok = true;
    bool b = false;
    do {
        b = c->execute(method.c_str(), request, response);
        ok = !isShuttingDown();

        if (!b && ok) {
            if (!printed && wait_for_master) {
                ROS_ERROR("[%s] Failed to contact master at [%s:%d].  %s", method.c_str(), master_host.c_str(), master_port, wait_for_master ? "Retrying..." : "");
                printed = true;
            }

            if (!wait_for_master) {
                releaseXMLRPCClient(c);
                return false;
            }

            if (!g_retry_timeout.isZero() && (ros::WallTime::now() - start_time) >= g_retry_timeout) {
                ROS_ERROR("[%s] Timed out trying to connect to the master after [%f] seconds", method.c_str(), g_retry_timeout.toSec());
                releaseXMLRPCClient(c);
                return false;
            }

            ros::WallDuration(0.05).sleep();
            slept = true;
        } else {
            if (!validateXmlrpcResponse(method, response, payload)) {
                releaseXMLRPCClient(c);

                return false;
            }

            break;
        }

        ok = !isShuttingDown();
    } while (ok);

    if (ok && slept) {
        ROS_INFO("Connected to master at [%s:%d]", master_host.c_str(), master_port);
    }

    releaseXMLRPCClient(c);

    return b;
}

} // namespace roscan
