/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
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

#include "publication.h"
#include "subscriber_link.h"
#include "single_subscriber_publisher.h"
#include "callback_queue_interface.h"
#include <ros/connection.h>
#include <ros/serialization.h>
#include <std_msgs/Header.h>

namespace roscan {

class PeerConnDisconnCallback : public CallbackInterface {
    public:
        PeerConnDisconnCallback(const SubscriberStatusCallback& callback, const SubscriberLinkPtr& sub_link, const bool use_tracked_object, const ros::VoidConstWPtr& tracked_object)
            : callback_{callback}, sub_link_{sub_link}, use_tracked_object_{use_tracked_object}, tracked_object_{tracked_object} {}

        CallbackInterface::CallResult call() override {
            if (use_tracked_object_) {
                const auto tracker = tracked_object_.lock();
                if (!tracker) {
                    return Invalid;
                }
            }
            SingleSubscriberPublisher pub{sub_link_};
            callback_(pub);
            return CallbackInterface::Success;
        }

    private:
        SubscriberStatusCallback callback_;
        SubscriberLinkPtr sub_link_;
        bool use_tracked_object_;
        ros::VoidConstWPtr tracked_object_;
};

void Publication::addCallbacks(const SubscriberCallbacksPtr& callbacks) {
    std::lock_guard<std::mutex> lock{callbacks_mutex_};

    callbacks_.push_back(callbacks);

    // Add connect callbacks for all current subscriptions if this publisher wants them
    if (callbacks->connect_ && callbacks->callback_queue_) {
        std::lock_guard<std::mutex> lock{subscriber_links_mutex_};
        for (const auto& sub_link: subscriber_links_) {
            CallbackInterfacePtr cb{boost::make_shared<PeerConnDisconnCallback>(callbacks->connect_, sub_link, callbacks->has_tracked_object_, callbacks->tracked_object_)};
            callbacks->callback_queue_->addCallback(cb, (uint64_t)callbacks.get());
        }
    }
}

void Publication::removeCallbacks(const SubscriberCallbacksPtr& callbacks) {
    std::lock_guard<std::mutex> lock{callbacks_mutex_};

    const auto it = std::find(callbacks_.begin(), callbacks_.end(), callbacks);
    if (it != callbacks_.end()) {
        if ((*it)->callback_queue_) {
            (*it)->callback_queue_->removeByID((uint64_t)it->get());
        }
        callbacks_.erase(it);
    }
}

void Publication::drop() {
    // grab a lock here, to ensure that no subscription callback will
    // be invoked after we return
    {
        std::lock_guard<std::mutex> lock{publish_queue_mutex_};
        std::lock_guard<std::mutex> lock2{subscriber_links_mutex_};

        if (dropped_) {
            return;
        }
        dropped_ = true;
    }
    dropAllConnections();
}

bool Publication::enqueueMessage(const ros::SerializedMessage& m) {
    std::lock_guard<std::mutex> lock{subscriber_links_mutex_};
    if (dropped_) {
        return false;
    }

    const auto seq = incrementSequence();
    if (has_header_) {
        // If we have a header, we know it's immediately after the message length
        // Deserialize it, write the sequence, and then serialize it again.
        namespace ser = ros::serialization;
        std_msgs::Header header;
        ser::IStream istream{m.buf.get() + 4, (uint32_t)m.num_bytes - 4};
        ser::deserialize(istream, header);
        header.seq = seq;
        ser::OStream ostream{m.buf.get() + 4, (uint32_t)m.num_bytes - 4};
        ser::serialize(ostream, header);
    }

    for (const auto& sub_link: subscriber_links_) {
        sub_link->enqueueMessage(m, true, false);
    }

    if (latch_) {
        last_message_ = m;
    }
    return true;
}

void Publication::addSubscriberLink(const SubscriberLinkPtr& sub_link) {
    {
        std::lock_guard<std::mutex> lock{subscriber_links_mutex_};

        if (dropped_) {
            return;
        }

        subscriber_links_.push_back(sub_link);

        if (sub_link->isIntraprocess()) {
            ++intraprocess_subscriber_count_;
        }
    }

    if (latch_ && last_message_.buf) {
        sub_link->enqueueMessage(last_message_, true, true);
    }

    // This call invokes the subscribe callback if there is one.
    // This must happen *after* the push_back above, in case the
    // callback uses publish().
    peerConnect(sub_link);
}

void Publication::removeSubscriberLink(const SubscriberLinkPtr& sub_link) {
    SubscriberLinkPtr link;
    {
        std::lock_guard<std::mutex> lock{subscriber_links_mutex_};

        if (dropped_) {
            return;
        }

        if (sub_link->isIntraprocess()) {
            --intraprocess_subscriber_count_;
        }

        const auto it = std::find(subscriber_links_.cbegin(), subscriber_links_.cend(), sub_link);
        if (it != subscriber_links_.cend()) {
            link = *it;
            subscriber_links_.erase(it);
        }
    }

    if (link) {
        peerDisconnect(link);
    }
}

XmlRpc::XmlRpcValue Publication::getStats() {
    XmlRpc::XmlRpcValue stats;
    stats[0] = name_;
    XmlRpc::XmlRpcValue conn_data;
    conn_data.setSize(0); // force to be an array, even if it's empty

    std::lock_guard<std::mutex> lock{subscriber_links_mutex_};

    uint32_t cidx = 0;
    for (const auto& c: subscriber_links_) {
        const SubscriberLink::Stats& s = c->getStats();
        conn_data[cidx][0] = c->getConnectionID();
        // todo: figure out what to do here... the bytes_sent will wrap around
        // on some flows within a reasonable amount of time. xmlrpc++ doesn't
        // seem to give me a nice way to do 64-bit ints, perhaps that's a
        // limitation of xml-rpc, not sure. alternatively we could send the number
        // of KB transmitted to gain a few order of magnitude.
        conn_data[cidx][1] = (int)s.bytes_sent_;
        conn_data[cidx][2] = (int)s.message_data_sent_;
        conn_data[cidx][3] = (int)s.messages_sent_;
        conn_data[cidx][4] = 0; // not sure what is meant by connected
        ++cidx;
    }

    stats[1] = conn_data;
    return stats;
}

// Publisher : [(connection_id, destination_caller_id, direction, transport, topic_name, connected, connection_info_string)*]
// e.g. [(2, '/listener', 'o', 'TCPROS', '/chatter', 1, 'TCPROS connection on port 55878 to [127.0.0.1:44273 on socket 7]')]
void Publication::getInfo(XmlRpc::XmlRpcValue& info) {
    std::lock_guard<std::mutex> lock{subscriber_links_mutex_};

    for (const auto& c: subscriber_links_) {
        XmlRpc::XmlRpcValue curr_info;
        curr_info[0] = (int)c->getConnectionID();
        curr_info[1] = c->getDestinationCallerID();
        curr_info[2] = "o";
        curr_info[3] = c->getTransportType();
        curr_info[4] = name_;
        curr_info[5] = true; // For length compatibility with rospy
        curr_info[6] = c->getTransportInfo();
        info[info.size()] = curr_info;
    }
}

void Publication::dropAllConnections() {
    // Swap our publishers list with a local one so we can only lock for a short period of time, because a
    // side effect of our calling drop() on connections can be re-locking the publishers mutex
    V_SubscriberLink local_publishers;

    {
        std::lock_guard<std::mutex> lock{subscriber_links_mutex_};
        local_publishers.swap(subscriber_links_);
    }

    for (const auto& p: local_publishers) {
        p->drop();
    }
}

void Publication::peerConnect(const SubscriberLinkPtr& sub_link) {
    for (const auto& cbs: callbacks_) {
        if (cbs->connect_ && cbs->callback_queue_) {
            CallbackInterfacePtr cb{boost::make_shared<PeerConnDisconnCallback>(cbs->connect_, sub_link, cbs->has_tracked_object_, cbs->tracked_object_)};
            cbs->callback_queue_->addCallback(cb, (uint64_t)cbs.get());
        }
    }
}

void Publication::peerDisconnect(const SubscriberLinkPtr& sub_link) {
    for (const auto& cbs: callbacks_) {
        if (cbs->disconnect_ && cbs->callback_queue_) {
            CallbackInterfacePtr cb{boost::make_shared<PeerConnDisconnCallback>(cbs->disconnect_, sub_link, cbs->has_tracked_object_, cbs->tracked_object_)};
            cbs->callback_queue_->addCallback(cb, (uint64_t)cbs.get());
        }
    }
}

size_t Publication::getNumCallbacks() {
    std::lock_guard<std::mutex> lock{callbacks_mutex_};
    return callbacks_.size();
}

uint32_t Publication::incrementSequence() {
    std::lock_guard<std::mutex> lock{seq_mutex_};
    const auto old_seq = seq_;
    ++seq_;
    return old_seq;
}

uint32_t Publication::getNumSubscribers() {
    std::lock_guard<std::mutex> lock{subscriber_links_mutex_};
    return (uint32_t)subscriber_links_.size();
}

void Publication::getPublishTypes(bool& serialize, bool& nocopy, const std::type_info& ti) {
    std::lock_guard<std::mutex> lock{subscriber_links_mutex_};
    for (const auto& sub: subscriber_links_) {
        auto s = false;
        auto n = false;
        sub->getPublishTypes(s, n, ti);
        serialize = serialize || s;
        nocopy = nocopy || n;

        if (serialize && nocopy) {
            break;
        }
    }
}

bool Publication::hasSubscribers() {
    std::lock_guard<std::mutex> lock{subscriber_links_mutex_};
    return !subscriber_links_.empty();
}

void Publication::publish(ros::SerializedMessage& m) {
    if (m.message) {
        std::lock_guard<std::mutex> lock{subscriber_links_mutex_};
        for (const auto& sub: subscriber_links_) {
            if (sub->isIntraprocess()) {
                sub->enqueueMessage(m, false, true);
            }
        }
        m.message.reset();
    }

    if (m.buf) {
        std::lock_guard<std::mutex> lock{publish_queue_mutex_};
        publish_queue_.push_back(m);
    }
}

void Publication::processPublishQueue() {
    V_SerializedMessage queue;
    {
        std::lock_guard<std::mutex> lock{publish_queue_mutex_};

        if (dropped_) {
            return;
        }

        queue.insert(queue.end(), publish_queue_.begin(), publish_queue_.end());
        publish_queue_.clear();
    }

    for (const auto& m: queue) {
        enqueueMessage(m);
    }
}

bool Publication::validateHeader(const ros::Header& header, std::string& error_msg) {
    std::string md5sum, topic, client_callerid;
    if (!header.getValue("md5sum", md5sum) || !header.getValue("topic", topic) || !header.getValue("callerid", client_callerid)) {
        std::string msg("Header from subscriber did not have the required elements: md5sum, topic, callerid");
        ROS_ERROR("%s", msg.c_str());
        error_msg = msg;
        return false;
    }

    // Check whether the topic has been deleted from
    // advertised_topics through a call to unadvertise(), which could
    // have happened while we were waiting for the subscriber to
    // provide the md5sum.
    if (isDropped()) {
        std::string msg{"received a tcpros connection for a nonexistent topic [" + topic + "] from [" + client_callerid + "]."};
        ROS_ERROR("%s", msg.c_str());
        error_msg = msg;
        return false;
    }

    if (getMD5Sum() != md5sum && (md5sum != std::string{"*"} && getMD5Sum() != std::string{"*"})) {
        std::string datatype;
        header.getValue("type", datatype);

        std::string msg{"Client [" + client_callerid + "] wants topic " + topic + " to have datatype/md5sum [" + datatype + "/" + md5sum + "], but our version has [" + getDataType() + "/" + getMD5Sum() + "]. Dropping connection."};
        ROS_ERROR("%s", msg.c_str());
        error_msg = msg;
        return false;
    }
    return true;
}

} // namespace roscan
