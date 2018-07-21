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

#include "RosCanNode.hpp"
#include "intraprocess_subscriber_link.h"
#include "intraprocess_publisher_link.h"
#include "publication.h"
#include <ros/connection.h>
#include <ros/header.h>
#include <ros/transport/transport.h>
#include <mutex>

namespace roscan {

IntraProcessSubscriberLink::IntraProcessSubscriberLink(const RosCanNodePtr& node, const PublicationPtr& parent) : SubscriberLink{node}, dropped_{false} {
    parent_ = parent;
    topic_ = parent->getName();
}

void IntraProcessSubscriberLink::setSubscriber(const IntraProcessPublisherLinkPtr& subscriber) {
    subscriber_ = subscriber;
    connection_id_ = node_->connection_manager()->getNewConnectionID();
    destination_caller_id_ = node_->getName();
}

bool IntraProcessSubscriberLink::isLatching() const {
    if (const auto parent = parent_.lock()) {
        return parent->isLatching();
    }
    return false;
}

void IntraProcessSubscriberLink::enqueueMessage(const ros::SerializedMessage& m, const bool ser, const bool nocopy) {
    std::lock_guard<std::recursive_mutex> lock{drop_mutex_};
    if (dropped_) {
        return;
    }
    subscriber_->handleMessage(m, ser, nocopy);
}

void IntraProcessSubscriberLink::drop() {
    {
        std::lock_guard<std::recursive_mutex> lock{drop_mutex_};
        if (dropped_) {
            return;
        }
        dropped_ = true;
    }

    if (subscriber_) {
        subscriber_->drop();
        subscriber_.reset();
    }

    if (const auto parent = parent_.lock()) {
        parent->removeSubscriberLink(shared_from_this());
    }
}

void IntraProcessSubscriberLink::getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti) {
    std::lock_guard<std::recursive_mutex> lock{drop_mutex_};
    if (dropped_) {
        return;
    }
    subscriber_->getPublishTypes(ser, nocopy, ti);
}

} // namespace roscan
