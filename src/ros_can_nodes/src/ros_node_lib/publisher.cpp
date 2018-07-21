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
#include "publisher.h"
#include "publication.h"
#include "topic_manager.h"

namespace roscan {

void Publisher::unadvertise() {
    if (!unadvertised_) {
        unadvertised_ = true;
        node_->topic_manager()->unadvertise(topic_, callbacks_);
        node_.reset();
    }
}

void Publisher::publish(const boost::function<ros::SerializedMessage(void)>& serfunc, ros::SerializedMessage& m) const {
    if (unadvertised_) {
        return;
    }
    node_->topic_manager()->publish(topic_, serfunc, m);
}

void Publisher::incrementSequence() const {
    if (!unadvertised_) {
        node_->topic_manager()->incrementSequence(topic_);
    }
}

void Publisher::shutdown() {
    if (!unadvertised_) {
        unadvertise();
    }
}

std::string Publisher::getTopic() const {
    if (!unadvertised_) {
        return topic_;
    }
    return std::string{};
}

uint32_t Publisher::getNumSubscribers() const {
    if (!unadvertised_) {
        return node_->topic_manager()->getNumSubscribers(topic_);
    }
    return 0;
}

bool Publisher::isLatched() const {
    if (unadvertised_) {
        throw ros::Exception("Call to isLatched() on an invalid Publisher");
    }
    const auto publication_ptr = node_->topic_manager()->lookupPublication(topic_);
    if (!publication_ptr) {
        throw ros::Exception("Call to isLatched() on an invalid Publisher");
    }
    return publication_ptr->isLatched();
}

} // namespace roscan
