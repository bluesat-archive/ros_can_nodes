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

#ifndef ROSCAN_INTRAPROCESS_SUBSCRIBER_LINK_H
#define ROSCAN_INTRAPROCESS_SUBSCRIBER_LINK_H

#include "common.h"
#include "subscriber_link.h"
#include <typeinfo>
#include <mutex>

namespace roscan {

// SubscriberLink handles broadcasting messages to a single subscriber on a single topic
class IntraProcessSubscriberLink : public SubscriberLink {
    public:
        IntraProcessSubscriberLink(const RosNodePtr& node, const PublicationPtr& parent);
        ~IntraProcessSubscriberLink() override {}

        void setSubscriber(const IntraProcessPublisherLinkPtr& subscriber);
        bool isLatching() const;

        void enqueueMessage(const ros::SerializedMessage& m, const bool ser, const bool nocopy) override;
        void drop() override;
        std::string getTransportType() override { return std::string("INTRAPROCESS"); }
        std::string getTransportInfo() override { return getTransportType(); }
        bool isIntraprocess() const override { return true; }
        void getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti) override;

    private:
        RosNodePtr node_;
        IntraProcessPublisherLinkPtr subscriber_;
        bool dropped_;
        std::recursive_mutex drop_mutex_;
};

} // namespace roscan

#endif // ROSCAN_INTRAPROCESS_SUBSCRIBER_LINK_H
