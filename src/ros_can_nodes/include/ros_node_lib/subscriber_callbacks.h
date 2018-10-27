#ifndef ROSCAN_SUBSCRIBER_CALLBACKS_H
#define ROSCAN_SUBSCRIBER_CALLBACKS_H

#include "single_subscriber_publisher.h"
#include "callback_queue_interface.h"
#include "rosdefs.h"
#include <boost/shared_ptr.hpp>

namespace roscan {

struct SubscriberCallbacks {
    SubscriberCallbacks(const SubscriberStatusCallback& connect = SubscriberStatusCallback{},
                        const SubscriberStatusCallback& disconnect = SubscriberStatusCallback{},
                        const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr{},
                        CallbackQueueInterface *callback_queue = nullptr)
        : connect_{connect}, disconnect_{disconnect}, callback_queue_{callback_queue} {
        has_tracked_object_ = false;
        if (tracked_object) {
            has_tracked_object_ = true;
            tracked_object_ = tracked_object;
        }
    }
    SubscriberStatusCallback connect_;
    SubscriberStatusCallback disconnect_;

    bool has_tracked_object_;
    ros::VoidConstWPtr tracked_object_;
    CallbackQueueInterface *callback_queue_;
};
typedef boost::shared_ptr<SubscriberCallbacks> SubscriberCallbacksPtr;

} // namespace roscan

#endif // ROSCAN_SUBSCRIBER_CALLBACKS_H