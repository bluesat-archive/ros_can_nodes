#ifndef ROSCAN_COMMON_H
#define ROSCAN_COMMON_H

#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <queue>
#include <deque>
#include <map>
#include <set>
#include <algorithm>
#include <typeinfo>
#include <cstdint>
#include <cstring>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <boost/signals2/connection.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/bind.hpp>
#include <ros/time.h>

namespace ros {

typedef boost::shared_ptr<void> VoidPtr;
typedef boost::weak_ptr<void> VoidWPtr;
typedef boost::shared_ptr<void const> VoidConstPtr;
typedef boost::weak_ptr<void const> VoidConstWPtr;
typedef boost::signals2::signal<void(void)> VoidSignal;
typedef boost::function<void(void)> VoidFunc;

class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;
class TransportTCP;
typedef boost::shared_ptr<TransportTCP> TransportTCPPtr;
class TransportUDP;
typedef boost::shared_ptr<TransportUDP> TransportUDPPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;
typedef std::set<ConnectionPtr> S_Connection;
typedef std::vector<ConnectionPtr> V_Connection;

/*
class ServicePublication;
typedef boost::shared_ptr<ServicePublication> ServicePublicationPtr;
typedef std::list<ServicePublicationPtr> L_ServicePublication;
typedef std::vector<ServicePublicationPtr> V_ServicePublication;

class ServiceServerLink;
typedef boost::shared_ptr<ServiceServerLink> ServiceServerLinkPtr;
typedef std::list<ServiceServerLinkPtr> L_ServiceServerLink;

class Transport;
typedef boost::shared_ptr<Transport> TransportPtr;

class NodeHandle;
typedef boost::shared_ptr<NodeHandle> NodeHandlePtr;
*/

// // Structure passed as a parameter to the callback invoked by a ros::SteadyTimer
// struct SteadyTimerEvent {
//     SteadyTime last_expected; ///< In a perfect world, this is when the last callback should have happened
//     SteadyTime last_real;     ///< When the last callback actually happened

//     SteadyTime current_expected; ///< In a perfect world, this is when the current callback should be happening
//     SteadyTime current_real;     ///< This is when the current callback was actually called (SteadyTime::now() as of the beginning of the callback)

//     struct {
//         WallDuration last_duration; ///< How long the last callback ran for
//     } profile;
// };
struct SteadyTimerEvent;
typedef boost::function<void(const SteadyTimerEvent&)> SteadyTimerCallback;

} // namespace ros

namespace roscan {

typedef std::vector<std::string> V_string;
typedef std::map<std::string, std::string> M_string;

class RosCanNode;
typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;

class Publisher;
typedef boost::shared_ptr<Publisher> PublisherPtr;
typedef std::vector<Publisher> V_Publisher;

class Publication;
typedef boost::shared_ptr<Publication> PublicationPtr;
typedef boost::weak_ptr<Publication> PublicationWPtr;
typedef std::vector<PublicationPtr> V_Publication;

class PublisherLink;
typedef boost::shared_ptr<PublisherLink> PublisherLinkPtr;
typedef std::vector<PublisherLinkPtr> V_PublisherLink;

class TransportPublisherLink;
typedef boost::shared_ptr<TransportPublisherLink> TransportPublisherLinkPtr;

class IntraProcessPublisherLink;
typedef boost::shared_ptr<IntraProcessPublisherLink> IntraProcessPublisherLinkPtr;

class Subscriber;
typedef boost::shared_ptr<Subscriber> SubscriberPtr;
typedef std::vector<Subscriber> V_Subscriber;

class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef boost::weak_ptr<Subscription> SubscriptionWPtr;
typedef std::list<SubscriptionPtr> L_Subscription;
typedef std::vector<SubscriptionPtr> V_Subscription;
typedef std::set<SubscriptionPtr> S_Subscription;

class SubscriberLink;
typedef boost::shared_ptr<SubscriberLink> SubscriberLinkPtr;
typedef std::vector<SubscriberLinkPtr> V_SubscriberLink;

class TransportSubscriberLink;
typedef boost::shared_ptr<TransportSubscriberLink> TransportSubscriberLinkPtr;

class IntraProcessSubscriberLink;
typedef boost::shared_ptr<IntraProcessSubscriberLink> IntraProcessSubscriberLinkPtr;

class SingleSubscriberPublisher;
typedef boost::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

class CallbackQueueInterface;
class CallbackInterface;
typedef boost::shared_ptr<CallbackInterface> CallbackInterfacePtr;

class CallbackQueue;
typedef boost::shared_ptr<CallbackQueue> CallbackQueuePtr;

struct SubscribeOptions;
struct AdvertiseOptions;
class ROSOutAppender;

class SubscriptionQueue;
typedef boost::shared_ptr<SubscriptionQueue> SubscriptionQueuePtr;

struct SubscriberCallbacks {
    SubscriberCallbacks(const SubscriberStatusCallback& connect = SubscriberStatusCallback(),
                        const SubscriberStatusCallback& disconnect = SubscriberStatusCallback(),
                        const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
                        CallbackQueueInterface* callback_queue = 0)
        : connect_(connect), disconnect_(disconnect), callback_queue_(callback_queue) {
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
    CallbackQueueInterface* callback_queue_;
};
typedef boost::shared_ptr<SubscriberCallbacks> SubscriberCallbacksPtr;

/*
// Structure passed as a parameter to the callback invoked by a ros::Timer
struct TimerEvent {
    Time last_expected; ///< In a perfect world, this is when the last callback should have happened
    Time last_real;     ///< When the last callback actually happened

    Time current_expected; ///< In a perfect world, this is when the current callback should be happening
    Time current_real;     ///< This is when the current callback was actually called (Time::now() as of the beginning of the callback)

    struct {
        WallDuration last_duration; ///< How long the last callback ran for
    } profile;
};
typedef boost::function<void(const TimerEvent&)> TimerCallback;

// Structure passed as a parameter to the callback invoked by a ros::WallTimer
struct WallTimerEvent {
    WallTime last_expected; ///< In a perfect world, this is when the last callback should have happened
    WallTime last_real;     ///< When the last callback actually happened

    WallTime current_expected; ///< In a perfect world, this is when the current callback should be happening
    WallTime current_real;     ///< This is when the current callback was actually called (Time::now() as of the beginning of the callback)

    struct {
        WallDuration last_duration; ///< How long the last callback ran for
    } profile;
};
typedef boost::function<void(const WallTimerEvent&)> WallTimerCallback;

class ServiceManager;
typedef boost::shared_ptr<ServiceManager> ServiceManagerPtr;
*/

class TopicManager;
typedef boost::shared_ptr<TopicManager> TopicManagerPtr;
class ConnectionManager;
typedef boost::shared_ptr<ConnectionManager> ConnectionManagerPtr;
class XMLRPCManager;
typedef boost::shared_ptr<XMLRPCManager> XMLRPCManagerPtr;
class PollManager;
typedef boost::shared_ptr<PollManager> PollManagerPtr;

} // namespace roscan

#endif // ROSCAN_COMMON_H