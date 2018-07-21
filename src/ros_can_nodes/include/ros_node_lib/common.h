#ifndef ROSCAN_COMMON_H
#define ROSCAN_COMMON_H

#include <vector>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace roscan {

class RosCanNode;
typedef boost::shared_ptr<RosCanNode> RosCanNodePtr;

class Publisher;
typedef boost::shared_ptr<Publisher> PublisherPtr;

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

class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef boost::weak_ptr<Subscription> SubscriptionWPtr;

class SubscriberLink;
typedef boost::shared_ptr<SubscriberLink> SubscriberLinkPtr;
typedef std::vector<SubscriberLinkPtr> V_SubscriberLink;

class TransportSubscriberLink;
typedef boost::shared_ptr<TransportSubscriberLink> TransportSubscriberLinkPtr;

class IntraProcessSubscriberLink;
typedef boost::shared_ptr<IntraProcessSubscriberLink> IntraProcessSubscriberLinkPtr;

class SingleSubscriberPublisher;
typedef boost::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

class CallbackQueue;
typedef boost::shared_ptr<CallbackQueue> CallbackQueuePtr;

struct SubscribeOptions;
struct AdvertiseOptions;
class ROSOutAppender;

class SubscriptionQueue;
typedef boost::shared_ptr<SubscriptionQueue> SubscriptionQueuePtr;

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