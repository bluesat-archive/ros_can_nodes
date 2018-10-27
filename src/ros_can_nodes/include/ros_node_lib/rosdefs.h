#ifndef ROSCAN_ROSDEFS_H
#define ROSCAN_ROSDEFS_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace ros {

typedef boost::shared_ptr<void const> VoidConstPtr;
typedef boost::weak_ptr<void const> VoidConstWPtr;

class TransportTCP;
typedef boost::shared_ptr<TransportTCP> TransportTCPPtr;
class TransportUDP;
typedef boost::shared_ptr<TransportUDP> TransportUDPPtr;
class Connection;
typedef boost::shared_ptr<Connection> ConnectionPtr;

// class ServicePublication;
// typedef boost::shared_ptr<ServicePublication> ServicePublicationPtr;
// typedef std::list<ServicePublicationPtr> L_ServicePublication;
// typedef std::vector<ServicePublicationPtr> V_ServicePublication;

// class ServiceServerLink;
// typedef boost::shared_ptr<ServiceServerLink> ServiceServerLinkPtr;
// typedef std::list<ServiceServerLinkPtr> L_ServiceServerLink;

struct SteadyTimerEvent;

void disableAllSignalsInThisThread();

} // namespace ros

#endif // ROSCAN_ROSDEFS_H