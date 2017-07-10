#include "ros/xmlrpc_manager.h"
#include <iostream>

using namespace std;

namespace ros {

    void myTest() {
        XMLRPCManagerPtr srvr1(new XMLRPCManager);
        XMLRPCManagerPtr srvr2(new XMLRPCManager);
        XMLRPCManagerPtr srvr3(new XMLRPCManager);
        srvr1->start();
        srvr2->start();
        srvr3->start();
        cout << srvr1->getServerURI() << endl;
        cout << srvr2->getServerURI() << endl;
        cout << srvr3->getServerURI() << endl;
        cout << srvr1->getServerURI() << endl;
        srvr1->shutdown();
        cout << srvr2->getServerURI() << endl;
        srvr2->shutdown();
        cout << srvr3->getServerURI() << endl;
        srvr3->shutdown();
    }

}

int main() {
    ros::myTest();
    return 0;
}

