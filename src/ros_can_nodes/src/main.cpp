#include "xmlrpc_manager.h"

namespace ros {

    void myTest() {
        XMLRPCManagerPtr p = XMLRPCManager::instance();
    }

}

int main() {
    ros::myTest();
    return 0;
}

