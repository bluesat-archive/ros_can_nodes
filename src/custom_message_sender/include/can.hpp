#include <string>
#include <linux/can.h>

int open_can_port(const std::string& port);
int send_can_frame(const can_frame& frame);