#!/usr/bin/env python

import sys
import os.path
import glob
import itertools

import roslib.msgs
import roslib.packages
from rospkg import RosPack

try:
    from cStringIO import StringIO #Python 2.x
except ImportError:
    from io import StringIO #Python 3.x


hpp_contents =\
"""
/* Auto-generated for ros_can_nodes */
/* DO NOT EDIT */
#ifndef MESSAGE_PROPERTIES_MAP_HPP
#define MESSAGE_PROPERTIES_MAP_HPP

#include <unordered_map>
#include <string>

struct message_properties {
    const std::string definition;
    const std::string md5sum;
};

extern const std::unordered_map<std::string, const message_properties> message_properties_map;

#endif // MESSAGE_PROPERTIES_MAP_HPP
"""

cpp_contents =\
"""
/* Auto-generated for ros_can_nodes */
/* DO NOT EDIT */

#include <ros/message_traits.h>
#include <unordered_map>
#include <string>
#include "message_properties_map.hpp"

{0}

const std::unordered_map<std::string, const message_properties> message_properties_map = {{
{1}
}};
"""

map_entry_template =\
'{{ std::string{{ros::message_traits::DataType<{0}>::value()}},' +\
'{{ std::string{{ros::message_traits::Definition<{0}>::value()}},' +\
'std::string{{ros::message_traits::MD5Sum<{0}>::value()}} }} }},'


def generate_hpp():
    hpp = StringIO()
    hpp.write(hpp_contents)
    with open('include/message_properties_map.hpp', 'w') as f:
        f.write(hpp.getvalue())


def generate_cpp(messages_library):
    messages_includes = "\n".join(['#include <{}/{}.h>'.format(p, m) for p, m in messages_library])
    map_entries = "\n".join([map_entry_template.format('{}::{}'.format(p, m)) for p, m in messages_library])
    
    cpp = StringIO()
    cpp.write(cpp_contents.format(messages_includes, map_entries))
    with open('src/message_properties_map.cpp', 'w') as f:
        f.write(cpp.getvalue())


def get_messages(package):
    package_dir = RosPack().get_path(package)

    # get all message names and strip .msg
    msg_glob = package_dir + "/msg/*.msg"
    msgs = glob.glob(msg_glob)
    msgs_names = [os.path.splitext(os.path.split(m)[1])[0] for m in msgs]

    # get all package .h file names and strip .h
    headers_glob = "{}/../..{}/include/{}/*.h".format(package_dir, "" if "/opt/ros" in package_dir else "/devel", package)
    headers = glob.glob(headers_glob)
    headers_names = [os.path.splitext(os.path.split(h)[1])[0] for h in headers]

    # only care about msgs with their corresponding headers available
    return set(msgs_names).intersection(headers_names)


def generate_message_properties_map(packages):
    # build a list of (package, message) tuples
    messages_library = list(itertools.chain.from_iterable([[(p, m) for m in get_messages(p)] for p in packages]))
    
    generate_hpp()
    generate_cpp(messages_library)


if __name__ == '__main__':
    generate_message_properties_map(sys.argv[1:])
