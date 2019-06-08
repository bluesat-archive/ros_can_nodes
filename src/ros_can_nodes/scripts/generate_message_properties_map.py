#!/usr/bin/env python

import sys
import os
import itertools
import rosmsg

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


def makedirs(directory):
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise


def generate_hpp():
    hpp = StringIO()
    hpp.write(hpp_contents)
    output_dir = os.getcwd() + '/include/'
    makedirs(output_dir)
    with open(output_dir + 'message_properties_map.hpp', 'w') as f:
        f.write(hpp.getvalue())


def generate_cpp(messages_library):
    messages_includes = "\n".join(['#include <{}/{}.h>'.format(p, m) for p, m in messages_library])
    map_entries = "\n".join([map_entry_template.format('{}::{}'.format(p, m)) for p, m in messages_library])
    
    cpp = StringIO()
    cpp.write(cpp_contents.format(messages_includes, map_entries))
    output_dir = os.getcwd() + '/src/'
    makedirs(output_dir)
    with open(output_dir + '/message_properties_map.cpp', 'w') as f:
        f.write(cpp.getvalue())


def generate_message_properties_map(packages):
    # build a list of (package, message) tuples
    msgs = list(itertools.chain.from_iterable([rosmsg.list_msgs(p) for p in packages]))
    messages_library = [tuple(msg.split('/')) for msg in msgs]
    
    generate_hpp()
    generate_cpp(messages_library)


if __name__ == '__main__':
    generate_message_properties_map(sys.argv[1:])
