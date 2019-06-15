#!/usr/bin/env python

import os.path
import re
import itertools
from six import StringIO
from rospkg import RosPack
import rosmsg
import genmsg
from genmsg import gentools
import gencpp


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

#include <unordered_map>
#include <string>
#include "message_properties_map.hpp"

const std::unordered_map<std::string, const message_properties> message_properties_map = {{
{}
}};
"""


def generate_hpp():
    hpp = StringIO()
    hpp.write(hpp_contents)
    with open('include/message_properties_map.hpp', 'w') as f:
        f.write(hpp.getvalue())


def generate_cpp(msg_properties):
    map_entry_template = '{{ "{}",{{ "{}", "{}" }} }}'
    map_entries = ',\n'.join([map_entry_template.format(t, d, m) for t, d, m in msg_properties])
    cpp = StringIO()
    cpp.write(cpp_contents.format(map_entries))
    with open('src/message_properties_map.cpp', 'w') as f:
        f.write(cpp.getvalue())


def generate_message_properties_map():
    rp = RosPack()
    msg_context = genmsg.MsgContext.create_default()

    # find all possible messages
    packages = rp.list()
    msgs = itertools.chain.from_iterable([rosmsg.list_msgs(p, rp) for p in packages])

    # build search paths
    search_path = {}
    for p in packages:
        package_paths = rosmsg._get_package_paths(p, rp)
        search_path[p] = [os.path.join(d, 'msg') for d in package_paths]

    # save some space by stripping comments and blank lines
    comments = re.compile(r'^\s*(.*)#.*$', re.MULTILINE)
    blank_lines = re.compile(r'\n(\s*\n)*')
    msg_properties = []
    for m in msgs:
        spec = genmsg.load_msg_by_type(msg_context, m, search_path)
        genmsg.load_depends(msg_context, spec, search_path)
        text = gentools.compute_full_text(msg_context, spec)
        text = comments.sub(r'\1', text)
        text = blank_lines.sub('\n', text)
        text = '\n'.join([l.strip() for l in text.splitlines() if l != ''])
        text = gencpp.escape_message_definition(text)
        md5sum = gentools.compute_md5(msg_context, spec)
        msg_properties.append((m, text, md5sum))
    
    generate_hpp()
    generate_cpp(msg_properties)


if __name__ == '__main__':
    generate_message_properties_map()
