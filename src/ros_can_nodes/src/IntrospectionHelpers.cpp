#include "IntrospectionHelpers.hpp"
#include <ros_type_introspection/ros_introspection.hpp>
#include <ros_type_introspection/utils/shape_shifter.hpp>
#include <deque>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <cstdint>
#include <ros/ros.h>

namespace IntrospectionHelpers {
    using RosIntrospection::Parser;
    using RosIntrospection::ShapeShifter;
    using RosIntrospection::ROSField;
    using RosIntrospection::ROSType;
    using RosIntrospection::BuiltinType;

    // RosIntrospection parser
    static Parser parser;

    // map of message type to vector of fields in that type
    static std::unordered_map<std::string, std::vector<ROSField>> msg_fields;

    // Mutex for the message registration process
    static std::mutex registration_mutex;

    /**
     * Recursively modifies the buffer using msg_fields as a message type template
     */
    void modify_buffer_recursive(const ROSField& field, std::deque<uint8_t>& raw, std::vector<uint8_t>& modified) {
        // transfer bytes from raw to modified
        static constexpr auto transfer_bytes = [](auto& raw, auto& modified, const auto size) {
            // copy size bytes from start of raw to end of modified
            modified.insert(modified.cend(), raw.cbegin(), raw.cbegin() + size);

            // delete size bytes from start of raw
            raw.erase(raw.cbegin(), raw.cbegin() + size);
        };

        // modifies the size specifier of array-like fields
        // also returns the size value
        static constexpr auto modify_size = [](auto& raw, auto& modified) {
            // get the size value
            const auto size = raw.front();

            // transfer size byte over
            transfer_bytes(raw, modified, 1);

            // delete 3 bytes from start of raw
            raw.erase(raw.cbegin(), raw.cbegin() + 3);
            return size;
        };

        const auto& type = field.type();

        if (field.isArray()) {
            auto len = field.arraySize();
            if (len == -1) { // varied length array
                // varied length array types stored as 'size' then contents
                // modify size
                len = modify_size(raw, modified);
            }

            // iterate through size, modify contents recursively
            const auto non_array_field = ROSField{type.baseName() + " x"};
            for (auto i = 0;i < len;++i) {
                modify_buffer_recursive(non_array_field, raw, modified);
            }
        } else if (type.typeID() == BuiltinType::STRING) {
            // string is stored as 'size' then string contents (no null terminator)
            // modify size
            const auto len = modify_size(raw, modified);

            // transfer string contents
            transfer_bytes(raw, modified, len);
        } else if (type.typeID() == BuiltinType::OTHER) {
            // non-primitive internal message type
            // iterate through type's fields and modify contents recursively
            for (const auto& f : msg_fields[type.baseName()]) {
                modify_buffer_recursive(f, raw, modified);
            }
        } else {
            // non-string primitives - directly transfer bytes
            transfer_bytes(raw, modified, type.typeSize());
        }
    }

    void registerMessage(const ShapeShifter::ConstPtr& msg, const std::string& topic_name) {
        // get message type and definition
        const std::string& datatype = msg->getDataType();
        const std::string& definition = msg->getMessageDefinition();

        {
            std::lock_guard<std::mutex> registration_lock{registration_mutex};

            // register with RosIntrospection parser to create type list
            parser.registerMessageDefinition(topic_name, ROSType{datatype}, definition);

            // store fields structure for each message type found
            for (const auto& t : parser.getMessageInfo(topic_name)->type_list) {
                auto& fields = msg_fields[t.type().baseName()];

                if (fields.size() == 0) { // process fields only if not processed before
                    ROS_INFO_STREAM("NEW: " << t.type());
                    for (const auto& f : t.fields()) {
                        if (!f.isConstant()) { // constants are not part of message buffers
                            fields.push_back(f);
                        }
                    }
                }
            }
        }
    }

    void printRegistered() {
        for (const auto& msg : msg_fields) {
            ROS_INFO_STREAM("MESSAGETYPE: " << msg.first);
            for (const auto& f : msg.second) {
                const auto array_size = f.arraySize();
                const auto array_str = "[" + ((array_size != -1) ? std::to_string(array_size) : "")  + "]";
                ROS_INFO_STREAM(f.name() << " (" << f.type() << (f.isArray() ? array_str : "") << ")");
            }
        }
    }

    std::vector<uint8_t> modify_buffer(const std::string& datatype, const uint8_t *const data, const uint32_t size) {
        std::deque<uint8_t> raw{data, data + size};
        std::vector<uint8_t> modified;
        modified.reserve(size);
        modify_buffer_recursive(ROSField{datatype + " x"}, raw, modified);
        return modified;
    }

} // namespace IntrospectionHelpers