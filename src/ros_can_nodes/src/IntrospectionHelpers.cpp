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
     * Recursively transforms the buffer using msg_fields as a message type template
     */
    std::vector<uint8_t> to_can_buf_recursive(const ROSField& field, std::vector<uint8_t>& buf) {
        // transfer bytes from src to dst
        static constexpr auto transfer_bytes = [](auto& src, auto& dst, const auto len) {
            dst.insert(dst.cend(), src.cbegin(), src.cbegin() + len);
            src.erase(src.cbegin(), src.cbegin() + len);
        };

        // extracts the length specifier of array-like fields, deleting it from buf
        static constexpr auto extract_array_len = [](auto& buf) {
            const auto len = *reinterpret_cast<uint32_t *>(buf.data());
            buf.erase(buf.cbegin(), buf.cbegin() + sizeof(uint32_t));
            return len;
        };

        // inserts the buffer's length in bytes as a 16bit integer to its start
        static constexpr auto insert_buf_length = [](auto& buf) {
            const auto len = static_cast<uint16_t>(buf.size());
            const auto ptr = reinterpret_cast<const uint8_t *const>(&len);
            buf.insert(buf.cbegin(), ptr, ptr + sizeof(uint16_t));
        };

        std::vector<uint8_t> tmp_buf;
        tmp_buf.reserve(buf.size() + sizeof(uint32_t));
        
        const auto& type = field.type();

        if (field.isArray()) {
            auto len = field.arraySize();
            if (len == -1) { // varied length array
                // varied length array types stored as 'array length' then contents
                len = extract_array_len(buf);
            }

            // iterate through size, recursively transform buffer
            const auto non_array_field = ROSField{type.baseName() + " x"};
            for (auto i = 0;i < len;++i) {
                const auto tmp_buf_i = to_can_buf_recursive(non_array_field, buf);
                tmp_buf.insert(tmp_buf.cend(), tmp_buf_i.cbegin(), tmp_buf_i.cend());
            }
            insert_buf_length(tmp_buf);
        } else if (type.typeID() == BuiltinType::STRING) {
            // string is stored as 'length' then string contents (no null terminator)
            const auto len = extract_array_len(buf);
            transfer_bytes(buf, tmp_buf, len);

            // add null terminator
            tmp_buf.push_back(0);
            insert_buf_length(tmp_buf);
        } else if (type.typeID() == BuiltinType::OTHER) {
            // non-primitive internal message type
            // iterate through type's fields and modify contents recursively
            for (const auto& field : msg_fields[type.baseName()]) {
                const auto tmp_buf_i = to_can_buf_recursive(field, buf);
                tmp_buf.insert(tmp_buf.cend(), tmp_buf_i.cbegin(), tmp_buf_i.cend());
            }
        } else {
            // non-string primitives - directly transfer bytes
            transfer_bytes(buf, tmp_buf, type.typeSize());
        }
        return tmp_buf;
    }

    /**
     * Recursively shrinks the buffer using msg_fields as a message type template
     * shrinks all length specifiers from 4 bytes to 1
     */
    void shrink_buf_recursive(const ROSField& field, std::deque<uint8_t>& raw, std::vector<uint8_t>& modified) {
        // transfer bytes from src to dst
        static constexpr auto transfer_bytes = [](auto& src, auto& dst, const auto size) {
            dst.insert(dst.cend(), src.cbegin(), src.cbegin() + size);
            src.erase(src.cbegin(), src.cbegin() + size);
        };

        // shrinks the length specifier of array-like fields
        // also returns the length value
        static constexpr auto shrink_array_len = [](auto& raw, auto& modified) {
            // get the length value
            const auto len = raw.front();

            // transfer length byte over
            transfer_bytes(raw, modified, 1);

            // delete 3 bytes from start of raw
            raw.erase(raw.cbegin(), raw.cbegin() + 3);
            return len;
        };

        const auto& type = field.type();

        if (field.isArray()) {
            auto len = field.arraySize();
            if (len == -1) { // varied length array
                // varied length array types stored as 'array length' then contents
                len = shrink_array_len(raw, modified);
            }

            // iterate through len, shrink contents recursively
            const auto non_array_field = ROSField{type.baseName() + " x"};
            for (auto i = 0;i < len;++i) {
                shrink_buf_recursive(non_array_field, raw, modified);
            }
        } else if (type.typeID() == BuiltinType::STRING) {
            // string is stored as 'length' then string contents (no null terminator)
            const auto len = shrink_array_len(raw, modified);

            // transfer string contents
            transfer_bytes(raw, modified, len);
        } else if (type.typeID() == BuiltinType::OTHER) {
            // non-primitive internal message type
            // iterate through type's fields and modify contents recursively
            for (const auto& f : msg_fields[type.baseName()]) {
                shrink_buf_recursive(f, raw, modified);
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

    void print_buf(const std::vector<uint8_t>& buf) {
        ROS_INFO("BUF");
        for (const auto c : buf) {
            char str[10] = {0};
            if (isprint(c)) {
                sprintf(str, "%02x (%c)", c, c);
            } else {
                sprintf(str, "%02x", c);
            }
            ROS_INFO("%s", str);
        }
        ROS_INFO("ENDRET");
    }

    std::vector<uint8_t> modify_buffer(const std::string& datatype, const uint8_t *const data, const uint32_t size) {
        // std::deque<uint8_t> raw{data, data + size};
        // std::vector<uint8_t> modified;
        // modified.reserve(size);
        // shrink_buf_recursive(ROSField{datatype + " x"}, raw, modified);
        // return modified;
        std::vector<uint8_t> raw{data, data + size};
        const auto ret = to_can_buf_recursive(ROSField{datatype + " x"}, raw);
        print_buf(ret);
        return ret;
    }

} // namespace IntrospectionHelpers