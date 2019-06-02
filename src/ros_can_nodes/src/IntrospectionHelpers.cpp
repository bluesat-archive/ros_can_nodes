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
     * Transfer bytes from start of src buffer to end of dst buffer
     */
    static constexpr auto transfer_bytes = [](std::vector<uint8_t>& src, std::vector<uint8_t>& dst, const uint32_t len) {
        dst.insert(dst.cend(), src.cbegin(), src.cbegin() + len);
        src.erase(src.cbegin(), src.cbegin() + len);
    };

    /**
     * Extracts and returns the first 4 bytes of the buffer as a uint32, deleting it from buf
     */
    static constexpr auto extract_uint32 = [](std::vector<uint8_t>& buf) {
        const auto n = *reinterpret_cast<uint32_t *>(buf.data());
        buf.erase(buf.cbegin(), buf.cbegin() + sizeof(n));
        return n;
    };

    /**
     * Prepends the buffer given a pointer and length
     */
    static constexpr auto prepend_buf = [](std::vector<uint8_t>& buf, const void *const ptr, const uint32_t len) {
        const auto ptr_u8 = static_cast<const uint8_t *const>(ptr);
        buf.insert(buf.cbegin(), ptr_u8, ptr_u8 + len);
    };

    /**
     * Prepends the buffer with a uint16 which represents its length in bytes
     */
    static constexpr auto prepend_buf_len = [](std::vector<uint8_t>& buf) {
        const auto len = static_cast<uint16_t>(buf.size());
        prepend_buf(buf, &len, sizeof(len));
    };

    /**
     * Prepends the buffer with a uint32
     */
    static constexpr auto prepend_uint32 = [](std::vector<uint8_t>& buf, const uint32_t n) {
        prepend_buf(buf, &n, sizeof(n));
    };

    /**
     * Extracts the first 2 bytes of the buffer as a uint16, then extracts that many bytes
     * into a new buffer and returns it, deleting all extracted bytes from buf
     */
    static constexpr auto extract_uint16_len_data = [](std::vector<uint8_t>& buf) {
        const auto len = *reinterpret_cast<uint16_t *>(buf.data());
        const auto data_start = buf.cbegin() + sizeof(len);
        const auto data_end = data_start + len;
        const std::vector<uint8_t> new_buf{data_start, data_end};
        buf.erase(buf.cbegin(), data_end);
        return new_buf;
    };

    /**
     * Recursively transforms the buffer to CAN specific buffer using msg_fields
     * as a message type template
     */
    std::vector<uint8_t> to_can_buf_recursive(const ROSField& field, std::vector<uint8_t>& buf) {
        std::vector<uint8_t> tmp_buf;
        tmp_buf.reserve(buf.size() + sizeof(uint32_t));
        
        const auto& type = field.type();

        if (field.isArray()) {
            auto len = field.arraySize();
            if (len == -1) { // varied length array
                // varied length array types stored as 'array length' then contents
                len = extract_uint32(buf);
            }

            // iterate through size, recursively transform buffer
            const auto non_array_field = ROSField{type.baseName() + " x"};
            for (auto i = 0;i < len;++i) {
                const auto tmp_buf_i = to_can_buf_recursive(non_array_field, buf);
                tmp_buf.insert(tmp_buf.cend(), tmp_buf_i.cbegin(), tmp_buf_i.cend());
            }
            prepend_buf_len(tmp_buf);
        } else if (type.typeID() == BuiltinType::STRING) {
            // string is stored as 'length' then string contents (no null terminator)
            const auto len = extract_uint32(buf);
            transfer_bytes(buf, tmp_buf, len);

            // add null terminator
            tmp_buf.push_back(0);
            prepend_buf_len(tmp_buf);
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
     * Recursively transforms the buffer to ROS specific buffer using msg_fields
     * as a message type template
     */
    std::vector<uint8_t> to_ros_buf_recursive(const ROSField& field, std::vector<uint8_t>& buf) {
        std::vector<uint8_t> tmp_buf;
        tmp_buf.reserve(buf.size() + sizeof(uint32_t));
        
        const auto& type = field.type();

        if (field.isArray()) {
            auto new_buf = extract_uint16_len_data(buf);

            const auto non_array_field = ROSField{type.baseName() + " x"};

            // keep recursively extracting items from the array contents, keeping track of item count
            auto element_count = 0u;
            while (new_buf.size() != 0) {
                const auto tmp_buf_i = to_ros_buf_recursive(non_array_field, new_buf);
                tmp_buf.insert(tmp_buf.cend(), tmp_buf_i.cbegin(), tmp_buf_i.cend());
                ++element_count;
            }

            // prepend element count if varied length array
            if (field.arraySize() == -1) {
                prepend_uint32(tmp_buf, element_count);
            }
        } else if (type.typeID() == BuiltinType::STRING) {
            auto new_buf = extract_uint16_len_data(buf);

            // strings have null terminators in can buffer but not in ros buffer
            transfer_bytes(new_buf, tmp_buf, new_buf.size() - 1);
            prepend_uint32(tmp_buf, tmp_buf.size());
        } else if (type.typeID() == BuiltinType::OTHER) {
            // non-primitive internal message type
            // iterate through type's fields and modify contents recursively
            for (const auto& field : msg_fields[type.baseName()]) {
                const auto tmp_buf_i = to_ros_buf_recursive(field, buf);
                tmp_buf.insert(tmp_buf.cend(), tmp_buf_i.cbegin(), tmp_buf_i.cend());
            }
        } else {
            // non-string primitives - directly transfer bytes
            transfer_bytes(buf, tmp_buf, type.typeSize());
        }
        return tmp_buf;
    }

    void register_message(const std::string& datatype, const std::string& definition) {
        std::lock_guard<std::mutex> registration_lock{registration_mutex};

        // register with RosIntrospection parser to create type list
        parser.registerMessageDefinition(datatype, ROSType{datatype}, definition);

        // store fields structure for each message type found
        for (const auto& t : parser.getMessageInfo(datatype)->type_list) {
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

    void print_registered() {
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
        ROS_INFO("ENDBUF");
    }

    std::vector<uint8_t> to_can_buf(const std::string& datatype, const uint8_t *const data, const uint32_t size) {
        std::vector<uint8_t> raw{data, data + size};
        return to_can_buf_recursive(ROSField{datatype + " x"}, raw);
    }

    std::vector<uint8_t> to_ros_buf(const std::string& datatype, const uint8_t *const data, const uint32_t size) {
        std::vector<uint8_t> raw{data, data + size};
        return to_ros_buf_recursive(ROSField{datatype + " x"}, raw);
    }

} // namespace IntrospectionHelpers