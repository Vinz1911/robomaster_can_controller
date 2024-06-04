// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Modifications by Vinzenz Weist
// Copyright (c) 2024 Vinzenz Weist

#include "robomaster_can_controller/message.h"
#include "robomaster_can_controller/utils.h"

#include <cassert>
#include <iomanip>

namespace robomaster_can_controller {
    Message::Message(const uint32_t device_id, const std::vector<uint8_t> &data)
        : is_valid_(false),
          device_id_(device_id),
          sequence_(0),
          type_(0)

    {
        if(10 < data.size()) {
            this->type_ = little_endian_to_uint16(data[4], data[5]);
            this->sequence_ = little_endian_to_uint16(data[6], data[7]);
            this->payload_.clear();
            this->payload_.insert(std::begin(this->payload_), std::cbegin(data) + 8, std::cend(data) - 2);
            this->is_valid_ = true;
        }
    }

    Message::Message(const uint32_t device_id, const uint16_t type, const uint16_t sequence, const std::vector<uint8_t> payload)
        : is_valid_(true),
          device_id_(device_id),
          sequence_(sequence),
          type_(type),
          payload_(payload)
    { }

    uint32_t Message::get_device_id() const {
        return this->device_id_;
    }

    uint16_t Message::get_sequence() const {
        return this->sequence_;
    }
    uint16_t Message::get_type() const {
        return this->type_;
    }

    std::vector<uint8_t> Message::get_payload() const {
        return this->payload_;
    }

    size_t Message::get_length() const {
        return this->payload_.size() + 10;
    }

    bool Message::is_valid() const {
        return this->is_valid_;
    }

    void Message::increment_sequence() {
        this->sequence_++;
    }

    void Message::set_value_uint8(const size_t index, const uint8_t value) {
        assert(index < this->payload_.size());
        this->payload_[index] = value;
    }

    void Message::set_value_int8(const size_t index, const int8_t value) {
        assert(index < this->payload_.size());
        this->payload_[index] = value;
    }

    void Message::set_value_uint16(const size_t index, const uint16_t value) {
        assert(index + 1 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
    }

    void Message::set_value_int16(const size_t index, const int16_t value) {
        assert(index + 1 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
    }

    void Message::set_value_uint32(const size_t index, const uint32_t value) {
        assert(index + 3 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
        this->payload_[index + 2] = static_cast<uint8_t>(value >> 16);
        this->payload_[index + 3] = static_cast<uint8_t>(value >> 24);
    }

    void Message::set_value_int32(const size_t index, const int32_t value) {
        assert(index + 3 < this->payload_.size());
        this->payload_[index] = static_cast<uint8_t>(value);
        this->payload_[index + 1] = static_cast<uint8_t>(value >> 8);
        this->payload_[index + 2] = static_cast<uint8_t>(value >> 16);
        this->payload_[index + 3] = static_cast<uint8_t>(value >> 24);
    }

    void Message::set_value_float(const size_t index, const float value) {
        assert(index + 3 < this->payload_.size());
        float non_const_value = value;
        const uint32_t float_bits = *reinterpret_cast<uint32_t*>(&non_const_value);
        this->set_value_uint32(index, float_bits);
    }

    uint8_t Message::get_value_uint8(const size_t index) const {
        assert(index < this->payload_.size());
        return this->payload_[index];
    }

    int8_t Message::get_value_int8(const size_t index) const {
        assert(index < this->payload_.size());
        return this->payload_[index];
    }

    uint16_t Message::get_value_uint16(const size_t index) const {
        assert(index + 1 < this->payload_.size());
        uint16_t value = this->payload_[index + 1];
        value = value << 8 | this->payload_[index];
        return value;
    }

    int16_t Message::get_value_int16(const size_t index) const {
        assert(index + 1 < this->payload_.size());
        int16_t value = this->payload_[index + 1];
        value = value << 8 | this->payload_[index];
        return value;
    }

    uint32_t Message::get_value_uint32(const size_t index) const {
        assert(index + 3 < this->payload_.size());
        uint32_t value = this->payload_[index + 3];
        value = value << 8 | this->payload_[index + 2];
        value = value << 8 | this->payload_[index + 1];
        value = value << 8 | this->payload_[index];
        return value;
    }

    int32_t Message::get_value_int32(const size_t index) const {
        assert(index + 3 < this->payload_.size());
        int32_t value = this->payload_[index + 3];
        value = value << 8 | this->payload_[index + 2];
        value = value << 8 | this->payload_[index + 1];
        value = value << 8 | this->payload_[index];
        return value;
    }

    float Message::get_value_float(const size_t index) const {
        assert(index + 3 < this->payload_.size());
        uint32_t value = this->get_value_uint32(index);
        return *reinterpret_cast<float*>(&value);
    }

    void Message::set_payload(const std::vector<uint8_t> &payload) {
        this->payload_ = payload;
    }

    std::vector<uint8_t> Message::to_vector() const {
        std::vector<uint8_t> vector;
        if (this->is_valid_) {
            // header, crc usw + payload
            vector.resize(10 + this->payload_.size());
            vector[0] = 0x55;
            vector[1] = static_cast<uint8_t>(vector.size());
            vector[2] = 0x04;
            vector[3] = calculate_crc8(vector.data(), 3);
            vector[4] = static_cast<uint8_t>(this->type_);
            vector[5] = static_cast<uint8_t>(this->type_ >> 8);
            vector[6] = static_cast<uint8_t>(this->sequence_);
            vector[7] = static_cast<uint8_t>(this->sequence_ >> 8);

            for (size_t i = 0; i < this->payload_.size(); i++) { vector[8 + i] = this->payload_[i]; }
            const uint16_t crc16 = calculate_crc16(vector.data(), vector.size() - 2);

            vector[vector.size() - 2] = static_cast<uint8_t>(crc16);
            vector[vector.size() - 1] = static_cast<uint8_t>(crc16 >> 8);
        }
        return vector;
    }

    std::ostream& operator<<(std::ostream& os, const Message& msg) {
        os << "Message( 0x"
           << std::setfill('0') << std::setw(4) << std::hex << msg.get_device_id() << ", 0x"
           << std::setfill('0') << std::setw(4) << std::hex << msg.get_type() << ", "
           << std::setfill(' ') << std::setw(5) << std::dec << msg.get_sequence() << ", { ";

        if (!msg.get_payload().empty()) {
            os << "0x";
            for (size_t i = 0; i < msg.get_payload().size(); i++) {
                os << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(msg.get_payload()[i]);
                if (i < msg.get_payload().size() - 1) { os << ", 0x"; }
            }
        }
        os << " })" << std::dec;
        return os;
    }
} // namespace robomaster_can_controller