// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Modifications by Vinzenz Weist
// Copyright (c) 2024 Vinzenz Weist

#include "robomaster_can_controller/can_socket.h"

#include <string.h>
#include <iostream>
#include <math.h>

namespace robomaster_can_controller {

CanSocket::CanSocket(): socket_(0) {
    memset(&this->ifr_, 0x0, sizeof(this->ifr_));
    memset(&this->addr_, 0x0, sizeof(this->addr_));
}

CanSocket::~CanSocket() {
    close(socket_);
}

void CanSocket::setTimeout(const size_t seconds, const size_t microseconds) {
    timeval t;
    t.tv_sec = seconds;
    t.tv_usec = microseconds;
    setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof(t));
}

void CanSocket::setTimeout(const double seconds) {
    if (seconds < 0.0) {
        const size_t seconds_t = static_cast<size_t>(std::floor(seconds));
        const size_t microseconds_t = static_cast<size_t>((seconds - std::floor(seconds)) * 1e6);
        this->setTimeout(seconds_t, microseconds_t);
    } else {
        this->setTimeout(0,0);
    }
}

bool CanSocket::init(const std::string &can_interface) {
    this->socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(this->socket_ < 0) {
        std::printf("[CAN]: Failed to open Socket\n");
        return false;
    }

    memcpy(this->ifr_.ifr_name, can_interface.c_str(), can_interface.size());
    if(ioctl(this->socket_, SIOGIFINDEX, &this->ifr_) < 0) {
        std::printf("[CAN]: Failed to request interface %s\n", can_interface);
        return false;
    }

    this->addr_.can_ifindex = this->ifr_.ifr_ifindex;
    this->addr_.can_family= PF_CAN;

    if(bind(this->socket_, (struct sockaddr *)&this->addr_, sizeof(this->addr_)) < 0) {
        std::printf("[CAN]: Failed to bind the address\n");
        return false;
    }
    return true;
}

bool CanSocket::sendFrame(const uint32_t id, const uint8_t data[8], const size_t length) {
    if (length <= 8) {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));

        frame.can_id = id;
        frame.can_dlc = length;
        memcpy((uint8_t *) frame.data, data, length);
        
        if(write(this->socket_, &frame, sizeof(frame)) < 0) {
            std::printf("[CAN]: Failed to send frame\n");
            return false;
        }
    } else {
        std::printf("[CAN]: Failed to send frame\n");
        return false;
    }
    return true;
}

bool CanSocket::readFrame(uint32_t &id, uint8_t data[8], size_t &length) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    if(read(this->socket_, &frame, sizeof(frame)) < 0) {
        std::printf("[CAN]: Failed to read frame\n");
        return false;
    }

    id = (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK): (frame.can_id & CAN_SFF_MASK);
    length = frame.can_dlc;
    memcpy(data, frame.data, length);
    return true;
}

} // namespace robomaster_can_controller