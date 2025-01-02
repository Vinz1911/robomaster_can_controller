// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/queue_msg.h"
#include "robomaster_can_controller/definitions.h"
#include "gtest/gtest.h"

namespace robomaster_can_controller {
    TEST(QueueTest, PushAndPop) {
        QueueMsg queue;

        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 0, std::vector<uint8_t>{static_cast<uint8_t>(10)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 1, std::vector<uint8_t>{static_cast<uint8_t>(11)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 2, std::vector<uint8_t>{static_cast<uint8_t>(12)}));

        Message m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 0);
        ASSERT_EQ(m.get_payload()[0], 10);
        ASSERT_TRUE(m.is_valid());

        m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 1);
        ASSERT_EQ(m.get_payload()[0], 11);
        ASSERT_TRUE(m.is_valid());

        m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 2);
        ASSERT_EQ(m.get_payload()[0], 12);
        ASSERT_TRUE(m.is_valid());

        m = queue.pop();
        ASSERT_EQ(m.get_sequence(), 0);
        ASSERT_EQ(m.get_payload().size(), 0);
        ASSERT_FALSE(m.is_valid());
    }

    TEST(QueueTest, Overflow) {
        QueueMsg queue;

        for (size_t i = 0; i < queue.max_queue_size() + 1; i++) { queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, i, std::vector<uint8_t>{static_cast<uint8_t>(i)})); }

        Message m = queue.pop();

        ASSERT_EQ(m.get_sequence(), 1);
        ASSERT_EQ(m.get_payload()[0], 1);
        ASSERT_TRUE(m.is_valid());

        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, queue.max_queue_size() + 1, std::vector<uint8_t>{static_cast<uint8_t>(queue.max_queue_size() + 1)}));

        while (!queue.empty()) { m = queue.pop(); }

        ASSERT_EQ(m.get_sequence(), queue.max_queue_size() + 1);
        ASSERT_EQ(m.get_payload()[0], queue.max_queue_size() + 1);
        ASSERT_TRUE(m.is_valid());   
    }

    TEST(QueueTest, Clear) {
        QueueMsg queue;

        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 0, std::vector<uint8_t>{static_cast<uint8_t>(10)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 1, std::vector<uint8_t>{static_cast<uint8_t>(11)}));
        queue.push(Message(DEVICE_ID_MOTION_CONTROLLER, 1337, 2, std::vector<uint8_t>{static_cast<uint8_t>(12)}));

        ASSERT_EQ(queue.size(), 3);
        ASSERT_FALSE(queue.empty());

        queue.clear();

        ASSERT_EQ(queue.size(), 0);
        ASSERT_TRUE(queue.empty());
    }
} // namespace robomaster_can_controller
