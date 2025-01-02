// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

#include "robomaster_can_controller/utils.h"
#include "robomaster_can_controller/message.h"
#include "robomaster_can_controller/definitions.h"
#include "gtest/gtest.h"

namespace robomaster_can_controller {
    const static Message MSG_ENABLE = Message( DEVICE_ID_INTELLI_CONTROLLER, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x01 });
    const static Message MSG_DISABLE = Message( DEVICE_ID_INTELLI_CONTROLLER, 0xc309, 0, { 0x40, 0x3f, 0x19, 0x00 });

    TEST(UtilTest, Little) {
        uint8_t lsb = 0xAD;
        uint8_t msb = 0xDE;
        ASSERT_EQ(little_endian_to_uint16(lsb, msb), 0xDEAD);
    }

    TEST(UtilTest, calculate_crc16) {
        std::vector<uint8_t> vector_enable = MSG_ENABLE.to_vector();
        std::vector<uint8_t> vector_disable = MSG_DISABLE.to_vector();

        ASSERT_EQ(calculate_crc16(vector_enable.data(), vector_enable.size() -2), calculate_crc16(vector_enable.data(), vector_enable.size() - 2));
        ASSERT_NE(calculate_crc16(vector_enable.data(), vector_enable.size() -2), calculate_crc16(vector_disable.data(), vector_disable.size() - 2));

        const uint16_t crc16 = calculate_crc16(vector_enable.data(), vector_enable.size() -2);
        vector_enable[10]++;

        ASSERT_NE(calculate_crc16(vector_enable.data(), vector_enable.size() - 2), crc16);
    }

    TEST(UtilTest, calculate_crc8) {
        std::vector<uint8_t> vector_enable = MSG_ENABLE.to_vector();
        std::vector<uint8_t> vector_disable = MSG_DISABLE.to_vector();

        ASSERT_EQ(calculate_crc8(vector_enable.data(), vector_enable.size() -2), calculate_crc8(vector_enable.data(), vector_enable.size() - 2));
        ASSERT_NE(calculate_crc8(vector_enable.data(), vector_enable.size() -2), calculate_crc8(vector_disable.data(), vector_disable.size() - 2));

        const uint8_t crc8 = calculate_crc8(vector_enable.data(), vector_enable.size() -2);
        vector_enable[10]++;

        ASSERT_NE(calculate_crc8(vector_enable.data(), vector_enable.size() - 2), crc8);
    }

    TEST(UtilTest, clip) {
        ASSERT_FLOAT_EQ(clip<float>(-10.0f, -1.0f, 1.0f), -1.0f);
        ASSERT_FLOAT_EQ(clip<float>( -1.0f, -1.0f, 1.0f), -1.0f);
        ASSERT_FLOAT_EQ(clip<float>(  0.0f, -1.0f, 1.0f),  0.0f);
        ASSERT_FLOAT_EQ(clip<float>(  1.0f, -1.0f, 1.0f),  1.0f);
        ASSERT_FLOAT_EQ(clip<float>( 10.0f, -1.0f, 1.0f),  1.0f);

        ASSERT_FLOAT_EQ(clip<int>(-101, -100, 100), -100);
        ASSERT_FLOAT_EQ(clip<int>(-100, -100, 100), -100);
        ASSERT_FLOAT_EQ(clip<int>(   0, -100, 100),    0);
        ASSERT_FLOAT_EQ(clip<int>( 100, -100, 100),  100);
        ASSERT_FLOAT_EQ(clip<int>( 101, -100, 100),  100);
    }
} // namespace robomaster_can_controller
