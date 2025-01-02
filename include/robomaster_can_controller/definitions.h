// Copyright Fraunhofer IML
//
// Licensed under the MIT License.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: MIT

// Modifications by Vinzenz Weist
// Copyright (c) 2024 Vinzenz Weist

#ifndef ROBOMASTER_CAN_CONTROLLER_DEFINITIONS_H_
#define ROBOMASTER_CAN_CONTROLLER_DEFINITIONS_H_

#include <cstdint>
#include <memory>

namespace robomaster_can_controller {
    // definition for the device can id.
    static constexpr uint16_t DEVICE_ID_INTELLI_CONTROLLER = 0x201;
    static constexpr uint16_t DEVICE_ID_MOTION_CONTROLLER = 0x202;
    static constexpr uint16_t DEVICE_ID_GIMBAL = 0x203;
    static constexpr uint16_t DEVICE_ID_HIT_DETECTOR_1 = 0x211;
    static constexpr uint16_t DEVICE_ID_HIT_DETECTOR_2 = 0x212;
    static constexpr uint16_t DEVICE_ID_HIT_DETECTOR_3 = 0x213;
    static constexpr uint16_t DEVICE_ID_HIT_DETECTOR_4 = 0x214;

    // mask of the LED which can be combined with | symbol.
    static constexpr uint16_t LED_MASK_ALL   = 0x000f;
    static constexpr uint16_t LED_MASK_BACK  = 0x0001;
    static constexpr uint16_t LED_MASK_FRONT = 0x0002;
    static constexpr uint16_t LED_MASK_LEFT  = 0x0004;
    static constexpr uint16_t LED_MASK_RIGHT = 0x0008;
} // namespace robomaster_can_controller

#endif // ROBOMASTER_CAN_CONTROLLER_DEFINITIONS_H_