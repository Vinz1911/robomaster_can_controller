#!/usr/bin/env python3

# Copyright Fraunhofer IML
#
# Licensed under the MIT License.
# For details on the licensing terms, see the LICENSE file.
# SPDX-License-Identifier: MIT

# Import the RoboMaster class from robomaster_can_crontroller_py
from robomaster_can_controller_py import RoboMaster, State
from pypad2 import Gamepad, Keymap
from time import sleep

def callback(state: State):
    """ Callback function to print the RoboMaster states."""
    print(state)

def main():
    # hookup gamepad
    gamepad = Gamepad(path='/dev/input/event2')

    # create RoboMaster object
    robo = RoboMaster()

    # initialize RoboMaster with can interface 'can0'
    if not robo.init("can0"):
        print("Error: Could not init RoboMaster")
        exit(1)

    # register the callback function to print the robomaster states
    # robo.bind(callback)

    # Enable the robomaster to execute drive commands.
    robo.enable()

    # CAUTION: Sleep for a small time to not overfill the can bus communication.
    sleep(0.025)

    while True:
        buttons = gamepad.pressed()
        trigger = 0
        thumb = 0
        for button in buttons:
            if button.name == 'AXE_R2':
                trigger = int(buttons[button])
            if button.name == 'AXE_DX':
                thumb =  int(buttons[button])
            if button.name == 'BTN_TRIANGLE':
                robo.disable()

        speed = trigger
        fr = speed
        fl = speed
        rl = speed
        rr = speed

        if thumb == -1:
            fl = int(fl * 0.5)
            rl = int(rl * 0.5)

        if thumb == 1:
            fr = int(fr * 0.5)
            rr = int(rr * 0.5)

        if speed == 0:
            robo.stop()
        else:
            robo.wheel_rpm(fr, fl, rl, rr)

        sleep(0.025)

if __name__ == '__main__':
    main()
