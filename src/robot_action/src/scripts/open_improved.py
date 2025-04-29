#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Cuff
)
from intera_core_msgs.msg import IODeviceStatus

# Device-specific state topic (replace with your gripper ID)
STATE_TOPIC = '/io/end_effector/stp_021709TP00328/state'

# Timeout and polling settings
COMMAND_TIMEOUT = 5.0  # seconds
POLL_RATE = 0.05       # seconds

def wait_for_open(timeout=COMMAND_TIMEOUT):
    """
    Blocks until the 'open' sensor reads true in state.signals,
    or until timeout elapses.
    """
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        device_status = rospy.wait_for_message(STATE_TOPIC, IODeviceStatus)
        for sig in device_status.signals:
            if 'open_' in sig.name.lower():
                # sig.data is a JSON-like string, e.g. "[true]" or "[false]"
                if '[true]' in sig.data.lower():
                    rospy.loginfo("Gripper mechanically open")
                    return True
        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("Timeout waiting for 'open' sensor")
            return False
        rospy.sleep(POLL_RATE)


def open_gripper():
    """
    Opens the ClickSmart gripper by toggling valves
    and waiting for the open sensor.
    """
    rospy.init_node('open_gripper', anonymous=True)

    gripper = get_current_gripper_interface()
    if not isinstance(gripper, SimpleClickSmartGripper):
        rospy.logerr("Interface is not SimpleClickSmartGripper")
        return

    cuff = Cuff()

    # 1. Deactivate the 'grip' valve
    rospy.loginfo("Deactivating grip valve")
    gripper.set_ee_signal_value('grip', False)
    rospy.sleep(0.1)

    # 2. Activate the 'open' valve
    rospy.loginfo("Activating open valve")
    gripper.set_ee_signal_value('open', True)

    # 3. Wait for the mechanical 'open' sensor
    if wait_for_open():
        rospy.loginfo("Gripper successfully opened")
    else:
        rospy.logerr("Gripper failed to open within timeout")


if __name__ == "__main__":
    try:
        open_gripper()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown requested; exiting")  
