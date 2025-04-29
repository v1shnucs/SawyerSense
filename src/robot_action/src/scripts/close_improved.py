#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Cuff
)
from intera_core_msgs.msg import IODeviceStatus, IODataStatus

# Device-specific state topic (replace with your gripper ID)
STATE_TOPIC = '/io/end_effector/stp_021709TP00328/state'

# Timeout and polling settings
COMMAND_TIMEOUT = 5.0  # seconds
POLL_RATE = 0.05       # seconds

def wait_for_closed(timeout=COMMAND_TIMEOUT):
    """
    Blocks until the 'closed' sensor reads true in state.signals,
    or until timeout elapses.
    """
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        # Receive the device status message
        device_status = rospy.wait_for_message(STATE_TOPIC, IODeviceStatus)
        # Look through each signal for the "closed" sensor
        for sig in device_status.signals:  # IODataStatus entries :contentReference[oaicite:1]{index=1}
            if 'closed' in sig.name.lower():
                # sig.data is a JSON-like string, e.g. "[true]" or "[false]"
                if '[true]' in sig.data.lower():
                    rospy.loginfo("Gripper mechanically closed")
                    return True
        # Timeout guard
        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("Timeout waiting for 'closed' sensor")
            return False
        rospy.sleep(POLL_RATE)

def close_gripper():
    """
    Closes the ClickSmart gripper by toggling valves
    and waiting for the closed sensor.
    """
    rospy.init_node('close_gripper', anonymous=True)

    # Obtain the ClickSmart interface
    gripper = get_current_gripper_interface()
    if not isinstance(gripper, SimpleClickSmartGripper):
        rospy.logerr("Interface is not SimpleClickSmartGripper")
        return

    # Optional: use the cuff if you want button control
    cuff = Cuff()

    # 1. Deactivate the 'open' valve
    rospy.loginfo("Deactivating open valve")
    gripper.set_ee_signal_value('open', False)

    # 2. Activate the 'grip' valve
    rospy.loginfo("Activating grip valve")
    gripper.set_ee_signal_value('grip', True)

    # 3. Wait for the mechanical 'closed' sensor
    if wait_for_closed():
        rospy.loginfo("Gripper successfully closed")
    else:
        rospy.logerr("Gripper failed to close within timeout")

if __name__ == "__main__":
    try:
        close_gripper()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown requested; exiting")  # clean exit
