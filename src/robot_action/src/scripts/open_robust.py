#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from intera_interface import SimpleClickSmartGripper, get_current_gripper_interface
from intera_core_msgs.msg import IODeviceStatus

STATE_TOPIC     = '/io/end_effector/stp_021709TP00328/state'
MAX_RETRIES     = 3
COMMAND_TIMEOUT = 5.0   # seconds per attempt
POLL_RATE       = 0.05  # seconds between polls
SWITCH_DELAY    = 0.2   # seconds between valve toggles

def is_open():
    """Check the open sensor once; return True if fully open."""
    status = rospy.wait_for_message(STATE_TOPIC, IODeviceStatus)
    for sig in status.signals:
        if 'open_' in sig.name.lower() and '[true]' in sig.data.lower():
            return True
    return False

def attempt_open(gripper):
    """One shot at opening: toggle valves, then wait for sensor."""
    rospy.loginfo("  Deactivating grip valve")
    gripper.set_ee_signal_value('grip', False)  # release grip valve
    rospy.sleep(SWITCH_DELAY)

    rospy.loginfo("  Activating open valve")
    gripper.set_ee_signal_value('open', True)   # engage open valve

    # Wait for the open sensor or timeout
    start = rospy.Time.now()
    while (rospy.Time.now() - start).to_sec() < COMMAND_TIMEOUT:
        if is_open():
            rospy.loginfo("  Sensor reports open")
            return True
        rospy.sleep(POLL_RATE)
    rospy.logwarn("  Timeout waiting for 'open' sensor ({}s)".format(COMMAND_TIMEOUT))
    return False

def open_gripper():
    gripper = get_current_gripper_interface()
    if not isinstance(gripper, SimpleClickSmartGripper):
        rospy.logerr("Not a SimpleClickSmartGripper; aborting")
        return

    for attempt in range(1, MAX_RETRIES + 1):
        rospy.loginfo("Open attempt {}/{}".format(attempt, MAX_RETRIES))
        if attempt_open(gripper):
            rospy.loginfo("Gripper successfully opened on attempt {}".format(attempt))
            return
        if attempt < MAX_RETRIES:
            rospy.loginfo("Retrying after brief pause...")
            rospy.sleep(0.5)
    rospy.logerr("All {} open attempts failed; consider recalibration".format(MAX_RETRIES))

if __name__ == "__main__":
    try:
        rospy.init_node('open_gripper', anonymous=True)
        open_gripper()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown requested; exiting")
