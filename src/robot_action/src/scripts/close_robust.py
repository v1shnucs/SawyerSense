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

def is_closed():
    """Check the closed sensor once; return True if fully closed."""
    status = rospy.wait_for_message(STATE_TOPIC, IODeviceStatus)
    for sig in status.signals:
        if 'closed' in sig.name.lower() and '[true]' in sig.data.lower():
            return True
    return False

def attempt_close(gripper):
    """One shot at closing: toggle valves, then wait for sensor."""
    rospy.loginfo("  Deactivating open valve")
    gripper.set_ee_signal_value('open', False)
    rospy.sleep(SWITCH_DELAY)

    rospy.loginfo("  Activating grip valve")
    gripper.set_ee_signal_value('grip', True)

    # Wait for the closed sensor or timeout
    start = rospy.Time.now()
    while (rospy.Time.now() - start).to_sec() < COMMAND_TIMEOUT:
        if is_closed():
            rospy.loginfo("  Sensor reports closed")
            return True
        rospy.sleep(POLL_RATE)
    rospy.logwarn("  Timeout waiting for 'closed' sensor ({}s)".format(COMMAND_TIMEOUT))
    return False

def close_gripper():
    gripper = get_current_gripper_interface()
    if not isinstance(gripper, SimpleClickSmartGripper):
        rospy.logerr("Not a SimpleClickSmartGripper; aborting")
        return

    # Up to MAX_RETRIES attempts
    for attempt in range(1, MAX_RETRIES + 1):
        rospy.loginfo("Close attempt {}/{}".format(attempt, MAX_RETRIES))
        if attempt_close(gripper):
            rospy.loginfo("Gripper successfully closed on attempt {}".format(attempt))
            return
        # Optionally reinitialize the gripper if it's the last attempt
        if attempt < MAX_RETRIES:
            rospy.loginfo("Retrying after opening and a brief pause...")
            from open_robust import open_gripper as robust_open
            robust_open()
            rospy.sleep(0.5)
    # All retries failed
    rospy.logerr("All {} close attempts failed; consider recalibration".format(MAX_RETRIES))

if __name__ == "__main__":
    try:
        rospy.init_node('close_gripper', anonymous=True)
        close_gripper()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown requested; exiting")
