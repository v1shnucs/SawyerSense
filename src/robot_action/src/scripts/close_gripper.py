#!/usr/bin/env python

import intera_interface
import rospy

from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Cuff
)

VALID_LIMB = "right"

def close_gripper(arm=VALID_LIMB):
    """
    Close the gripper.

    Args:
        arm (str): Name of limb to use as gripper
    """ 
    try:
        cuff = Cuff(limb=arm)
        gripper = get_current_gripper_interface()
        is_clicksmart = isinstance(gripper, SimpleClickSmartGripper)

        if is_clicksmart:
            if gripper.needs_init():
                print("Clicksmart gripper needs initialization")
                gripper.initialize()
        else:
            if not (gripper.is_calibrated() or gripper.calibrate() == True):
                raise Exception("Gripper calibration failed")
        
        gripper.set_ee_signal_value('grip', True)
        print("Gripper closed")
        
    except Exception as e:
        print("Error in close_gripper: %s" % str(e))
        raise

if __name__ == "__main__":
    try:
        rospy.init_node("close_gripper", anonymous=True, log_level=rospy.ERROR)
        print("close_gripper.py entered")
        close_gripper()
        print("close_gripper.py exiting")
    except Exception as e:
        print("Error: %s" % str(e))