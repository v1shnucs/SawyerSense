#!/usr/bin/env python

# Code modified from intera_examples
import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb
from std_msgs.msg import String

# Global vars to share between main and callback
g_limb = None
g_pub = None
g_args = None
g_wpt_opts = None

def move_arm():
    """Moves the arm to the photo position."""
    global g_limb, g_pub, g_args, g_wpt_opts
    
    if g_limb is None:
        rospy.logerr("Limb not initialized!")
        return

    try:
        rospy.loginfo("Executing arm move to photo position.")
        traj = MotionTrajectory(limb=g_limb)
        waypoint = MotionWaypoint(options=g_wpt_opts.to_msg(), limb=g_limb)
        joint_angles = g_limb.joint_ordered_angles()

        waypoint.set_joint_angles(joint_angles=joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        if len(g_args['joint_angles']) != len(joint_angles):
            rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
            return

        waypoint.set_joint_angles(joint_angles=g_args['joint_angles'])
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=g_args['timeout'])
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.sleep(0.5)  # Wait for publisher to initialize
            rospy.loginfo('Successfully moved arm for photo')
            g_pub.publish("move_complete") # Publish completion
        else:
            rospy.logerr('Motion controller failed with error %s', result.errorId)
            
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected.')
    except Exception as e:
        rospy.logerr('Error in move_arm: %s', str(e))

def trigger_callback(msg):
    """Callback to move arm when /retake_photo_trigger is received."""
    rospy.loginfo("Received photo retake trigger. Moving arm.")
    move_arm()

def main():
    global g_limb, g_pub, g_args, g_wpt_opts
    
    try:
        rospy.init_node('move_arm_for_photo')
        rospy.loginfo("move_arm_for_photo.py node started.")
        
        g_limb = Limb()
        g_pub = rospy.Publisher('/move_arm_finished', String, queue_size=10)
        
        g_args = {'speed_ratio': 0.5, 
                  'accel_ratio': 0.5, 
                  'timeout': None,
                  'joint_angles': [1.466916015625, -0.0958193359375, -1.5692177734375, 1.5055166015625, 1.448626953125, 1.6323173828125, 3.2174990234375]
                 }

        g_wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=g_args['speed_ratio'],
                                           max_joint_accel=g_args['accel_ratio'])
        
        # --- Run the initial move at startup ---
        rospy.loginfo("Performing initial arm move at startup.")
        move_arm() 
        
        # --- Now, subscribe for future retakes ---
        rospy.Subscriber('/retake_photo_trigger', String, trigger_callback)
        rospy.loginfo("Subscribed to /retake_photo_trigger. Spinning.")
        
        rospy.spin() # Keep the node alive to listen for triggers

    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected. Exiting.')
    rospy.loginfo("move_arm_for_photo.py exiting")

if __name__ == '__main__':
    main()