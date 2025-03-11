#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import intera_interface
import argparse
import os

# Camera topics
HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
ARM_CAMERA_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"  # not functional, needs conversion from mono8 encoding
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"

def callback(data, args):
    try:
        width = data.width
        height = data.height
        encoding = data.encoding
        (filename, camera, save_dir) = args

        image_data = np.frombuffer(data.data, dtype=np.uint8)

        rospy.loginfo("Image Data: {} x {} pixels, encoding: {}".format(width, height, encoding))

        if encoding == "bgr8":
            image_array = image_data.reshape((height, width, 3))
        elif encoding == "rgb8":
            image_array = image_data.reshape((height, width, 3))[:,:, ::-1]
        elif encoding == "bgra8":
            image_array = image_data.reshape((height, width, 4))[:,:,[2,1,0,3]]
            image_array = image_array[:,:,:3]
        else:
            rospy.logerr("Encoding not recognized: {}".format(encoding))
            return

        # Ensure save directory exists
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # Save image
        save_path = os.path.join(save_dir, '{}.jpg'.format(filename))
        cv2.imwrite(save_path, image_array)
        rospy.loginfo("Image saved to: {}".format(save_path))

        rospy.signal_shutdown('Image captured successfully.')
        
    except Exception as e:
        rospy.logerr("Error in callback: {0}".format(e))
        rospy.signal_shutdown('Error in image capture.')

def take_photo(filename, camera, save_dir):
    """
    Take a photo using the specified camera.
    
    Args:
        filename (str): Name for the saved image file (without extension)
        camera (str): Either 'head_camera' or 'rs' for RealSense camera
        save_dir (str): Directory to save the image
    """
    rospy.init_node('take_photo', anonymous=True)

    if camera == 'head_camera':
        # Initialize and start head camera streaming
        cameras = intera_interface.Cameras()
        cameras.start_streaming(camera)
        camera_topic = HEAD_CAMERA_TOPIC
    else:
        # Use RealSense camera
        camera_topic = REALSENSE_CAMERA_TOPIC

    rospy.loginfo("Subscribing to camera topic: {}".format(camera_topic))
    rospy.Subscriber(camera_topic, Image, callback, callback_args=(filename, camera, save_dir))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Program terminated by user.')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Takes photo using Sawyer\'s cameras.',
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument('filename', 
                        help='Name of image file to be saved (without extension)')
    
    parser.add_argument('-c', '--camera', 
                        choices=['head_camera', 'rs'],
                        default='head_camera',
                        help='Camera to use:\n'
                             'head_camera: Sawyer\'s head camera\n'
                             'rs: RealSense camera')
    
    parser.add_argument('-d', '--dir',
                        default=os.path.join(os.path.dirname(__file__), '../images'),
                        help='Directory to save images (default: ../images)')

    args = parser.parse_args()

    take_photo(args.filename, args.camera, args.dir)