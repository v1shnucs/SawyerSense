#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import intera_interface
import argparse
import os

HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
ARM_CAMERA_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"

def callback(data, args):
    try:
        width = data.width
        height = data.height
        encoding = data.encoding
        (filename, camera) = args

        image_data = np.frombuffer(data.data, dtype=np.uint8)
        print("Image data: {} {} {}".format(width, height, encoding))

        if encoding == "rgb8":
            image_array = image_data.reshape((height, width, 3))
            # Convert RGB to BGR for cv2.imwrite
            image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
        elif encoding == "bgr8":
            image_array = image_data.reshape((height, width, 3))
        elif encoding == "bgra8":
            image_array = image_data.reshape((height, width, 4))[:,:,[2,1,0,3]]
            image_array = image_array[:,:,:3]
        else:
            print("Encoding not recognized")
            return

        # Ensure save directory exists
        save_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'images')
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # Save image
        save_path = os.path.join(save_dir, '{}.jpg'.format(filename))
        
        # Crop the image
        crop_x = 100
        crop_y = 125
        y_offset = 15
        cropped_image = image_array[crop_y+y_offset:height-crop_y+y_offset, crop_x:width-crop_x]
        
        cv2.imwrite(save_path, cropped_image)
        rospy.loginfo("Saved image to {0}.jpg".format(filename))
        rospy.signal_shutdown('Photo taken successfully.')

    except Exception as e:
        rospy.logerr("Error: {0}".format(e))

def take_photo(filename, camera):
    rospy.init_node('take_photo', anonymous=True)

    rospy.loginfo("Using camera: {}".format(camera))

    if camera == 'head_camera':
        cameras = intera_interface.Cameras()
        cameras.start_streaming(camera)
        path = HEAD_CAMERA_TOPIC
    elif camera == 'arm_camera':
        cameras = intera_interface.Cameras()
        cameras.start_streaming(camera)
        path = ARM_CAMERA_TOPIC
    else:
        path = REALSENSE_CAMERA_TOPIC

    rospy.loginfo("Subscribing to {}".format(path))
    rospy.Subscriber(path, Image, callback, callback_args=(filename, camera))
    rospy.spin()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Takes photo using Sawyer head camera or RealSense camera'
    )
    parser.add_argument('filename', help='Name of jpg file to be saved')
    parser.add_argument('-c', '--camera', help='"head_camera" or "rs" for RealSense camera, default is rs')
    args = parser.parse_args()
    if args.camera is None:
        args.camera = 'rs'  # Default to RealSense camera

    take_photo(args.filename, args.camera)
