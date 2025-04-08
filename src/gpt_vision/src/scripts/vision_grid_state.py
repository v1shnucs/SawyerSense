#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import json
import cv2
import intera_interface
import argparse
import os
import base64
import urllib2
from threading import Event

# Event to track if arm is ready
arm_ready = Event()

def arm_moved_callback(msg):
    """Callback when arm movement is complete"""
    global arm_ready
    if msg.data == "move_complete":
        rospy.loginfo("Arm is in position for photo")
        arm_ready.set()

# Vision service endpoint
VISION_SERVICE_URL = "http://localhost:8001/vision"

# Camera topics
HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
ARM_CAMERA_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"

# Initialize the publisher as a global variable
grid_state_publisher = None

def encode_image(image_array):
    """Convert numpy array to base64 string"""
    success, encoded_image = cv2.imencode('.jpg', image_array)
    if success:
        # Python 2 compatibility for base64 encoding
        return base64.b64encode(encoded_image.tostring())
    return None

def call_vision_service(image_base64):
    """Call vision service to analyze image"""
    try:
        # Prepare request
        headers = {'Content-Type': 'application/json'}
        data = json.dumps({'image_base64': image_base64})
        
        rospy.loginfo("Sending request to vision service...")
        # Create request
        req = urllib2.Request(
            VISION_SERVICE_URL,
            data=data,
            headers=headers
        )
        
        # Make request
        response = urllib2.urlopen(req, timeout=30)  # 30 second timeout
        result = json.loads(response.read())
        rospy.loginfo("Vision service response received")
        return result['grid_state']
    except Exception as e:
        if hasattr(e, 'read'):
            error_details = e.read()
            try:
                error_json = json.loads(error_details)
                rospy.logerr("Vision service error: %s", error_json.get('detail', str(e)))
            except:
                rospy.logerr("Vision service error: %s", error_details)
        else:
            rospy.logerr("Error calling vision service: %s", str(e))
        return None

def analyze_image_with_gpt4v(image_base64):
    """Send image to vision service for analysis"""
    return call_vision_service(image_base64)

def callback(data, args):
    try:
        width = data.width
        height = data.height
        encoding = data.encoding
        (filename, camera) = args

        image_data = np.frombuffer(data.data, dtype=np.uint8)
        rospy.loginfo("Processing image: %d x %d, %s", width, height, encoding)

        # Convert image data based on encoding
        if encoding == "rgb8":
            image_array = image_data.reshape((height, width, 3))
            image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
        elif encoding == "bgr8":
            image_array = image_data.reshape((height, width, 3))
        elif encoding == "bgra8":
            image_array = image_data.reshape((height, width, 4))[:,:,[2,1,0,3]]
            image_array = image_array[:,:,:3]
        else:
            rospy.logerr("Encoding not recognized")
            return

        # Crop the image
        crop_x = 100
        crop_y = 125
        y_offset = 15
        cropped_image = image_array[crop_y+y_offset:height-crop_y+y_offset, crop_x:width-crop_x]

        # Save directory handling
        save_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'images')
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        save_path = os.path.join(save_dir, '{0}.jpg'.format(filename))
        cv2.imwrite(save_path, cropped_image)

        # Encode and analyze image
        image_base64 = encode_image(cropped_image)
        if image_base64:
            grid_state = analyze_image_with_gpt4v(image_base64)
            if grid_state:
                rospy.loginfo("Grid State: %s", grid_state)
                grid_state_publisher.publish(grid_state)
            else:
                rospy.logerr("Failed to analyze image with GPT-4V")
        else:
            rospy.logerr("Failed to encode image")

        rospy.signal_shutdown('Photo processed and analyzed successfully.')

    except Exception as e:
        rospy.logerr("Error in callback: %s", str(e))

def vision_grid_state(filename, camera):
    global grid_state_publisher, arm_ready
    rospy.init_node('vision_grid_state', anonymous=True)
    
    # Clear the arm ready event
    arm_ready.clear()

    # Initialize the publisher
    grid_state_publisher = rospy.Publisher('/grid_state', String, queue_size=10)
    
    # Subscribe to arm movement completion topic
    rospy.Subscriber('/move_arm_finished', String, arm_moved_callback)
    
    rospy.loginfo("Waiting for arm to move into position...")
    if not arm_ready.wait(timeout=10.0):  # Wait up to 10 seconds
        rospy.logwarn("Timed out waiting for arm to move, proceeding with photo anyway")
    
    rospy.sleep(1)  # Wait for publisher to initialize

    rospy.loginfo("Using camera: %s", camera)

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

    rospy.loginfo("Subscribing to %s", path)
    rospy.Subscriber(path, Image, callback, callback_args=(filename, camera))
    rospy.spin()

if __name__ == "__main__":
    try:
        # Get parameters from ROS parameter server, with defaults
        rospy.init_node('vision_grid_state', anonymous=True)
        filename = rospy.get_param('~filename', 'workspace')
        camera = rospy.get_param('~camera', 'rs')
        
        vision_grid_state(filename, camera)
    except rospy.ROSInterruptException:
        pass