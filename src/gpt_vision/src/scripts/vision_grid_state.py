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

# Vision service endpoint
VISION_SERVICE_URL = "http://localhost:8001/vision"

# Camera topics
HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
ARM_CAMERA_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"

# Global variables
grid_state_publisher = None
g_filename = 'workspace'
g_camera = 'rs'
g_camera_topic = REALSENSE_CAMERA_TOPIC
g_cameras_interface = None # For Sawyer cameras

def encode_image(image_array):
    """Convert numpy array to base64 string"""
    success, encoded_image = cv2.imencode('.jpg', image_array)
    if success:
        return base64.b64encode(encoded_image.tostring())
    return None

def call_vision_service(image_base64):
    """Call vision service to analyze image"""
    try:
        headers = {'Content-Type': 'application/json'}
        data = json.dumps({'image_base64': image_base64})
        
        rospy.loginfo("Sending request to vision service...")
        req = urllib2.Request(VISION_SERVICE_URL, data, headers)
        response = urllib2.urlopen(req, timeout=30)
        result = json.loads(response.read())
        rospy.loginfo("Vision service response received")
        return result['grid_state']
    except Exception as e:
        rospy.logerr("Error calling vision service: %s", str(e))
        return None

def analyze_image_with_gpt4v(image_base64):
    """Send image to vision service for analysis"""
    return call_vision_service(image_base64)

def process_image_data(data, args):
    """Processes a single image frame to get grid state."""
    global grid_state_publisher
    try:
        width = data.width
        height = data.height
        encoding = data.encoding
        (filename, camera) = args

        image_data = np.frombuffer(data.data, dtype=np.uint8)
        rospy.loginfo("Processing image: %d x %d, %s", width, height, encoding)

        if encoding == "rgb8":
            image_array = image_data.reshape((height, width, 3))
            image_array = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
        elif encoding == "bgr8":
            image_array = image_data.reshape((height, width, 3))
        else: # Add simplified logic for other formats
            image_array = image_data.reshape((height, width, -1))
            if image_array.shape[2] == 4: # Handle BGRA/RGBA
                image_array = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)
            elif image_array.shape[2] == 1: # Handle grayscale
                image_array = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
            else:
                 rospy.logerr("Encoding not recognized: %s", encoding)
                 return

        # Crop the image
        crop_x = 100
        crop_y = 125
        y_offset = 15
        cropped_image = image_array[crop_y+y_offset:height-crop_y+y_offset, crop_x:width-crop_x]

        # Save directory
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
                grid_state_publisher.publish(grid_state) # Publish the new state
            else:
                rospy.logerr("Failed to analyze image with GPT-4V")
        else:
            rospy.logerr("Failed to encode image")

        # --- DO NOT SHUT DOWN ---
        rospy.loginfo('Photo processed. Ready for next /move_arm_finished trigger.')

    except Exception as e:
        rospy.logerr("Error in process_image_data: %s", str(e))


def arm_moved_callback(msg):
    """Callback when arm movement is complete. This is our trigger to take a photo."""
    global g_filename, g_camera, g_camera_topic
    if msg.data == "move_complete":
        rospy.loginfo("Arm is in position. Triggering photo capture.")
        try:
            # Get a single image message
            image_data = rospy.wait_for_message(g_camera_topic, Image, timeout=5.0)
            # Process it
            process_image_data(image_data, (g_filename, g_camera))
        except rospy.ROSException as e:
            rospy.logerr("Failed to get image from topic %s: %s", g_camera_topic, e)
        except Exception as e:
            rospy.logerr("Error during image capture: %s", str(e))


def vision_grid_state(filename, camera):
    global grid_state_publisher, g_filename, g_camera, g_camera_topic, g_cameras_interface
    rospy.init_node('vision_grid_state', anonymous=True)
    
    g_filename = filename
    g_camera = camera
    
    # Latch=True ensures the last published grid state is available to new subscribers
    grid_state_publisher = rospy.Publisher('/grid_state', String, queue_size=10, latch=True)
    
    rospy.loginfo("vision_grid_state node started.")
    rospy.loginfo("Using camera: %s", camera)

    # Setup camera stream
    if camera == 'head_camera':
        g_cameras_interface = intera_interface.Cameras()
        if not g_cameras_interface.start_streaming(camera):
             rospy.logerr("Failed to start head_camera stream.")
             return
        g_camera_topic = HEAD_CAMERA_TOPIC
    elif camera == 'arm_camera':
        g_cameras_interface = intera_interface.Cameras()
        if not g_cameras_interface.start_streaming(camera):
             rospy.logerr("Failed to start arm_camera stream.")
             return
        g_camera_topic = ARM_CAMERA_TOPIC
    else:
        g_camera_topic = REALSENSE_CAMERA_TOPIC
    
    rospy.loginfo("Camera topic set to: %s", g_camera_topic)
    
    # Subscribe to arm movement completion topic. This is the node's trigger.
    rospy.Subscriber('/move_arm_finished', String, arm_moved_callback)
    
    rospy.loginfo("Waiting for /move_arm_finished triggers...")
    rospy.spin() # Keep alive
    
    # Cleanup on shutdown
    if g_cameras_interface:
        rospy.loginfo("Stopping camera stream.")
        g_cameras_interface.stop_streaming(camera)

if __name__ == "__main__":
    try:
        # Parameters are now set in the launch file
        filename = rospy.get_param('~filename', 'workspace')
        camera = rospy.get_param('~camera', 'rs')
        vision_grid_state(filename, camera)
    except rospy.ROSInterruptException:
        pass