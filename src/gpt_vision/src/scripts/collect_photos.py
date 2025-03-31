#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import intera_interface
import os
import csv

HEAD_CAMERA_TOPIC = "/io/internal_camera/head_camera/image_rect_color"
ARM_CAMERA_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"
REALSENSE_CAMERA_TOPIC = "/camera/color/image_raw"

# Define valid colors and shapes for column ordering
COLORS = ['y', 'r', 'b', 'g']  # yellow, red, blue, green
SHAPES = ['c', 't', 's']      # circle, triangle, square

def get_ordered_features():
    """Get features in the correct order for CSV columns"""
    features = []
    for space in range(1, 13):
        # Add each color-shape combination in correct order
        for color in COLORS:
            for shape in SHAPES:
                features.append('s{}{}{}'.format(space, color, shape))
        # Add empty state
        features.append('s{}e'.format(space))
    return features

class PhotoCollector:
    def __init__(self, camera='rs'):
        self.camera = camera
        self.photo_count = 0
        self.taking_photo = False
        self.features = get_ordered_features()
        
        # Ensure save directories exist
        self.base_dir = os.path.dirname(os.path.dirname(__file__))
        self.image_dir = os.path.join(self.base_dir, 'training_images')
        self.csv_path = os.path.join(self.base_dir, 'photo_metadata.csv')
        
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
            
        # Create/check CSV file with headers
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['filename', 'camera'] + self.features)
        
        rospy.init_node('photo_collector', anonymous=True)
        
        if camera == 'head_camera':
            cameras = intera_interface.Cameras()
            cameras.start_streaming(camera)
            self.camera_topic = HEAD_CAMERA_TOPIC
        else:
            self.camera_topic = REALSENSE_CAMERA_TOPIC
            
        self.subscriber = None

    def get_grid_state(self):
        """Get the current state of all spaces in the grid"""
        print("\nEnter grid state information:")
        print("For each occupied space, enter: <space_number> <color> <shape>")
        print("Colors: y (yellow), r (red), b (blue), g (green)")
        print("Shapes: c (circle), t (triangle), s (square)")
        print("Example: '1 r c' for red circle in space 1")
        print("Press Enter with no input when done.")
        
        # Initialize all spaces as empty (1 for empty, 0 for others)
        grid_state = {}
        for feature in self.features:
            if feature.endswith('e'):  # Empty state
                grid_state[feature] = 1
            else:
                grid_state[feature] = 0
        
        while True:
            entry = raw_input("Space color shape (or Enter to finish): ").strip()
            if not entry:
                break
                
            try:
                # Split input into space number, color, and shape
                parts = entry.split()
                if len(parts) != 3:
                    print("Invalid format. Use: <space_number> <color> <shape>")
                    continue
                    
                space_num, color, shape = parts
                if not space_num.isdigit() or not (1 <= int(space_num) <= 12):
                    print("Space number must be between 1 and 12")
                    continue
                    
                if color not in COLORS:
                    print("Invalid color. Use: y, r, b, or g")
                    continue
                    
                if shape not in SHAPES:
                    print("Invalid shape. Use: c, t, or s")
                    continue
                
                # Mark the space as occupied
                feature = 's{}{}{}'.format(space_num, color, shape)
                grid_state[feature] = 1
                # Mark the space as not empty
                grid_state['s{}e'.format(space_num)] = 0
                
            except Exception as e:
                print("Error: {}. Please try again.".format(e))
                
        return grid_state

    def image_callback(self, data):
        if not self.taking_photo:
            return
            
        try:
            width = data.width
            height = data.height
            encoding = data.encoding
            
            image_data = np.frombuffer(data.data, dtype=np.uint8)
            
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
            
            # Save image
            filename = "training_{}".format(self.photo_count)
            save_path = os.path.join(self.image_dir, '{}.jpg'.format(filename))
            
            cv2.imwrite(save_path, cropped_image)
            
            # Get grid state
            grid_state = self.get_grid_state()
            
            # Save to CSV in correct column order
            with open(self.csv_path, 'a') as f:
                writer = csv.writer(f)
                row = [filename, self.camera]
                for feature in self.features:
                    row.append(grid_state[feature])
                writer.writerow(row)
                
            rospy.loginfo("Saved image to {}.jpg with metadata".format(filename))
            self.photo_count += 1
            self.taking_photo = False
            
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
            self.taking_photo = False

    def collect_photos(self):
        self.subscriber = rospy.Subscriber(
            self.camera_topic, 
            Image, 
            self.image_callback
        )
        
        rospy.loginfo("Starting photo collection using {}".format(self.camera))
        rospy.loginfo("Press Enter to take a photo, or type 'q' and Enter to quit")
        
        while not rospy.is_shutdown():
            user_input = raw_input()
            if user_input.lower() == 'q':
                break
                
            self.taking_photo = True
            while self.taking_photo and not rospy.is_shutdown():
                rospy.sleep(0.1)
        
        rospy.signal_shutdown('Photo collection completed')

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description='Collect photos and metadata for computer vision training'
    )
    parser.add_argument(
        '-c', '--camera',
        help='"head_camera" or "rs" for RealSense camera, default is rs',
        default='rs'
    )
    args = parser.parse_args()
    collector = PhotoCollector(args.camera)
    collector.collect_photos()