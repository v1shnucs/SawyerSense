# Vision Model Testing Guide

## Setup

1. Launch the RealSense camera:
```bash
roslaunch realsense2_camera demo_pointcloud.launch
```

2. Start the main service (in a new terminal):
```bash
cd gpt_service
source venv/bin/activate
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

3. Start the vision service (in a new terminal):
```bash
cd gpt_service
source venv/bin/activate
export OPENAI_API_KEY=your_api_key
uvicorn vision_service:app --host 0.0.0.0 --port 8001 --reload
```

4. Run the vision test (in a new terminal):
```bash
export OPENAI_API_KEY=your_api_key && python2 src/gpt_vision/src/scripts/vision_grid_state.py test_photo
```

## Grid Layout

The grid is arranged in three rows:
- Top row: spaces 9,10,11,12
- Middle row: spaces 5,6,7,8
- Bottom row: spaces 1,2,3,4

## Expected Responses

The vision system will respond with one of these formats:

1. For objects detected:
```
Space X has [color] [shape]
```
Example:
```
Space 2 has blue square
Space 7 has red triangle
```

2. For empty grid:
```
Grid is empty - no objects detected
```

3. For unclear images:
```
Unable to analyze image clearly
```

## Testing Different Cameras

### Head Camera
```bash
export OPENAI_API_KEY=your_api_key && python2 src/gpt_vision/src/scripts/vision_grid_state.py test_photo -c head_camera
```

### Arm Camera
```bash
export OPENAI_API_KEY=your_api_key && python2 src/gpt_vision/src/scripts/vision_grid_state.py test_photo -c arm_camera
```

### RealSense Camera (Default)
```bash
export OPENAI_API_KEY=your_api_key && python2 src/gpt_vision/src/scripts/vision_grid_state.py test_photo
```

## Image Processing Details

The vision system:
1. Captures an image from the specified camera
2. Crops the image (crop_x=100, crop_y=125, y_offset=15)
3. Saves the cropped image in src/gpt_vision/src/images/
4. Sends the image to GPT-4V for analysis
5. Returns and publishes the grid state description

## Troubleshooting

1. If vision service fails:
   - Check if both services are running (ports 8000 and 8001)
   - Verify OPENAI_API_KEY is correctly set in all terminals
   - Make sure GPT_SERVICE_URL is set if needed: export GPT_SERVICE_URL=http://localhost:8001

2. If image capture fails:
   - Check if the camera is publishing:
     ```bash
     rostopic list | grep camera
     rostopic echo /camera/color/image_raw -n 1
     ```
   - For Sawyer cameras, verify the robot is enabled:
     ```bash
     rosrun intera_interface enable_robot.py -e
     ```

3. If analysis returns "Unable to analyze image clearly":
   - Check lighting conditions
   - Verify camera focus
   - Ensure grid is within the cropped image area
   - Inspect saved image in src/gpt_vision/src/images/