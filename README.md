# SawyerSense

## System Startup Instructions

Follow these steps in order to start the complete system:

### 1. Start the Webcam
First, start the RealSense camera for point cloud visualization:
```bash
# Read the startup script first (optional)
./start.txt

# Launch RealSense camera
roslaunch realsense2_camera demo_pointcloud.launch
```

### 2. Start GPT Service (Terminal 1)
Open a new terminal and start the main GPT service:
```bash
# Source the Intera environment
./intera.sh

# Navigate to GPT service directory
cd gpt_service

# Activate Python virtual environment
source venv/bin/activate

# Set OpenAI API key (replace with your actual key)
export OPENAI_API_KEY=<your-openai-api-key>

# Start the main service (choose one option)
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
# OR for step-by-step mode:
# uvicorn main_step:app --host 0.0.0.0 --port 8000 --reload
```

### 3. Start Vision Service (Terminal 2)
Open another new terminal and start the vision service:
```bash
# Source the Intera environment
./intera.sh

# Navigate to GPT service directory
cd gpt_service

# Activate Python virtual environment
source venv/bin/activate

# Set OpenAI API key (replace with your actual key)
export OPENAI_API_KEY=<your-openai-api-key>

# Start the vision service
uvicorn vision_service:app --host 0.0.0.0 --port 8001 --reload
```

### 4. Launch Complete System (Terminal 3)
Open a final terminal and launch the ROS system:
```bash
# Launch the complete system (choose one option)
roslaunch gpt_vision complete_system.launch
# OR for step-by-step mode:
# roslaunch gpt_vision complete_system_step.launch
```

### System Workflow
Once all services are running, the system will:
1. Enable the robot
2. Move the arm to the photo position
3. Wait for the arm to be fully positioned
4. Take a photo of the workspace using RealSense camera
5. Start listening for voice commands
6. Process commands through GPT and execute robot actions
7. Provide voice feedback through text-to-speech

## Setup

1. Clone the repository and its submodules
```bash
git clone https://github.com/v1shnucs/ros_2025.git
cd ros_2025
git submodule update --init --recursive
```

2. Set up environment variables
```bash
# Copy the sample environment file
cp .env.sample .env

# Edit .env file and add your OpenAI API key
# Replace 'your_api_key_here' with your actual OpenAI API key
```

3. Source the ROS workspace
```bash
source devel/setup.bash
```


## Features

- Integration with OpenAI GPT API for natural language processing
- Robot control scripts for grab and place actions
- Gripper control functionality
- Proper error handling and logging
- Safe trajectory planning with prep positions

## Notes

- The `.env` file containing your API key is automatically ignored by git for security
- Make sure to keep your API key confidential and never commit it to version control
- If you receive an error about OPENAI_API_KEY not being set, ensure you've properly set up the .env file
- The repository includes Sawyer Robot dependencies as submodules
