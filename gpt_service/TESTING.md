# Testing Instructions

## Initial Setup
First, run this in every terminal you open:
```bash
cd /home/vishnu/ros_2025
./intera.sh
```

Now open 5 different terminal windows and run these commands in order:

## Terminal 1: Start the GPT Service
```bash
cd gpt_service
source venv/bin/activate
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Terminal 2: Run ROS Node
```bash
# First source your ROS workspace
source devel/setup.bash
# Then start the node with GPT service URL
export GPT_SERVICE_URL=http://localhost:8000
export OPENAI_API_KEY=your-openapi-api-key && python2 src/gpt_vision/src/scripts/get_gpt_response.py
```

## Terminal 3: Run Action Node
```bash
# First source your ROS workspace
source devel/setup.bash
python2 src/robot_action/src/scripts/act_gpt.py
```

## Terminal 4: Publish Grid State
```bash
# First source your ROS workspace
source devel/setup.bash
# Then publish the message
rostopic pub /grid_state std_msgs/String "data: 'Current grid: Square 5 has red circle, Square 9 is empty'" -1
```

## Terminal 5: Publish Transcription
```bash
# First source your ROS workspace
source devel/setup.bash
# Then publish the message
rostopic pub /transcription std_msgs/String "data: 'Move the red circle from space 5 to space 9'" -1
```

Make sure to:
1. Run commands in this exact order
2. Wait for each service/node to fully start before running the next
3. Source the ROS workspace in each terminal that runs ROS commands
4. Have the robot enabled and ready before testing