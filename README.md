# SawyerSense

## Instructions for Using the System

1. **Set Up GPT Services**:
   - Start the GPT services in two terminals:
     ```bash
     # Terminal 1
     cd gpt_service
     source venv/bin/activate
     export OPENAI_API_KEY=<your-openai-api-key>
     uvicorn main:app --host 0.0.0.0 --port 8000 --reload

     # Terminal 2
     cd gpt_service
     source venv/bin/activate
     export OPENAI_API_KEY=<your-openai-api-key>
     uvicorn vision_service:app --host 0.0.0.0 --port 8001 --reload
     ```

2. **Launch the System**:
   - Copy the template launch file and update it with your API key and service URLs:
     ```bash
     cp src/gpt_vision/launch/complete_system.launch.template src/gpt_vision/launch/complete_system.launch
     ```

   - Edit `src/launch/complete_system.launch` to include:
     - `OPENAI_API_KEY` as your OpenAI API key
     - `VISION_SERVICE_URL` as the vision service URL (e.g., `http://localhost:8001`)

   - Start the system:
     ```bash
     cd /home/vishnu/ros_2025
     ./intera.sh
     source devel/setup.bash
     roslaunch gpt_vision complete_system.launch
     ```

3. **System Workflow**:
   - The system will:
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
