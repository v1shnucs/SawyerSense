# ROS 2025 Project

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

4. Run the nodes
```bash
# In separate terminals:
rosrun robot_action act_gpt.py
rosrun gpt_vision get_gpt_response.py
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