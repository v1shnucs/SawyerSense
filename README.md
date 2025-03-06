# ROS 2025 Project

## Setup

1. Clone the repository
```bash
git clone <repository-url>
cd ros_2025
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

## Notes

- The `.env` file containing your API key is automatically ignored by git for security
- Make sure to keep your API key confidential and never commit it to version control
- If you receive an error about OPENAI_API_KEY not being set, ensure you've properly set up the .env file