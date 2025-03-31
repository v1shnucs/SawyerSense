#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#This file is based on gpt_imagecall.py
#The input will be reading from 2 topics: transcription and grid state
# program will append grid state to the transcription and append that to the conversation then publish result to gpt_response

import rospy
import traceback
import os
import json
from std_msgs.msg import String
import time
import requests

class GPTResponseNode:
    def __init__(self):
        rospy.init_node('gpt_response_node', anonymous=True)
        
        # Initialize OpenAI API key from environment variable
        self.api_key = os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            rospy.logerr("OPENAI_API_KEY environment variable not set!")
            raise ValueError("OPENAI_API_KEY environment variable is required")
        
        # Initialize publishers
        self.response_pub = rospy.Publisher('/gpt_response', String, queue_size=10)
        self.speak_pub = rospy.Publisher('/speak_text', String, queue_size=10)
        
        # Initialize subscribers
        rospy.Subscriber('/transcription', String, self.transcription_callback)
        rospy.Subscriber('/grid_state', String, self.grid_state_callback)
        
        # Store latest grid state
        self.latest_grid_state = None
        # Store latest transcription
        self.latest_transcription = None
        
        # System prompt
        self.system_prompt = """You are a helpful robot assistant that controls a robot arm. 
When given instructions about moving objects between spaces, along with the current grid state, you should:
1. Analyze the current state and the requested move
2. Provide a brief explanation of what you'll do
3. Give a sequence of grab and place actions

Your response must be in JSON format with two fields:
- "speak": A brief explanation of what you're going to do
- "actions": An array of actions, where each action has:
  - "action": either "grab" or "place"
  - "square": an integer from 1-12

Example response:
{
    "speak": "I'll move the red circle from space 5 to space 9",
    "actions": [
        {"action": "grab", "square": 5},
        {"action": "place", "square": 9}
    ]
}"""
        
        rospy.loginfo("GPT Response Node initialized")
    
    def grid_state_callback(self, msg):
        """Store the latest grid state"""
        self.latest_grid_state = msg.data
        rospy.loginfo("Received grid state: %s", self.latest_grid_state)

    def transcription_callback(self, msg):
        """Handle new transcription by combining it with grid state"""
        self.latest_transcription = msg.data
        rospy.loginfo("Received transcription: %s", self.latest_transcription)
        
        # Only proceed if we have both transcription and grid state
        if self.latest_grid_state:
            # Combine transcription with grid state (Python 2 compatible string formatting)
            combined_input = "Transcription: {}\nGrid State: {}".format(
                self.latest_transcription, self.latest_grid_state)
            self.process_input(combined_input)
        else:
            rospy.logwarn("Received transcription but no grid state available yet")

    def get_gpt_response(self, user_input):
        try:
            # Create the prompt
            prompt = self.system_prompt + "\n\nInput:\n" + user_input + "\n\nResponse:"
            
            # Prepare the API request
            headers = {
                "Content-Type": "application/json",
                "Authorization": "Bearer {}".format(self.api_key)
            }
            
            data = {
                "model": "gpt-3.5-turbo-instruct",
                "prompt": prompt,
                "max_tokens": 150,
                "temperature": 0.3,
                "top_p": 1,
                "frequency_penalty": 0,
                "presence_penalty": 0,
                "stop": ["\n\n"]
            }
            
            # Make the API request
            response = requests.post(
                "https://api.openai.com/v1/completions",
                headers=headers,
                json=data
            )
            
            if response.status_code != 200:
                rospy.logerr("API request failed: %s", response.text)
                return None
            
            # Get the response text
            response_data = response.json()
            response_text = response_data["choices"][0]["text"].strip()
            rospy.loginfo("Raw GPT response: %s", response_text)
            
            # Parse the JSON response
            try:
                parsed_response = json.loads(response_text)
                # Validate response format
                if not isinstance(parsed_response, dict):
                    raise ValueError("Response must be a dictionary")
                if "speak" not in parsed_response or "actions" not in parsed_response:
                    raise ValueError("Response must contain 'speak' and 'actions' fields")
                if not isinstance(parsed_response["actions"], list):
                    raise ValueError("'actions' must be an array")
                
                return parsed_response
                
            except json.JSONDecodeError:
                rospy.logerr("Failed to parse GPT response as JSON: %s", response_text)
                return None
                
        except Exception as e:
            rospy.logerr("Error getting GPT response: %s", str(e))
            return None

    def publish_actions(self, actions):
        """Publish each action with a small delay between them"""
        for action in actions:
            # Create JSON string for the action
            action_json = json.dumps(action)
            
            # Publish the action
            self.response_pub.publish(action_json)
            rospy.loginfo("Published action: %s", action_json)
            
            # Wait for a short time between actions
            time.sleep(0.5)

    def process_input(self, combined_input):
        """Process the combined transcription and grid state"""
        try:
            rospy.loginfo("Processing combined input: %s", combined_input)
            
            # Get GPT response
            response = self.get_gpt_response(combined_input)
            
            if response:
                # Publish the speak text
                if "speak" in response:
                    self.speak_pub.publish(response["speak"])
                    rospy.loginfo("Published speak text: %s", response["speak"])
                
                # Publish the actions
                if "actions" in response:
                    self.publish_actions(response["actions"])
            
        except Exception as e:
            rospy.logerr("Processing error: %s", str(e))
            traceback.print_exc()

if __name__ == '__main__':
    try:
        node = GPTResponseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass