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
        # GPT service configuration
        self.gpt_service_url = os.getenv('GPT_SERVICE_URL', 'http://localhost:8000')
        rospy.loginfo("Using GPT service at: %s", self.gpt_service_url)
        
        # Initialize persistent HTTP session
        self.session = requests.Session()
        self.session.headers.update({
            "Content-Type": "application/json",
            "Accept": "application/json",
            "Connection": "keep-alive"
        })
        
        rospy.loginfo("GPT Response Node initialized")
    
    def grid_state_callback(self, msg):
        """Store the latest grid state and process any pending transcription"""
        self.latest_grid_state = msg.data
        rospy.loginfo("Received grid state: %s", self.latest_grid_state)
        
        # Process any pending transcription
        if self.latest_transcription:
            self._process_inputs()

    def transcription_callback(self, msg):
        """Handle new transcription and process if grid state is available"""
        self.latest_transcription = msg.data
        rospy.loginfo("Received transcription: %s", self.latest_transcription)
        
        if self.latest_grid_state:
            self._process_inputs()
        else:
            rospy.logwarn("Received transcription but no grid state available yet")
    
    def _process_inputs(self):
        """Helper to combine and process inputs"""
        combined_input = "Transcription: {}\nGrid State: {}".format(
            self.latest_transcription, self.latest_grid_state)
        self.process_input(combined_input)

    def get_gpt_response(self, user_input):
        try:
            # Extract transcription and grid state from user input
            parts = user_input.split("\nGrid State: ")
            if len(parts) != 2:
                raise ValueError("Invalid input format")
            
            transcription = parts[0].replace("Transcription: ", "").strip()
            grid_state = parts[1].strip()

            # Prepare request data
            data = {
                "transcription": transcription,
                "grid_state": grid_state
            }

            # Make request to FastAPI service using persistent session with retries
            max_retries = 2
            timeout = 8  # Increased timeout for API response
            
            for attempt in range(max_retries + 1):
                try:
                    response = self.session.post(
                        "{}/gpt".format(self.gpt_service_url),
                        json=data,
                        timeout=(2, timeout)  # (connect timeout, read timeout)
                    )
                    break
                except requests.exceptions.Timeout:
                    if attempt < max_retries:
                        rospy.logwarn("Request timed out, retrying... (attempt %d/%d)",
                                    attempt + 1, max_retries)
                        continue
                    raise

            if response.status_code != 200:
                rospy.logerr("Service request failed: %s", response.text)
                return None

            # Get the response JSON
            return response.json()

        except requests.exceptions.RequestException as e:
            rospy.logerr("Error communicating with GPT service: %s", str(e))
            return None
        except Exception as e:
            rospy.logerr("Error processing response: %s", str(e))
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
    finally:
        # Clean up session on exit
        if hasattr(node, 'session'):
            node.session.close()