#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#This file is based on gpt_imagecall.py
#The input will be reading from 2 topics: transcription and grid state
# program will append grid state to the transcription and append that to the conversation then publish result to gpt_response
#  
import rospy
import openai
import traceback
import os
from std_msgs.msg import String

class GPTResponseNode:
    def __init__(self):
        rospy.init_node('gpt_response_node', anonymous=True)
        
        # Initialize OpenAI API key from environment variable
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            rospy.logerr("OPENAI_API_KEY environment variable not set!")
            raise ValueError("OPENAI_API_KEY environment variable is required")
        openai.api_key = api_key
        
        # Initialize publishers
        self.response_pub = rospy.Publisher('/gpt_response', String, queue_size=10)
        
        # Initialize subscribers
        rospy.Subscriber('/user_input', String, self.user_input_callback)
        
        rospy.loginfo("GPT Response Node initialized")

    def clean_response(self, text):
        """Clean the response to ensure it's ASCII-safe and concise"""
        try:
            # Convert to ASCII, ignoring non-ASCII characters
            cleaned = text.encode('ascii', 'ignore').decode('ascii').strip()
            # Take only the first sentence if it's too long
            if len(cleaned) > 100:
                sentences = cleaned.split('.')
                return sentences[0] + '.'
            return cleaned
        except:
            return "Error cleaning response"

    def get_gpt_response(self, user_input):
        try:
            # Create a chat-like prompt format
            chat_prompt = (
                "You are a helpful robot assistant. Keep responses very brief and direct.\n\n"
                "Human: {}\n"
                "Assistant: "
            ).format(user_input)
            
            response = openai.Completion.create(
                engine="gpt-3.5-turbo-instruct",
                prompt=chat_prompt,
                max_tokens=25,  # Very short responses
                temperature=0.3,  # More focused
                stop=["\n", "Human:"],  # Stop at natural breakpoints
                top_p=1,
                frequency_penalty=0,
                presence_penalty=0
            )
            
            # Clean and return the response
            return self.clean_response(response['choices'][0]['text'].strip())
        except Exception as e:
            error_msg = str(e).encode('ascii', 'ignore').decode('ascii')
            rospy.logerr("Error getting GPT response: %s", error_msg)
            return "I apologize, but I'm having trouble processing your request."

    def user_input_callback(self, msg):
        try:
            user_input = msg.data
            rospy.loginfo("Received user input: %s", user_input)
            
            # Get GPT response
            response = self.get_gpt_response(user_input)
            
            # Publish response
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)
            rospy.loginfo("Published response: %s", response)
        except Exception as e:
            rospy.logerr("Callback error: %s", str(e))

if __name__ == '__main__':
    try:
        node = GPTResponseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass