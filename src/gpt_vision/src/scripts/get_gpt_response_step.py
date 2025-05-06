#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# This file provides step-by-step action execution with user confirmation
# It now handles "logical steps" which can contain multiple primitive actions.

import rospy
import traceback
import os
import json
from std_msgs.msg import String
import time
import requests

class GPTResponseStepNode:
    def __init__(self):
        rospy.init_node('gpt_response_step_node', anonymous=True)
        
        self.api_key = os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            rospy.logwarn("OPENAI_API_KEY environment variable not set! Service calls might fail if not configured on the service side.")
        
        self.response_pub = rospy.Publisher('/gpt_response', String, queue_size=10) # For act_gpt.py
        self.speak_pub = rospy.Publisher('/speak_text', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/transcription', String, self.transcription_callback)
        rospy.Subscriber('/grid_state', String, self.grid_state_callback)
        rospy.Subscriber('/is_finished', String, self.primitive_action_done_callback) # From act_gpt.py
        
        self.latest_grid_state = None
        self.latest_transcription = None
        
        self.current_task_id = None
        self.waiting_for_confirmation_for_logical_step = False # True when waiting for user to confirm a logical step

        # For handling sequence of primitive actions within a confirmed logical step
        self.current_logical_step_primitive_actions = [] # List of primitive actions for the current logical step
        self.current_primitive_action_idx = 0
        self.executing_primitive_actions_sequence = False # True when iterating through primitive actions

        self.gpt_service_url = os.getenv('GPT_SERVICE_URL', 'http://localhost:8000')
        rospy.loginfo("Using GPT service at: %s", self.gpt_service_url)
        
        self.session = requests.Session()
        self.session.headers.update({
            "Content-Type": "application/json",
            "Accept": "application/json",
            "Connection": "keep-alive"
        })
        
        rospy.loginfo("GPT Response Step Node initialized. Waiting for grid state and transcription...")
        self.speak_pub.publish("Step by step planner is ready.")

    def grid_state_callback(self, msg):
        self.latest_grid_state = msg.data
        rospy.loginfo("Received grid state: %s", self.latest_grid_state)
        
        if self.latest_transcription and \
           not self.waiting_for_confirmation_for_logical_step and \
           not self.executing_primitive_actions_sequence and \
           not self.current_task_id:
            self._init_new_task_with_service()

    def transcription_callback(self, msg):
        transcription_text = msg.data
        rospy.loginfo("Received transcription: %s", transcription_text)
        
        if self.waiting_for_confirmation_for_logical_step and self.current_task_id:
            self.handle_logical_step_confirmation(transcription_text)
        elif self.executing_primitive_actions_sequence:
            rospy.loginfo("Currently executing a sequence of actions, ignoring new transcription: %s", transcription_text)
            self.speak_pub.publish("Please wait, I'm still working on the current step.")
        else: # New command
            self.latest_transcription = transcription_text
            if self.latest_grid_state and not self.current_task_id:
                self._init_new_task_with_service()
            elif not self.latest_grid_state:
                 rospy.logwarn("Received transcription but no grid state available yet for new task.")

    def _init_new_task_with_service(self):
        if not self.latest_transcription or not self.latest_grid_state:
            rospy.logwarn("Cannot initialize task: Missing transcription or grid state.")
            return

        rospy.loginfo("Initializing new task with transcription: '%s'", self.latest_transcription)
        try:
            response = self.session.post(
                "{}/init_task".format(self.gpt_service_url),
                json={
                    "transcription": self.latest_transcription,
                    "grid_state": self.latest_grid_state
                },
                timeout=(2, 20) # Increased timeout for potentially complex first plan
            )
            
            if response.status_code == 200:
                service_response_data = response.json()
                self.current_task_id = service_response_data.get("task_id")
                if not self.current_task_id:
                    rospy.logerr("Service did not return a task_id during init.")
                    self.speak_pub.publish("Sorry, I couldn't start the task properly with the planner.")
                    self.reset_state()
                    return
                rospy.loginfo("New task initialized with ID: %s", self.current_task_id)
                self._handle_service_response(service_response_data)
            else:
                rospy.logerr("Failed to initialize task with service: %s - %s", response.status_code, response.text)
                self.speak_pub.publish("Sorry, I couldn't initialize the task with the planning service.")
                self.reset_state()
                
        except requests.exceptions.RequestException as e:
            rospy.logerr("Error initializing task (request failed): %s", str(e))
            self.speak_pub.publish("I couldn't reach the planning service to start the task.")
            self.reset_state()
        except Exception as e:
            rospy.logerr("Error in _init_new_task_with_service: %s", str(e))
            traceback.print_exc()
            self.speak_pub.publish("An unexpected error occurred while starting the task.")
            self.reset_state()

    def _handle_service_response(self, service_response_data):
        """ Handles responses from /init_task and /confirm_logical_step """
        try:
            speak_text = service_response_data.get("speak")
            # Expecting 'primitive_actions' from the service now
            self.current_logical_step_primitive_actions = service_response_data.get("primitive_actions", [])
            completed = service_response_data.get("completed", False)

            if speak_text:
                self.speak_pub.publish(speak_text)
            
            if completed:
                rospy.loginfo("Task %s marked as completed by service.", self.current_task_id)
                self.reset_state()
                return
            
            if self.current_logical_step_primitive_actions:
                # We have a new logical step (with one or more primitive actions) to propose
                self.current_primitive_action_idx = 0 # Reset for the new set of actions
                self.executing_primitive_actions_sequence = False # Not executing yet, just proposing
                self.waiting_for_confirmation_for_logical_step = True
                rospy.loginfo("Proposing logical step: '%s' with %d primitive actions. Waiting for user confirmation.", 
                              speak_text, len(self.current_logical_step_primitive_actions))
            else: 
                rospy.logwarn("No primitive actions in service response, but task not marked completed. Task ID: %s", self.current_task_id)
                if not speak_text: # If service didn't provide specific message
                    self.speak_pub.publish("The planner didn't suggest any actions. Please try a different command.")
                self.reset_state() # Or ask user for clarification

        except Exception as e:
            rospy.logerr("Error handling service response: %s. Response data: %s", str(e), service_response_data)
            traceback.print_exc()
            self.speak_pub.publish("There was an issue processing the response from the planning service.")
            self.reset_state()

    def handle_logical_step_confirmation(self, user_speech_text):
        if not self.current_task_id:
            rospy.logwarn("Received confirmation but no active task ID.")
            return

        confirmation_text = user_speech_text.lower()
        # More flexible confirmation checking
        positive_keywords = ['yes', 'confirmed', 'ok', 'continue', 'yep', 'yeah', 'proceed', 'affirmative', 'do it', 'sure', 'alright']
        confirmed = any(keyword in confirmation_text for keyword in positive_keywords)
        
        rospy.loginfo("Handling logical step confirmation: '%s', Confirmed: %s for task %s",
                      confirmation_text, confirmed, self.current_task_id)
        
        self.waiting_for_confirmation_for_logical_step = False # No longer waiting for this specific confirmation

        if confirmed:
            if self.current_logical_step_primitive_actions:
                rospy.loginfo("Logical step confirmed. Starting execution of %d primitive actions.", 
                              len(self.current_logical_step_primitive_actions))
                self.executing_primitive_actions_sequence = True
                self.current_primitive_action_idx = 0
                self._execute_next_primitive_action() # Start the sequence
            else:
                rospy.logwarn("Logical step confirmed, but no primitive actions to execute. This might be an error.")
                # This case implies the service should have marked completed or provided actions.
                # We'll call confirm_logical_step to let the service know and potentially get a new state.
                self._send_logical_step_confirmation_to_service(True, None)
        else: # Logical step rejected by user
            rospy.loginfo("Logical step rejected by user. Sending feedback to service.")
            self.current_logical_step_primitive_actions = [] # Clear actions as they were rejected
            self._send_logical_step_confirmation_to_service(False, user_speech_text) # Send feedback

    def _execute_next_primitive_action(self):
        if not self.executing_primitive_actions_sequence:
            return

        if self.current_primitive_action_idx < len(self.current_logical_step_primitive_actions):
            action_to_execute = self.current_logical_step_primitive_actions[self.current_primitive_action_idx]
            rospy.loginfo("Executing primitive action %d/%d: %s", 
                          self.current_primitive_action_idx + 1, 
                          len(self.current_logical_step_primitive_actions), 
                          action_to_execute)
            self.publish_primitive_action(action_to_execute)
            # Now we wait for primitive_action_done_callback
        else:
            # All primitive actions in the current logical step are done
            rospy.loginfo("All primitive actions for the current logical step executed.")
            self.executing_primitive_actions_sequence = False
            # Inform the service that the confirmed logical step is completed.
            self._send_logical_step_confirmation_to_service(True, "Logical step actions completed by robot.")


    def primitive_action_done_callback(self, msg):
        if self.executing_primitive_actions_sequence and msg.data == "done":
            rospy.loginfo("Primitive action %d confirmed done by act_gpt.", self.current_primitive_action_idx + 1)
            self.current_primitive_action_idx += 1
            # Optional: Add a small delay before next action if needed for robot stability
            # rospy.sleep(0.5)
            self._execute_next_primitive_action() # Execute next or finalize logical step
        elif self.executing_primitive_actions_sequence and msg.data != "done":
            rospy.logwarn("Received unexpected message on /is_finished: %s while executing sequence. Waiting for 'done'.", msg.data)


    def _send_logical_step_confirmation_to_service(self, confirmed_status, feedback_text): # Type hints removed for Python 2
        if not self.current_task_id:
            rospy.logerr("Cannot send logical step confirmation: current_task_id is None.")
            return
        
        rospy.loginfo("Sending logical step confirmation to service. Task: %s, Confirmed: %s, Feedback: %s",
                      self.current_task_id, confirmed_status, feedback_text)
        try:
            response = self.session.post(
                "{}/confirm_logical_step".format(self.gpt_service_url), # Updated endpoint
                json={
                    "task_id": self.current_task_id,
                    "confirmed": confirmed_status,
                    "feedback": feedback_text
                },
                timeout=(2, 20) 
            )
            
            if response.status_code == 200:
                service_response_data = response.json()
                # Service will respond with the next logical step or completion status
                self._handle_service_response(service_response_data) 
            else:
                rospy.logerr("Failed to send logical step confirmation to service: %s - %s", response.status_code, response.text)
                self.speak_pub.publish("Sorry, there was an error communicating with the planner after that step.")
                self.reset_state()
                
        except requests.exceptions.RequestException as e:
            rospy.logerr("Error sending logical step confirmation (request failed): %s", str(e))
            self.speak_pub.publish("I couldn't reach the planning service to confirm the step.")
            self.reset_state()
        except Exception as e:
            rospy.logerr("Error in _send_logical_step_confirmation_to_service: %s", str(e))
            traceback.print_exc()
            self.speak_pub.publish("An unexpected error occurred while confirming the step with the planner.")
            self.reset_state()

    def publish_primitive_action(self, primitive_action_dict):
        """ Publishes a single primitive action to /gpt_response for act_gpt.py """
        if not isinstance(primitive_action_dict, dict) or \
           "action" not in primitive_action_dict or \
           "space" not in primitive_action_dict:
            rospy.logerr("Invalid primitive_action_dict for publishing: %s", primitive_action_dict)
            return

        try:
            action_json = json.dumps(primitive_action_dict) # Already in {"action": ..., "space": ...} format
            self.response_pub.publish(action_json)
            rospy.loginfo("Published primitive action to /gpt_response: %s", action_json)
        except Exception as e:
            rospy.logerr("Error publishing primitive action: %s", str(e))
            traceback.print_exc()
    
    def reset_state(self):
        rospy.loginfo("Resetting node state.")
        self.current_task_id = None
        self.waiting_for_confirmation_for_logical_step = False
        self.latest_transcription = None 
        self.current_logical_step_primitive_actions = []
        self.current_primitive_action_idx = 0
        self.executing_primitive_actions_sequence = False
        # Do not clear latest_grid_state, it might still be valid for a quick follow-up

if __name__ == '__main__':
    try:
        node = GPTResponseStepNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPTResponseStepNode shutting down.")
    finally:
        if 'node' in locals() and hasattr(node, 'session') and node.session:
            node.session.close()
            rospy.loginfo("HTTP session closed.")