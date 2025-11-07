#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Auto-confirm step execution:
# - Behaves like step mode (plans logical steps with multiple primitive actions)
# - Does NOT ask the user to confirm each step
# - Executes actions immediately and informs the planner when done to get the next step

import rospy
import traceback
import os
import json
from std_msgs.msg import String
import time
import requests

class GPTResponseStepAutoNode:
    def __init__(self):
        rospy.init_node('gpt_response_step_no_confirm_node', anonymous=True)

        self.api_key = os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            rospy.logwarn("OPENAI_API_KEY not set (OK if the FastAPI service handles auth).")

        # Publishers
        self.response_pub = rospy.Publisher('/gpt_response', String, queue_size=10)  # For act_gpt.py
        self.speak_pub = rospy.Publisher('/speak_text', String, queue_size=10)
        # ADD THIS LINE:
        self.photo_trigger_pub = rospy.Publisher('/retake_photo_trigger', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/transcription', String, self.transcription_callback)
        rospy.Subscriber('/grid_state', String, self.grid_state_callback)
        rospy.Subscriber('/is_finished', String, self.primitive_action_done_callback)  # From act_gpt.py

        # State
        self.latest_grid_state = None
        self.latest_transcription = None

        self.current_task_id = None
        self.current_logical_step_primitive_actions = []
        self.current_primitive_action_idx = 0
        self.executing_primitive_actions_sequence = False

        # Collect full plan and ask one confirmation before executing all logical steps
        self.waiting_for_confirmation_for_all_steps = False
        self.planned_logical_steps = []  # list of {'speak': str, 'primitive_actions': [dict]}
        # Keep legacy flag available
        self.waiting_for_final_confirmation = False
        # Optional: simple counter to track how many logical steps were executed in this task
        self.executed_logical_steps_count = 0

        # Service endpoint (use new auto-step service)
        self.gpt_service_url = os.getenv('GPT_SERVICE_URL', 'http://localhost:8000')
        rospy.loginfo("Using GPT service at: %s", self.gpt_service_url)

        # HTTP session
        self.session = requests.Session()
        self.session.headers.update({
            "Content-Type": "application/json",
            "Accept": "application/json",
            "Connection": "keep-alive"
        })

        rospy.loginfo("GPT Response Auto-Step Node initialized. Waiting for grid state and transcription...")
        self.speak_pub.publish("Auto step planner is ready.")

    def grid_state_callback(self, msg):
        self.latest_grid_state = msg.data
        rospy.loginfo("Received grid state: %s", self.latest_grid_state)

        # If we already have a transcription and are idle, start a new task
        if self.latest_transcription and \
           not self.executing_primitive_actions_sequence and \
           not self.current_task_id:
            self._init_new_task_with_service()

    def transcription_callback(self, msg):
        transcription_text = msg.data
        rospy.loginfo("Received transcription: %s", transcription_text)

        # If we're awaiting a ONE-TIME confirmation for executing the entire collected plan, handle it here.
        if self.waiting_for_confirmation_for_all_steps and self.current_task_id:
            confirmation_text = transcription_text.lower()
            positive_keywords = ['yes', 'confirmed', 'ok', 'continue', 'yep', 'yeah', 'proceed', 'affirmative', 'confirm', 'do it', 'sure']
            confirmed = any(keyword in confirmation_text for keyword in positive_keywords)
            rospy.loginfo("User response for full-plan confirmation: '%s' => confirmed=%s for task %s", confirmation_text, confirmed, self.current_task_id)
            # Clear the waiting flag for the full-plan confirmation
            self.waiting_for_confirmation_for_all_steps = False
            if confirmed:
                # Start executing the collected plan
                rospy.loginfo("Starting execution of collected plan with %d logical steps.", len(self.planned_logical_steps))
                self._start_executing_planned_steps()
            else:
                # User rejected the full plan; send feedback to planner and reset
                self._send_logical_step_confirmation_to_service(False, transcription_text)
                self.reset_state()
            return

        if self.executing_primitive_actions_sequence:
            rospy.loginfo("Currently executing actions; ignoring new transcription: %s", transcription_text)
            self.speak_pub.publish("Please wait, I'm executing the current plan.")
            return

        # Treat as a new task request
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
                timeout=(2, 20)
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
                # Collect the full sequence of logical steps from the planner BEFORE executing anything.
                self.planned_logical_steps = []
                # Include the initial step returned by init_task
                if 'speak' in service_response_data or 'primitive_actions' in service_response_data:
                    step = {
                        'speak': service_response_data.get('speak', ''),
                        'primitive_actions': service_response_data.get('primitive_actions', [])
                    }
                    self.planned_logical_steps.append(step)
                # Now iteratively request subsequent logical steps from the planner by telling it the previous step was executed.
                # This asks the planner for the next logical step without executing anything locally.
                try:
                    self._collect_full_plan_from_service()
                    # Present the collected plan to the user and request one confirmation for the entire plan
                    self._present_plan_for_confirmation()
                except Exception as e:
                    rospy.logerr("Error collecting full plan from service: %s", str(e))
                    traceback.print_exc()
                    self.speak_pub.publish("Sorry, I couldn't collect the full plan from the planner.")
                    self.reset_state()
                return
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
        """Handle responses from /init_task and /confirm_logical_step in auto-confirm mode."""
        try:
            speak_text = service_response_data.get("speak")
            # Expecting 'primitive_actions' array
            self.current_logical_step_primitive_actions = service_response_data.get("primitive_actions", [])
            completed = service_response_data.get("completed", False)

            if speak_text:
                self.speak_pub.publish(speak_text)

            if completed:
                rospy.loginfo("Task %s marked as completed by service.", self.current_task_id)
                # Instead of resetting immediately, ask the user to confirm the OVERALL task completion.
                self.waiting_for_final_confirmation = True
                # Inform the user and wait for their confirmation before fully resetting state.
                confirmation_prompt = "I've completed all the steps I can to achieve the task. Please say 'confirm' to accept completion, or provide feedback to retry."
                self.speak_pub.publish(confirmation_prompt)
                rospy.loginfo("Waiting for final user confirmation for task %s", self.current_task_id)
                return

            if self.current_logical_step_primitive_actions:
                # ADD THIS BLOCK
                first_action = self.current_logical_step_primitive_actions[0]
                if (len(self.current_logical_step_primitive_actions) == 1 and
                    isinstance(first_action, dict) and
                    first_action.get("action") == "photo"):

                    rospy.loginfo("Special 'photo' action detected. Triggering retake.")
                    self.photo_trigger_pub.publish("retake_photo")

                    # Reset flags to wait for the new grid state
                    self.executing_primitive_actions_sequence = False
                    self.current_logical_step_primitive_actions = []
                    return # Skip execution logic
                # END OF ADDED BLOCK

                # Auto-confirm: start executing immediately
                self.current_primitive_action_idx = 0
                self.executing_primitive_actions_sequence = True
                # Track that we've started executing another logical step
                self.executed_logical_steps_count += 1
                rospy.loginfo("Auto-executing logical step #%d with %d primitive actions.",
                              self.executed_logical_steps_count, len(self.current_logical_step_primitive_actions))
                self._execute_next_primitive_action()
            else:
                rospy.logwarn("No primitive actions in service response, but task not marked completed. Task ID: %s",
                              self.current_task_id)
                if not speak_text:
                    self.speak_pub.publish("The planner didn't suggest any actions. Please try a different command.")
                self.reset_state()

        except Exception as e:
            rospy.logerr("Error handling service response: %s. Response data: %s", str(e), service_response_data)
            traceback.print_exc()
            self.speak_pub.publish("There was an issue processing the response from the planning service.")
            self.reset_state()

    def _execute_next_primitive_action(self):
        if not self.executing_primitive_actions_sequence:
            return

        # Normal in-step execution
        if self.current_primitive_action_idx < len(self.current_logical_step_primitive_actions):
            action_to_execute = self.current_logical_step_primitive_actions[self.current_primitive_action_idx]
            rospy.loginfo("Executing primitive action %d/%d of logical step %d: %s",
                          self.current_primitive_action_idx + 1,
                          len(self.current_logical_step_primitive_actions),
                          getattr(self, 'exec_logical_idx', 0) + 1,
                          action_to_execute)
            self.publish_primitive_action(action_to_execute)
            # Waits for /is_finished callback to proceed
            return

        # Finished the primitive actions for the current logical step
        rospy.loginfo("Completed logical step %d.", getattr(self, 'exec_logical_idx', 0) + 1)
        # Notify planner that this logical step finished (keeps history consistent)
        try:
            self._send_logical_step_confirmation_to_service(True, "Logical step actions completed by robot.")
        except Exception:
            # _send... will handle logging and resetting if it fails
            pass

        # Move to the next planned logical step if any
        if hasattr(self, 'exec_logical_idx'):
            self.exec_logical_idx += 1
        else:
            self.exec_logical_idx = 1

        if self.exec_logical_idx < len(self.planned_logical_steps):
            # Prepare next logical step for execution
            next_step = self.planned_logical_steps[self.exec_logical_idx]
            self.current_logical_step_primitive_actions = next_step.get('primitive_actions', [])
            self.current_primitive_action_idx = 0
            rospy.loginfo("Starting execution of logical step %d with %d primitive actions.",
                          self.exec_logical_idx + 1, len(self.current_logical_step_primitive_actions))
            # Continue execution
            self._execute_next_primitive_action()
            return
        else:
            # All planned logical steps executed
            rospy.loginfo("All planned logical steps executed for task %s.", self.current_task_id)
            self.executing_primitive_actions_sequence = False
            # Cleanup and reset
            self.planned_logical_steps = []
            self.exec_logical_idx = 0
            self.current_logical_step_primitive_actions = []
            self.current_primitive_action_idx = 0
            self.speak_pub.publish("All requested steps are complete.")
            self.reset_state()
            return

    def primitive_action_done_callback(self, msg):
        if self.executing_primitive_actions_sequence and msg.data == "done":
            rospy.loginfo("Primitive action %d confirmed done by act_gpt.", self.current_primitive_action_idx + 1)
            self.current_primitive_action_idx += 1
            # Optional small delay before next action for stability
            # rospy.sleep(0.2)
            self._execute_next_primitive_action()
        elif self.executing_primitive_actions_sequence and msg.data != "done":
            rospy.logwarn("Received unexpected message on /is_finished: %s while executing sequence. Waiting for 'done'.",
                          msg.data)

    def _send_logical_step_confirmation_to_service(self, confirmed_status, feedback_text):
        if not self.current_task_id:
            rospy.logerr("Cannot send logical step confirmation: current_task_id is None.")
            return

        rospy.loginfo("Sending logical step completion to service. Task: %s, Confirmed: %s, Feedback: %s",
                      self.current_task_id, confirmed_status, feedback_text)
        try:
            response = self.session.post(
                "{}/confirm_logical_step".format(self.gpt_service_url),
                json={
                    "task_id": self.current_task_id,
                    "confirmed": confirmed_status,
                    "feedback": feedback_text
                },
                timeout=(2, 20)
            )

            if response.status_code == 200:
                service_response_data = response.json()
                # Planner will respond with the next logical step or completion status
                self._handle_service_response(service_response_data)
            else:
                rospy.logerr("Failed to send logical step confirmation to service: %s - %s",
                             response.status_code, response.text)
                self.speak_pub.publish("Error communicating with the planner after that step.")
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

    def _collect_full_plan_from_service(self):
        """Iteratively request next logical steps from the planner by telling it the previous step was 'executed'.
        This collects the full sequence of logical steps (planner-side) so we can present them to the user once."""
        if not self.current_task_id:
            raise RuntimeError("No current_task_id for plan collection.")
        rospy.loginfo("Collecting full plan from planner for task %s...", self.current_task_id)
        try:
            while True:
                response = self.session.post(
                    "{}/confirm_logical_step".format(self.gpt_service_url),
                    json={
                        "task_id": self.current_task_id,
                        "confirmed": True,
                        "feedback": "collecting_plan"
                    },
                    timeout=(2, 20)
                )
                if response.status_code != 200:
                    rospy.logerr("Failed to collect next logical step: %s - %s", response.status_code, response.text)
                    raise RuntimeError("Planner collection failed")
                data = response.json()
                # Append the returned logical step (may be completion marker)
                step = {
                    'speak': data.get('speak', ''),
                    'primitive_actions': data.get('primitive_actions', [])
                }
                self.planned_logical_steps.append(step)
                rospy.loginfo("Collected step %d: %s (actions: %d)", len(self.planned_logical_steps),
                              step['speak'], len(step['primitive_actions']))
                if data.get('completed', False):
                    rospy.loginfo("Planner reports overall task completed during collection.")
                    break
                # Continue looping to collect next logical step
            return
        except Exception as e:
            rospy.logerr("Error during full plan collection: %s", str(e))
            raise

    def _present_plan_for_confirmation(self):
        """Summarize the collected plan to the user and ask for a single confirmation to execute all steps."""
        if not self.planned_logical_steps:
            rospy.logwarn("No planned logical steps to present.")
            self.speak_pub.publish("I couldn't produce a plan for that request.")
            self.reset_state()
            return
        # Build a very short verbal summary (concise: actions only) to speed up confirmation
        summary_lines = []
        for idx, step in enumerate(self.planned_logical_steps):
            actions = step.get('primitive_actions', [])
            if actions:
                # Compact description like: "grab6,place3"
                actions_desc = ",".join(["{}{}".format(a.get('action'), a.get('space')) for a in actions])
                summary_lines.append("Step {}: {}".format(idx + 1, actions_desc))
            else:
                # Fall back to short speak or "no actions"
                short_speak = step.get('speak', '').split('.')[0] if step.get('speak') else "no actions"
                summary_lines.append("Step {}: {}".format(idx + 1, short_speak))
        # Publish each short step summary with minimal delay, then ask for one confirmation
        for line in summary_lines:
            self.speak_pub.publish(line)
            rospy.sleep(0.1)
        confirm_prompt = "Short plan: {} steps. Say 'yes' to run all, or 'no' to cancel.".format(len(self.planned_logical_steps))
        self.speak_pub.publish(confirm_prompt)
        self.waiting_for_confirmation_for_all_steps = True

    def _start_executing_planned_steps(self):
        """Begin execution of the collected planned_logical_steps sequentially."""
        if not self.planned_logical_steps:
            rospy.logwarn("No planned logical steps to execute.")
            self.reset_state()
            return
        # Initialize execution indices
        self.exec_logical_idx = 0
        first_step = self.planned_logical_steps[0]
        self.current_logical_step_primitive_actions = first_step.get('primitive_actions', [])
        self.current_primitive_action_idx = 0
        self.executing_primitive_actions_sequence = True
        # Track that we've started executing logical steps
        self.executed_logical_steps_count += 1
        rospy.loginfo("Beginning execution of planned sequence with %d logical steps.", len(self.planned_logical_steps))
        # Start execution
        self._execute_next_primitive_action()

    def publish_primitive_action(self, primitive_action_dict):
        """Publishes a single primitive action to /gpt_response for act_gpt.py"""
        if not isinstance(primitive_action_dict, dict) or \
           "action" not in primitive_action_dict or \
           "space" not in primitive_action_dict:
            rospy.logerr("Invalid primitive_action_dict for publishing: %s", primitive_action_dict)
            return

        try:
            action_json = json.dumps(primitive_action_dict)  # {"action": ..., "space": ...}
            self.response_pub.publish(action_json)
            rospy.loginfo("Published primitive action to /gpt_response: %s", action_json)
        except Exception as e:
            rospy.logerr("Error publishing primitive action: %s", str(e))
            traceback.print_exc()

    def reset_state(self):
        rospy.loginfo("Resetting node state.")
        self.current_task_id = None
        self.latest_transcription = None
        self.current_logical_step_primitive_actions = []
        self.current_primitive_action_idx = 0
        self.executing_primitive_actions_sequence = False
        # Keep latest_grid_state for follow-ups

if __name__ == '__main__':
    try:
        node = GPTResponseStepAutoNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPTResponseStepAutoNode shutting down.")
    finally:
        try:
            if 'node' in locals() and hasattr(node, 'session') and node.session:
                node.session.close()
                rospy.loginfo("HTTP session closed.")
        except Exception:
            pass