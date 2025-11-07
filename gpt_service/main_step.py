#!/usr/bin/env python3
import json
import os
import time
import re
from typing import List, Optional, Dict
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from openai import OpenAI
from dotenv import load_dotenv
import uuid
import traceback # For more detailed error logging

# Load environment variables
load_dotenv()

app = FastAPI(title="GPT Step Service")
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY"),
    timeout=20.0 # Increased timeout for potentially longer LLM responses
)

# Load system prompt
def load_system_prompt():
    try:
        prompt_file_path = os.path.join(os.path.dirname(__file__), "system_prompt_step.json")
        if not os.path.exists(prompt_file_path):
            print(f"Error: System prompt file not found at {prompt_file_path}")
            return None
        with open(prompt_file_path, "r") as f:
            prompt_data = json.load(f)
            return prompt_data[0]["content"]
    except Exception as e:
        print(f"Error loading system prompt from {prompt_file_path}: {e}")
        return None

system_prompt = load_system_prompt()
if not system_prompt:
    print("CRITICAL: Failed to load system prompt. GPT functionality will be impaired.")

def attempt_json_repair(text: str):
    """Attempt to repair common JSON formatting issues (copied from main.py)"""
    if not text:
        return None
    print(f"Attempting to repair JSON: {text[:100]}...")
    try:
        return json.loads(text)
    except:
        pass
    try:
        pattern = r'"([^"\\]*(\\.[^"\\]*)*)'
        unterminated = re.finditer(pattern + r'(?!")', text)
        fixed_text = text
        for match in unterminated:
            end_pos = match.end()
            fixed_text = fixed_text[:end_pos] + '"' + fixed_text[end_pos:]
        try:
            return json.loads(fixed_text)
        except:
            pass
    except Exception as e:
        print(f"Error in string repair: {e}")
        if text.count('{') > text.count('}'):
            text += '}' * (text.count('{') - text.count('}'))
        if text.count('[') > text.count(']'):
            text += ']' * (text.count('[') - text.count(']'))
        try:
            return json.loads(text)
        except Exception as e:
            print(f"Final repair attempt failed: {e}")
    try:
        open_braces = text.count('{')
        close_braces = text.count('}')
        open_brackets = text.count('[')
        close_brackets = text.count(']')
        fixed_text = text
        if open_braces > close_braces:
            fixed_text += '}' * (open_braces - close_braces)
        if open_brackets > close_brackets:
            fixed_text += ']' * (open_brackets - close_brackets)
        try:
            return json.loads(fixed_text)
        except:
            pass
    except Exception as e:
        print(f"Error in brace balancing: {e}")
    return None

# Models
class PrimitiveAction(BaseModel):
    action: str 
    space: int

class TaskState:
    def __init__(self):
        self.current_logical_step_speak: str = ""
        self.current_primitive_actions: List[PrimitiveAction] = []
        self.grid_state_at_request: str = ""  # Initial grid state for the overall task
        self.original_transcription: str = ""  # User's overall task goal
        self.history: List[Dict] = []

class TaskManager:
    def __init__(self):
        self.tasks: Dict[str, TaskState] = {}

    def create_task(self) -> str:
        task_id = str(uuid.uuid4())
        self.tasks[task_id] = TaskState()
        return task_id

    def get_task(self, task_id: str) -> Optional[TaskState]:
        return self.tasks.get(task_id)

task_manager = TaskManager()

class InitialRequest(BaseModel):
    transcription: str
    grid_state: str

class LogicalStepConfirmation(BaseModel):
    task_id: str
    confirmed: bool
    feedback: Optional[str] = None 

class TaskResponse(BaseModel):
    task_id: str
    speak: str 
    primitive_actions: List[PrimitiveAction] = [] 
    completed: bool # True if the OVERALL task goal is complete

async def get_llm_response(user_input_for_llm: str, task_id: Optional[str] = None) -> Dict:
    if not system_prompt:
        print(f"Task {task_id or 'N/A'}: Cannot call LLM, system_prompt is not loaded.")
        raise HTTPException(status_code=500, detail="System prompt not loaded, cannot process request.")
    try:
        print(f"Task {task_id or 'N/A'}: Sending to LLM (first 200 chars): {user_input_for_llm[:200]}...")
        llm_api_response = client.responses.create(
            model="gpt-5",  # Switched to gpt-5
            input=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "input_text",
                            "text": f"System Prompt: {system_prompt}\n\nUser Task: {user_input_for_llm}"
                        }
                    ]
                }
            ],
            reasoning={ "effort": "low" },
            # temperature removed: not supported by responses.create
            # max_tokens=500,
            # response_format removed; rely on system prompt to output JSON
        )
        # The new API returns a complex Response object. We must parse it.
        response_text = None
        if hasattr(llm_api_response, 'output') and isinstance(llm_api_response.output, list):
            for item in llm_api_response.output:
                if item.type == 'message' and hasattr(item, 'content') and isinstance(item.content, list):
                    for content_item in item.content:
                        if content_item.type == 'output_text' and hasattr(content_item, 'text'):
                            response_text = content_item.text.strip()
                            break
                    if response_text:
                        break

        if not response_text:
            raise ValueError(f"Unexpected response format. Could not find 'output_text': {llm_api_response}")

        print(f"Task {task_id or 'N/A'}: Raw LLM response (first 200 chars): {response_text[:200]}...")
        
        # Try parsing JSON, attempt repair if necessary
        try:
            parsed_llm_json = json.loads(response_text)
        except json.JSONDecodeError as e:
            print(f"Task {task_id or 'N/A'}: JSON parsing error: {e}. Attempting repair.")
            parsed_llm_json = attempt_json_repair(response_text)
            if not parsed_llm_json:
                raise ValueError(f"Failed to parse LLM response after repair attempts: {str(e)}")
            print(f"Task {task_id or 'N/A'}: Successfully repaired malformed JSON.")
        
        if not isinstance(parsed_llm_json, dict) or \
           "speak" not in parsed_llm_json or \
           "actions" not in parsed_llm_json:
            print(f"Task {task_id or 'N/A'}: LLM response missing 'speak' or 'actions' keys. Response: {parsed_llm_json}")
            raise ValueError("LLM response must contain 'speak' and 'actions' fields.")
        
        if not isinstance(parsed_llm_json["actions"], list):
            print(f"Task {task_id or 'N/A'}: LLM 'actions' field is not a list. Response: {parsed_llm_json}")
            raise ValueError("'actions' must be a list of primitive action objects.")
 
        for prim_action in parsed_llm_json["actions"]:
            if not isinstance(prim_action, dict) or \
               "action" not in prim_action or \
               "space" not in prim_action:
                print(f"Task {task_id or 'N/A'}: Invalid primitive action format. Action: {prim_action}, Response: {parsed_llm_json}")
                raise ValueError("Each primitive action must have 'action' and 'space' keys.")
            if not isinstance(prim_action["space"], int) or not (1 <= prim_action["space"] <= 16):
                 print(f"Task {task_id or 'N/A'}: Warning - Invalid space number {prim_action['space']} in primitive action. Action: {prim_action}. Allowing for now.")
                 # Consider if this should be a hard error or if LLM is expected to adhere to 1-12 based on prompt.
        return parsed_llm_json

    except json.JSONDecodeError as e_json:
        print(f"Task {task_id or 'N/A'}: Failed to parse LLM JSON response: {e_json}. Response text: {response_text}")
        raise ValueError(f"Failed to parse LLM JSON response: {response_text}")
    except Exception as e_llm:
        print(f"Task {task_id or 'N/A'}: Error communicating with or processing LLM response: {e_llm}")
        raise HTTPException(status_code=500, detail=f"Error with LLM: {str(e_llm)}")

def build_history_summary_for_llm(history: List[Dict]) -> str:
    summary = "\nRelevant History of Interaction (oldest to newest):\n"
    if not history:
        summary += "No prior interactions in this task yet.\n"
    for entry_idx, entry in enumerate(history):
        summary += f"--- Interaction {entry_idx + 1} ---\n"
        if entry["type"] == "llm_proposal":
            summary += f"You proposed: '{entry['speak']}' with actions: {entry['actions_proposed']}\n"
        elif entry["type"] == "user_confirmation":
            status = "CONFIRMED and EXECUTED" if entry["confirmed"] else "REJECTED"
            fb = f" User Feedback: '{entry['feedback_provided']}'" if entry.get("feedback_provided") else ""
            summary += f"User {status} your proposal ('{entry.get('confirmed_speak', 'N/A')}').{fb}\n"
            if entry["confirmed"] and entry.get("confirmed_actions"):
                 summary += f"  Executed actions: {entry['confirmed_actions']}\n"
    summary += "--- End of History ---\n"
    return summary

@app.post("/init_task", response_model=TaskResponse)
async def init_task(request: InitialRequest):
    task_id = task_manager.create_task()
    task_state = task_manager.get_task(task_id)
    if not task_state:
        raise HTTPException(status_code=500, detail="Failed to create task state.")

    task_state.original_transcription = request.transcription # This is the overall goal
    task_state.grid_state_at_request = request.grid_state
    task_state.history = [] # Initialize history

    user_input_for_llm = (
        f"The user's overall task goal is: '{task_state.original_transcription}'.\n"
        f"The initial grid state is: '{task_state.grid_state_at_request}'.\n"
        f"This is the beginning of the task. Please provide the first logical step."
    )
    
    try:
        llm_json_response = await get_llm_response(user_input_for_llm, task_id)
        
        task_state.current_logical_step_speak = llm_json_response["speak"]
        task_state.current_primitive_actions = [PrimitiveAction(**pa) for pa in llm_json_response["actions"]]
        
        task_state.history.append({
            "type": "llm_proposal",
            "speak": task_state.current_logical_step_speak,
            "actions_proposed": [pa.dict() for pa in task_state.current_primitive_actions],
            "timestamp": time.time()
        })

        # Overall task is not complete on the first step, unless LLM says so (empty actions)
        overall_task_completed = not bool(task_state.current_primitive_actions)

        return TaskResponse(
            task_id=task_id,
            speak=task_state.current_logical_step_speak,
            primitive_actions=task_state.current_primitive_actions,
            completed=overall_task_completed 
        )
    except (HTTPException, ValueError) as e:
        raise e
    except Exception as e:
        print(f"Task {task_id}: Unexpected error in init_task: {e}")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Unexpected error initializing task: {str(e)}")

@app.post("/confirm_logical_step", response_model=TaskResponse)
async def confirm_logical_step(confirmation: LogicalStepConfirmation):
    task_state = task_manager.get_task(confirmation.task_id)
    if not task_state:
        raise HTTPException(status_code=404, detail=f"Task {confirmation.task_id} not found")

    # Log the outcome of the previously proposed logical step
    task_state.history.append({
        "type": "user_confirmation",
        "confirmed": confirmation.confirmed,
        "feedback_provided": confirmation.feedback, # This is feedback on the *previous* proposal
        "confirmed_speak": task_state.current_logical_step_speak, 
        "confirmed_actions": [pa.dict() for pa in task_state.current_primitive_actions], # Actions that were (or would have been) executed
        "timestamp": time.time()
    })

    history_summary_for_llm = build_history_summary_for_llm(task_state.history)
    
    if confirmation.confirmed:
        # Previous logical step's actions were executed by the robot.
        # Now, ask LLM for the NEXT logical step or if the overall task is complete.
        user_input_for_llm = (
            f"The user's overall task goal is: '{task_state.original_transcription}'.\n"
            f"The initial grid state was: '{task_state.grid_state_at_request}'.\n"
            f"{history_summary_for_llm}"
            f"The last logical step proposed by you ('{task_state.current_logical_step_speak}') with actions "
            f"{[pa.dict() for pa in task_state.current_primitive_actions]} was CONFIRMED by the user and EXECUTED by the robot.\n"
            f"What is the next logical step to continue achieving the overall goal? "
            f"If the overall goal is now complete, respond with an empty 'actions' array and a completion 'speak' message."
        )
    else: # User REJECTED the previously proposed logical step
        user_input_for_llm = (
            f"The user's overall task goal is: '{task_state.original_transcription}'.\n"
            f"The initial grid state was: '{task_state.grid_state_at_request}'.\n"
            f"{history_summary_for_llm}"
            f"Your last proposed logical step ('{task_state.current_logical_step_speak}') was REJECTED by the user."
        )
        if confirmation.feedback: # This feedback is crucial for re-planning
            user_input_for_llm += f" The user provided this feedback: '{confirmation.feedback}'.\n"
        user_input_for_llm += "Please provide a new logical step to achieve the overall goal, considering this rejection and feedback."

    try:
        llm_json_response = await get_llm_response(user_input_for_llm, confirmation.task_id)
        
        task_state.current_logical_step_speak = llm_json_response["speak"]
        task_state.current_primitive_actions = [PrimitiveAction(**pa) for pa in llm_json_response["actions"]]
        
        # Add this new LLM proposal to history
        task_state.history.append({
            "type": "llm_proposal",
            "speak": task_state.current_logical_step_speak,
            "actions_proposed": [pa.dict() for pa in task_state.current_primitive_actions],
            "timestamp": time.time()
        })

        overall_task_completed = not bool(task_state.current_primitive_actions)
        if overall_task_completed:
            print(f"Task {confirmation.task_id}: LLM indicates overall task is complete.")
            # Optionally, you could add a final "overall_task_complete" entry to history here.

        return TaskResponse(
            task_id=confirmation.task_id,
            speak=task_state.current_logical_step_speak,
            primitive_actions=task_state.current_primitive_actions,
            completed=overall_task_completed
        )
    except (HTTPException, ValueError) as e:
        raise e
    except Exception as e:
        print(f"Task {confirmation.task_id}: Unexpected error in confirm_logical_step: {e}")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Unexpected error processing confirmation: {str(e)}")

@app.get("/task_status/{task_id}")
async def get_task_status(task_id: str):
    task_state = task_manager.get_task(task_id)
    if not task_state:
        raise HTTPException(status_code=404, detail="Task not found")
    
    return {
        "task_id": task_id,
        "overall_goal": task_state.original_transcription,
        "initial_grid_state": task_state.grid_state_at_request,
        "current_proposed_speak": task_state.current_logical_step_speak,
        "current_proposed_primitive_actions": [pa.dict() for pa in task_state.current_primitive_actions],
        "history_count": len(task_state.history),
        "history_preview": task_state.history[-3:] if len(task_state.history) > 3 else task_state.history # Last 3 history entries
    }

if __name__ == "__main__":
    import uvicorn
    if not system_prompt:
        print("CRITICAL: GPT Step Service is starting WITHOUT a valid system prompt. Most functionality will fail.")
    else:
        print("GPT Step Service starting with a loaded system prompt.")
    
    port = int(os.getenv("PORT", "8000"))
    print(f"Starting GPT Step Service on host 0.0.0.0 port {port}")
    uvicorn.run("main_step:app", host="0.0.0.0", port=port, reload=True)