#!/usr/bin/env python3
import json
import os
import time
from typing import List, Optional, Dict
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from openai import OpenAI
from dotenv import load_dotenv
import uuid
import traceback

# Load environment variables
load_dotenv()

app = FastAPI(title="GPT Step Auto Service")
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY"),
    timeout=20.0
)

# Load system prompt
def load_system_prompt() -> Optional[str]:
    try:
        prompt_file_path = os.path.join(os.path.dirname(__file__), "system_prompt_no_confirm.json")
        if not os.path.exists(prompt_file_path):
            print(f"Error: System prompt file not found at {prompt_file_path}")
            return None
        with open(prompt_file_path, "r") as f:
            prompt_data = json.load(f)
            return prompt_data[0]["content"]
    except Exception as e:
        print(f"Error loading system prompt: {e}")
        return None

system_prompt = load_system_prompt()
if not system_prompt:
    print("CRITICAL: Failed to load system prompt. GPT functionality will be impaired.")

# Models
class PrimitiveAction(BaseModel):
    action: str
    space: int

class TaskState:
    def __init__(self):
        self.current_logical_step_speak: str = ""
        self.current_primitive_actions: List[PrimitiveAction] = []
        self.grid_state_at_request: str = ""
        self.original_transcription: str = ""
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
    completed: bool

async def get_llm_response(user_input_for_llm: str, task_id: Optional[str] = None) -> Dict:
    if not system_prompt:
        print(f"Task {task_id or 'N/A'}: Cannot call LLM, system_prompt is not loaded.")
        raise HTTPException(status_code=500, detail="System prompt not loaded, cannot process request.")
    try:
        print(f"Task {task_id or 'N/A'}: Sending to LLM (first 200 chars): {user_input_for_llm[:200]}...")
        llm_api_response = client.responses.create(
            model="gpt-5",
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
            # Reuse attempt_json_repair by inlining a simple repair routine
            def attempt_json_repair_local(text):
                if not text:
                    return None
                try:
                    return json.loads(text)
                except:
                    pass
                try:
                    if text.count('{') > text.count('}'):
                        text += '}' * (text.count('{') - text.count('}'))
                    if text.count('[') > text.count(']'):
                        text += ']' * (text.count('[') - text.count(']'))
                    return json.loads(text)
                except:
                    return None
            parsed_llm_json = attempt_json_repair_local(response_text)
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
                print(f"Task {task_id or 'N/A'}: Warning - Invalid space number {prim_action['space']} in primitive action. Allowing for now.")
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
            summary += f"System marked your prior step as {status}.{fb}\n"
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

    task_state.original_transcription = request.transcription
    task_state.grid_state_at_request = request.grid_state
    task_state.history = []

    user_input_for_llm = (
        f"The user's overall task goal is: '{task_state.original_transcription}'.\n"
        f"The initial grid state is: '{task_state.grid_state_at_request}'.\n"
        f"This is the beginning of the task. Please provide the first logical step. Do not ask for user confirmation."
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
        "feedback_provided": confirmation.feedback,
        "confirmed_speak": task_state.current_logical_step_speak,
        "confirmed_actions": [pa.dict() for pa in task_state.current_primitive_actions],
        "timestamp": time.time()
    })

    history_summary_for_llm = build_history_summary_for_llm(task_state.history)

    if confirmation.confirmed:
        user_input_for_llm = (
            f"The user's overall task goal is: '{task_state.original_transcription}'.\n"
            f"The initial grid state was: '{task_state.grid_state_at_request}'.\n"
            f"{history_summary_for_llm}"
            f"The last logical step ('{task_state.current_logical_step_speak}') with actions "
            f"{[pa.dict() for pa in task_state.current_primitive_actions]} was EXECUTED by the robot.\n"
            f"Provide the next logical step. If the overall goal is complete, return empty 'actions' and a completion 'speak' message."
        )
    else:
        user_input_for_llm = (
            f"The user's overall task goal is: '{task_state.original_transcription}'.\n"
            f"The initial grid state was: '{task_state.grid_state_at_request}'.\n"
            f"{history_summary_for_llm}"
            f"The last proposed logical step ('{task_state.current_logical_step_speak}') was REJECTED."
        )
        if confirmation.feedback:
            user_input_for_llm += f" Feedback: '{confirmation.feedback}'.\n"
        user_input_for_llm += "Propose an alternative logical step."

    try:
        llm_json_response = await get_llm_response(user_input_for_llm, confirmation.task_id)

        task_state.current_logical_step_speak = llm_json_response["speak"]
        task_state.current_primitive_actions = [PrimitiveAction(**pa) for pa in llm_json_response["actions"]]

        task_state.history.append({
            "type": "llm_proposal",
            "speak": task_state.current_logical_step_speak,
            "actions_proposed": [pa.dict() for pa in task_state.current_primitive_actions],
            "timestamp": time.time()
        })

        overall_task_completed = not bool(task_state.current_primitive_actions)
        if overall_task_completed:
            print(f"Task {confirmation.task_id}: LLM indicates overall task is complete.")

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
        "history_preview": task_state.history[-3:] if len(task_state.history) > 3 else task_state.history
    }

if __name__ == "__main__":
    import uvicorn
    if not system_prompt:
        print("CRITICAL: GPT Auto Step Service is starting WITHOUT a valid system prompt. Most functionality will fail.")
    else:
        print("GPT Auto Step Service starting with a loaded system prompt.")
    port = int(os.getenv("PORT", "8000"))
    print(f"Starting GPT Auto Step Service on host 0.0.0.0 port {port}")
    uvicorn.run("main_no_confirm:app", host="0.0.0.0", port=port, reload=True)