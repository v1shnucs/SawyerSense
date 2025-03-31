#!/usr/bin/env python3
import json
import os
import time
from typing import List, Optional
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(title="GPT Service")
# Initialize OpenAI client with timeout and response cache
client = OpenAI(
    api_key=os.getenv("OPENAI_API_KEY"),
    timeout=10.0  # Set 10 second timeout
)
response_cache = {}  # Simple in-memory cache

# Load system prompt once at startup
def load_system_prompt():
    try:
        with open("system_prompt.json", "r") as f:
            prompt_data = json.load(f)
            # Cache the prompt in memory
            return prompt_data[0]["content"]
    except Exception as e:
        print(f"Error loading system prompt: {e}")
        return None

# Load system prompt once at startup
system_prompt = load_system_prompt()
if not system_prompt:
    raise RuntimeError("Failed to load system prompt")

class Action(BaseModel):
    action: str
    square: int

class GPTRequest(BaseModel):
    transcription: str
    grid_state: str

class GPTResponse(BaseModel):
    speak: str
    actions: List[Action]

@app.post("/gpt", response_model=GPTResponse)
async def process_gpt_request(request: GPTRequest):
    if not system_prompt:
        raise HTTPException(status_code=500, detail="System prompt not loaded")
    
    try:
        start_time = time.time()
        
        # Format the input for better readability
        user_input = "Transcription: {}\nGrid State: {}".format(
            request.transcription, request.grid_state
        )
        
        print(f"Processing request for transcription: {request.transcription}")
        
        # Check cache first
        cache_key = f"{request.transcription}:{request.grid_state}"
        if cache_key in response_cache:
            return response_cache[cache_key]

        # Call OpenAI API using chat completions with further optimized parameters
        response = client.chat.completions.create(
            model="gpt-3.5-turbo-1106",  # Using faster model
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_input}
            ],
            temperature=0.0,  # Deterministic responses
            max_tokens=100,   # Further reduced tokens
            response_format={"type": "json_object"},  # Force JSON response
            presence_penalty=0,
            frequency_penalty=0,
            top_p=0.1,       # More focused sampling
            seed=42          # Consistent responses
        )
        
        # Parse the response
        try:
            response_text = response.choices[0].message.content.strip()
            parsed_response = json.loads(response_text)
            
            # Log timing information
            api_time = time.time() - start_time
            print(f"OpenAI API call took {api_time:.2f} seconds")
            print(f"GPT Response: {response_text}")
            
            # Handle nested content structure
            if "content" in parsed_response:
                content = parsed_response["content"]
            else:
                content = parsed_response
                
            # Validate content structure
            if not isinstance(content, dict):
                raise ValueError("Content must be a dictionary")
            if "speak" not in content or "actions" not in content:
                raise ValueError("Response must contain 'speak' and 'actions' fields")
            if not isinstance(content["actions"], list):
                raise ValueError("'actions' must be an array")
            
            # Fix action field names if needed
            for action in content["actions"]:
                if "space" in action:
                    action["square"] = action["space"]
                    del action["space"]
            
            # Cache the valid response
            result = GPTResponse(**content)
            response_cache[cache_key] = result
            
            total_time = time.time() - start_time
            print(f"Total request processing took {total_time:.2f} seconds")
            
            return result
            
        except json.JSONDecodeError as e:
            raise HTTPException(status_code=500, detail=f"Failed to parse GPT response: {str(e)}")
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", "8000"))
    uvicorn.run(app, host="0.0.0.0", port=port)