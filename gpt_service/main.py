#!/usr/bin/env python3
import json
import os
import time
import re
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

def attempt_json_repair(text):
    """Attempt to repair common JSON formatting issues"""
    if not text:
        return None
        
    print(f"Attempting to repair JSON: {text[:100]}...")
    
    # Try original text first
    try:
        return json.loads(text)
    except:
        pass
    
    # Fix unterminated strings (common GPT issue)
    try:
        # Pattern to find unterminated JSON strings
        pattern = r'"([^"\\]*(\\.[^"\\]*)*)'
        # Find all matches that aren't followed by a closing quote
        unterminated = re.finditer(pattern + r'(?!")', text)
        
        fixed_text = text
        # For each match, add a closing quote
        for match in unterminated:
            end_pos = match.end()
            fixed_text = fixed_text[:end_pos] + '"' + fixed_text[end_pos:]
        
        # Test if the repair worked
        try:
            return json.loads(fixed_text)
        except:
            pass
    except Exception as e:
        print(f"Error in string repair: {e}")
        # Attempt to close unterminated JSON objects or arrays
        if text.count('{') > text.count('}'):
            text += '}' * (text.count('{') - text.count('}'))
        if text.count('[') > text.count(']'):
            text += ']' * (text.count('[') - text.count(']'))
        try:
            return json.loads(text)
        except Exception as e:
            print(f"Final repair attempt failed: {e}")
    
    # Try to balance braces and brackets
    try:
        open_braces = text.count('{')
        close_braces = text.count('}')
        open_brackets = text.count('[')
        close_brackets = text.count(']')
        
        fixed_text = text
        
        # Add missing closing braces
        if open_braces > close_braces:
            fixed_text += '}' * (open_braces - close_braces)
            
        # Add missing closing brackets
        if open_brackets > close_brackets:
            fixed_text += ']' * (open_brackets - close_brackets)
            
        try:
            return json.loads(fixed_text)
        except:
            pass
    except Exception as e:
        print(f"Error in brace balancing: {e}")
    
    return None

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
            max_tokens=200,   # Further reduced tokens
            response_format={"type": "json_object"},  # Force JSON response
            presence_penalty=0,
            frequency_penalty=0,
            top_p=0.1,       # More focused sampling
            seed=42          # Consistent responses
        )
        
        # Parse the response
        try:
            response_text = response.choices[0].message.content.strip()
            
            # Log the raw response for debugging
            print(f"Raw GPT response: {response_text}")
            
            # First try normal JSON parsing
            try:
                parsed_response = json.loads(response_text)
            except json.JSONDecodeError as e:
                print(f"JSON parsing error: {e}")
                # Attempt to repair malformed JSON
                parsed_response = attempt_json_repair(response_text)
                if not parsed_response:
                    raise ValueError(f"Failed to parse GPT response after repair attempts: {str(e)}")
                print("Successfully repaired malformed JSON")
            
            # Log timing information
            api_time = time.time() - start_time
            print(f"OpenAI API call took {api_time:.2f} seconds")
            print(f"GPT Response (parsed): {parsed_response}")
            
            # Handle nested content structure
            content = parsed_response
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