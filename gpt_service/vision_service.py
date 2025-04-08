#!/usr/bin/env python3
import os
import json
import time
import traceback
from typing import Optional
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from openai import OpenAI

app = FastAPI(title="Vision Service")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize OpenAI client with no persistent state
client = None

def get_client():
    global client
    if client is None:
        client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY"),
            timeout=30.0  # Increased timeout for vision API
        )
    return client

class VisionRequest(BaseModel):
    image_base64: str

class VisionResponse(BaseModel):
    grid_state: str

# Add global error handler
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    error_msg = f"Error: {str(exc)}\nTraceback: {traceback.format_exc()}"
    print(error_msg)
    return JSONResponse(
        status_code=500,
        content={"detail": str(exc)}
    )

@app.post("/vision", response_model=VisionResponse)
async def process_vision_request(request: VisionRequest):
    try:
        start_time = time.time()
        print("Received vision request, processing...")
        
        # Call GPT-4o API
        client = get_client()
        response = client.responses.create(
            model="gpt-4o",  # Using full GPT-4o model for better accuracy
            temperature=0,  # Add temperature=0 for consistent responses
            input=[{
                "role": "user",
                "content": [
                    {
                        "type": "input_text",
                        "text": """Analyze this grid image and identify objects:

Grid Layout:
- Top row: spaces 9,10,11,12
- Middle row: spaces 5,6,7,8
- Bottom row: spaces 1,2,3,4

Each space may contain an object with:
- Color: red, blue, yellow, or green
- Shape: circle, triangle, or square

For each object, respond with:
'Space X has [color] [shape]'
List one object per line.

If no objects are detected, respond with:
'Grid is empty'

If the image is unclear, respond with:
'Unable to analyze image'"""
                    },
                    {
                        "type": "input_image",
                        "image_url": f"data:image/jpeg;base64,{request.image_base64}",
                        "detail": "auto"
                    }
                ]
            }]
        )
        
        # Log raw response for debugging
        print(f"Raw response type: {type(response)}")
        print(f"Raw response: {response}")
        
        # Extract and validate response
        if not hasattr(response, 'output_text'):
            raise ValueError(f"Unexpected response format: {response}")
            
        grid_state = response.output_text.strip()
        if not grid_state:
            raise ValueError("Empty response from vision API")
            
        # Log complete response for debugging
        print(f"Complete grid state description: {grid_state}")
            
        # Log timing and results
        api_time = time.time() - start_time
        print(f"Vision API call took {api_time:.2f} seconds")
        print(f"Grid state: {grid_state}")
        
        return VisionResponse(grid_state=grid_state)
            
    except Exception as e:
        error_msg = f"Error processing vision request: {str(e)}\nTraceback: {traceback.format_exc()}"
        print(error_msg)
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", "8001"))
    uvicorn.run(app, host="0.0.0.0", port=port)