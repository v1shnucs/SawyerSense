# GPT Service for ROS1

This service provides a Python 3.11 FastAPI server that handles GPT-4 API calls for the ROS1 system.

## Setup

1. Create a Python 3.11 virtual environment:
```bash
python3.11 -m venv venv
source venv/bin/activate
```

2. Install requirements:
```bash
pip install -r requirements.txt
```

3. Configure environment:
   - Copy `.env.example` to `.env`
   - Update OPENAI_API_KEY in `.env`

## Running the Service

1. Start the FastAPI server:
```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

## Running the ROS Node

1. In your ROS workspace, set the GPT service URL:
```bash
export GPT_SERVICE_URL=http://localhost:8000
```

2. Run the ROS node:
```bash
rosrun gpt_vision get_gpt_response.py
```

## Testing

You can test the service directly using curl:
```bash
curl -X POST http://localhost:8000/gpt \
  -H "Content-Type: application/json" \
  -d '{
    "transcription": "Move the red circle from space 5 to space 9",
    "grid_state": "Current grid: Square 5 has red circle, Square 9 is empty"
  }'
```

## Architecture

- FastAPI service handles GPT-4 API calls (Python 3.11)
- ROS node communicates with service via HTTP (Python 2)
- Service returns structured JSON responses
- ROS node publishes responses to appropriate topics