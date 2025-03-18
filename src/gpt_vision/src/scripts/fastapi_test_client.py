import requests

image_path = "C:\\Users\\geand\\OneDrive\\Documents\\CSUCI\\ros_2025\\ros_2025\\gpt_vision\\src\\images\\test_img.jpg"  # Replace with your image path
with open(image_path, "rb") as f:
    response = requests.post("http://127.0.0.1:8000/predict/", files={"file": f})

print("Prediction:", response.json())