from fastapi import FastAPI, File, UploadFile
import numpy as np          
import pandas as pd        
import tensorflow as tf                 # 2.16.1
from tensorflow.keras.models import load_model
from PIL import Image
import uvicorn

MODEL1_PATH = "C:\\Users\\geand\\OneDrive\\Documents\\CSUCI\\ros_2025\\ros_2025\\gpt_vision\\src\\models\\grid_model_equalized.h5"
MODEL2_PATH = "C:\\Users\\geand\\OneDrive\\Documents\\CSUCI\\ros_2025\\ros_2025\\gpt_vision\\src\\models\\grid_model_unequalized.keras"
LABELS_PATH = "C:\\Users\\geand\\OneDrive\\Documents\\CSUCI\\SURF 2024\\Grid Model\\grid_model_test_photos\\test_labels.csv"
# Have to use this image resolution, smaller than camera resolution for
# model training and prediction efficiency
IMG_RES = (230,440)

app = FastAPI()

LABELS = [
    "There is a yellow circle in space 1. ",
    "There is a red circle in space 1. ",
    "There is a blue circle in space 1. ",
    "There is a green circle in space 1. ",
    "There is a yellow triangle in space 1. ",
    "There is a red triangle in space 1. ",
    "There is a blue triangle in space 1. ",
    "There is a green triangle in space 1. ",
    "There is a yellow square in space 1. ",
    "There is a red square in space 1. ",
    "There is a blue square in space 1. ",
    "There is a green square in space 1. ",
    "Space 1 is empty. ",
    "There is a yellow circle in space 2. ",
    "There is a red circle in space 2. ",
    "There is a blue circle in space 2. ",
    "There is a green circle in space 2. ",
    "There is a yellow triangle in space 2. ",
    "There is a red triangle in space 2. ",
    "There is a blue triangle in space 2. ",
    "There is a green triangle in space 2. ",
    "There is a yellow square in space 2. ",
    "There is a red square in space 2. ",
    "There is a blue square in space 2. ",
    "There is a green square in space 2. ",
    "Space 2 is empty. ",
    "There is a yellow circle in space 3. ",
    "There is a red circle in space 3. ",
    "There is a blue circle in space 3. ",
    "There is a green circle in space 3. ",
    "There is a yellow triangle in space 3. ",
    "There is a red triangle in space 3. ",
    "There is a blue triangle in space 3. ",
    "There is a green triangle in space 3. ",
    "There is a yellow square in space 3. ",
    "There is a red square in space 3. ",
    "There is a blue square in space 3. ",
    "There is a green square in space 3. ",
    "Space 3 is empty. ",
    "There is a yellow circle in space 4. ",
    "There is a red circle in space 4. ",
    "There is a blue circle in space 4. ",
    "There is a green circle in space 4. ",
    "There is a yellow triangle in space 4. ",
    "There is a red triangle in space 4. ",
    "There is a blue triangle in space 4. ",
    "There is a green triangle in space 4. ",
    "There is a yellow square in space 4. ",
    "There is a red square in space 4. ",
    "There is a blue square in space 4. ",
    "There is a green square in space 4. ",
    "Space 4 is empty. ",
    "There is a yellow circle in space 5. ",
    "There is a red circle in space 5. ",
    "There is a blue circle in space 5. ",
    "There is a green circle in space 5. ",
    "There is a yellow triangle in space 5. ",
    "There is a red triangle in space 5. ",
    "There is a blue triangle in space 5. ",
    "There is a green triangle in space 5. ",
    "There is a yellow square in space 5. ",
    "There is a red square in space 5. ",
    "There is a blue square in space 5. ",
    "There is a green square in space 5. ",
    "Space 5 is empty. ",
    "There is a yellow circle in space 6. ",
    "There is a red circle in space 6. ",
    "There is a blue circle in space 6. ",
    "There is a green circle in space 6. ",
    "There is a yellow triangle in space 6. ",
    "There is a red triangle in space 6. ",
    "There is a blue triangle in space 6. ",
    "There is a green triangle in space 6. ",
    "There is a yellow square in space 6. ",
    "There is a red square in space 6. ",
    "There is a blue square in space 6. ",
    "There is a green square in space 6. ",
    "Space 6 is empty. ",
    "There is a yellow circle in space 7. ",
    "There is a red circle in space 7. ",
    "There is a blue circle in space 7. ",
    "There is a green circle in space 7. ",
    "There is a yellow triangle in space 7. ",
    "There is a red triangle in space 7. ",
    "There is a blue triangle in space 7. ",
    "There is a green triangle in space 7. ",
    "There is a yellow square in space 7. ",
    "There is a red square in space 7. ",
    "There is a blue square in space 7. ",
    "There is a green square in space 7. ",
    "Space 7 is empty. ",
    "There is a yellow circle in space 8. ",
    "There is a red circle in space 8. ",
    "There is a blue circle in space 8. ",
    "There is a green circle in space 8. ",
    "There is a yellow triangle in space 8. ",
    "There is a red triangle in space 8. ",
    "There is a blue triangle in space 8. ",
    "There is a green triangle in space 8. ",
    "There is a yellow square in space 8. ",
    "There is a red square in space 8. ",
    "There is a blue square in space 8. ",
    "There is a green square in space 8. ",
    "Space 8 is empty. ",
    "There is a yellow circle in space 9. ",
    "There is a red circle in space 9. ",
    "There is a blue circle in space 9. ",
    "There is a green circle in space 9. ",
    "There is a yellow triangle in space 9. ",
    "There is a red triangle in space 9. ",
    "There is a blue triangle in space 9. ",
    "There is a green triangle in space 9. ",
    "There is a yellow square in space 9. ",
    "There is a red square in space 9. ",
    "There is a blue square in space 9. ",
    "There is a green square in space 9. ",
    "Space 9 is empty. ",
    "There is a yellow circle in space 10. ",
    "There is a red circle in space 10. ",
    "There is a blue circle in space 10. ",
    "There is a green circle in space 10. ",
    "There is a yellow triangle in space 10. ",
    "There is a red triangle in space 10. ",
    "There is a blue triangle in space 10. ",
    "There is a green triangle in space 10. ",
    "There is a yellow square in space 10. ",
    "There is a red square in space 10. ",
    "There is a blue square in space 10. ",
    "There is a green square in space 10. ",
    "Space 10 is empty. ",
    "There is a yellow circle in space 11. ",
    "There is a red circle in space 11. ",
    "There is a blue circle in space 11. ",
    "There is a green circle in space 11. ",
    "There is a yellow triangle in space 11. ",
    "There is a red triangle in space 11. ",
    "There is a blue triangle in space 11. ",
    "There is a green triangle in space 11. ",
    "There is a yellow square in space 11. ",
    "There is a red square in space 11. ",
    "There is a blue square in space 11. ",
    "There is a green square in space 11. ",
    "Space 11 is empty. ",
    "There is a yellow circle in space 12. ",
    "There is a red circle in space 12. ",
    "There is a blue circle in space 12. ",
    "There is a green circle in space 12. ",
    "There is a yellow triangle in space 12. ",
    "There is a red triangle in space 12. ",
    "There is a blue triangle in space 12. ",
    "There is a green triangle in space 12. ",
    "There is a yellow square in space 12. ",
    "There is a red square in space 12. ",
    "There is a blue square in space 12. ",
    "There is a green square in space 12. ",
    "Space 12 is empty. ",]

model1 = load_model(MODEL1_PATH)
model2 = load_model(MODEL2_PATH)
print("Models loaded.")
df = pd.read_csv(LABELS_PATH)

def make_readable(soft_vote_predictions):

    return f"Shape of ResNet result: {soft_vote_predictions.shape}"

def soft_vote(resized_img):
    # use two models to predict on all images
    predictions1 = model1.predict(resized_img)
    predictions2 = model2.predict(resized_img)
    # soft voting (average) of all outputs for accurate predictions
    soft_vote_predictions = (predictions1 + predictions2) / 2
    return make_readable(soft_vote_predictions)

def process_image(image_bytes):
    image_tensor = tf.io.decode_image(image_bytes, channels = 3)
    resized_img = tf.image.resize(image_tensor, IMG_RES)
    batched_img = tf.expand_dims(resized_img, axis=0)
    print(batched_img.shape)
    return soft_vote(batched_img)

@app.post("/predict/")
async def predict(file: UploadFile = File(...)):
    image_bytes = await file.read()
    result = process_image(image_bytes)
    return {"prediction": result}

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000)
