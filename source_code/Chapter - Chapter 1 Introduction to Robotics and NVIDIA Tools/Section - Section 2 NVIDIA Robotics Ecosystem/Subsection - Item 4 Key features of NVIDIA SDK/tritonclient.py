import numpy as np
import cv2
import tritonclient.http as httpclient

# init client (//: replace address with server IP)
client = httpclient.InferenceServerClient(url="localhost:8000", verbose=False)

# capture frame from camera (use hardware-synced stream on robot)
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
# preprocess: resize and convert to CHW float32
img = cv2.resize(frame, (224, 224)).astype(np.float32) / 255.0
img = np.transpose(img, (2, 0, 1))[None, ...]  # add batch dim

inputs = [httpclient.InferInput("input__0", img.shape, "FP32")]
inputs[0].set_data_from_numpy(img)
outputs = [httpclient.InferRequestedOutput("output__0")]

# synchronous inference (low-latency; async if batching multiple requests)
response = client.infer(model_name="humanoid_percept", inputs=inputs, outputs=outputs)
result = response.as_numpy("output__0")
# quick postprocess: classification or detection step
cap.release()