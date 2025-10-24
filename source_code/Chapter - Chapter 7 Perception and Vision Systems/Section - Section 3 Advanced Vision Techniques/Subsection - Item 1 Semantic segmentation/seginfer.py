import cv2
import numpy as np
import tensorrt as trt
# build or load engine previously compiled with TensorRT (omitted for brevity)
engine = ...  # TensorRT engine object
context = engine.create_execution_context()

def preprocess(frame):  # resize, normalize, convert to CHW
    img = cv2.resize(frame, (512,512))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    mean = np.array([0.485,0.456,0.406], dtype=np.float32)
    std = np.array([0.229,0.224,0.225], dtype=np.float32)
    img = (img - mean) / std
    return np.transpose(img, (2,0,1)).copy()

def postprocess(logits, depth, min_vol=0.002):  # logits->mask, depth filter
    seg = np.argmax(logits, axis=0).astype(np.uint8)
    # remove small floating segments using depth-backed area threshold
    # compute largest component volumes per class (simple heuristic)
    return seg

while True:
    ret, frame = camera.read()  # camera is calibrated RGB stream
    if not ret: break
    inp = preprocess(frame)
    # copy to device, run inference (bindings and buffer management omitted)
    logits = run_trt_inference(context, inp)  # returns CxHxW logits
    seg = postprocess(logits, depth_frame)  # depth_frame aligned to RGB
    # use segmentation for grasp candidate generation or obstacle flagging
    display = visualize_segmentation(frame, seg)  # simple overlay
    cv2.imshow('seg', display)
    if cv2.waitKey(1) & 0xFF == ord('q'): break