import cv2
import torch
import numpy as np
# assume mediapipe for keypoints, ROS publisher 'gest_pub' exists
model = torch.jit.load("gesture_lstm.pt")  # scripted model
model.eval()
window = []  # temporal buffer
W = 30  # window length (frames)
tau = 0.8  # confidence threshold
k_req = 3  # consecutive-frame hysteresis
consensus = []  # recent confirmed labels

cap = cv2.VideoCapture(0)  # camera input
while True:
    ret, frame = cap.read()
    if not ret: break
    keypoints = extract_keypoints(frame)  # returns Nx2 or Nx3, None if no person
    if keypoints is None:
        window.clear()
        continue
    vec = normalize_keypoints(keypoints)  # translate & scale normalization
    window.append(vec)
    if len(window) > W: window.pop(0)
    if len(window) == W:
        seq = torch.tensor(np.stack(window)).float().unsqueeze(0)  # (1,W,n)
        with torch.no_grad():
            logits = model(seq)  # (1,classes)
            probs = torch.softmax(logits[0], dim=0).cpu().numpy()
        label = int(np.argmax(probs))
        conf = float(np.max(probs))
        # hysteresis: require k_req confirmations above tau
        consensus.append((label, conf))
        if len(consensus) > k_req: consensus.pop(0)
        if all(l==label and c>=tau for l,c in consensus):
            gest_pub.publish(label)  # ROS publish gesture id
            consensus.clear()