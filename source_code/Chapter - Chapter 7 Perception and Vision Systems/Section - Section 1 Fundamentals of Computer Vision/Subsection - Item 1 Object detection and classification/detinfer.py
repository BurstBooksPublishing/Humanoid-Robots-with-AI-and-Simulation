# Pre-warmed TensorRT engine; depth aligned to RGB camera.
import time
import numpy as np

def preprocess(rgb, target_size):
    # resize and normalize; returns CHW float32 tensor
    img = cv2.resize(rgb, target_size)  # // preserve aspect in production
    img = img.astype(np.float32) / 255.0
    return img.transpose(2,0,1)[None, ...]  # batch dim

def postprocess(boxes, scores, classes, score_thresh, nms_iou):
    keep = scores > score_thresh
    boxes, scores, classes = boxes[keep], scores[keep], classes[keep]
    # apply NMS (TensorRT or CPU fallback)
    idxs = torchvision.ops.nms(torch.tensor(boxes), torch.tensor(scores), nms_iou)
    return boxes[idxs], scores[idxs], classes[idxs]

while True:
    t0 = time.time()
    rgb = rgb_stream.read()         # // head camera frame
    depth = depth_stream.read()     # // aligned depth
    x = preprocess(rgb, (640,384))
    boxes, scores, classes = trt_engine.infer(x)  # // returns numpy arrays
    boxes, scores, classes = postprocess(boxes, scores, classes, 0.3, 0.45)
    # compute 3D centroids using median depth inside each box
    for b, s, c in zip(boxes, scores, classes):
        u0,v0,u1,v1 = map(int, b)
        z = np.median(depth[v0:v1, u0:u1])  # // robust depth sample
        if np.isfinite(z) and z>0.2:
            p = z * np.linalg.inv(K) @ np.array([ (u0+u1)/2, (v0+v1)/2, 1.0 ])
            publish_detection(label=c, score=float(s), position=p.tolist())  # // ROS message
    latency = time.time()-t0
    telemetry.log('det_latency', latency)  # // use for system health