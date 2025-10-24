import cv2, numpy as np
# load rectified images and parameters (f in pixels, baseline B in meters)
left = cv2.imread('left_rect.png', cv2.IMREAD_GRAYSCALE)  # rectified left
right = cv2.imread('right_rect.png', cv2.IMREAD_GRAYSCALE)  # rectified right
f = 700.0  # focal length in pixels (example)
B = 0.12   # baseline in meters (12 cm)
# create stereo matcher (tune numDisparities and blockSize for performance)
stereo = cv2.StereoBM_create(numDisparities=128, blockSize=15)
disp = stereo.compute(left, right).astype(np.float32) / 16.0  # OpenCV scaling
# avoid division by zero; mask invalid disparities
valid = disp > 0.1
depth = np.zeros_like(disp, dtype=np.float32)
depth[valid] = (f * B) / disp[valid]  # apply z = f*B/d
# depth now holds metric distances in meters for manipulation planning
cv2.imwrite('depth_meters.exr', depth)  # save for inspection (float32)