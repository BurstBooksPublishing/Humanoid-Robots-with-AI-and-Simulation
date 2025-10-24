import cv2
import numpy as np

# Load rectification maps and Q (reprojection) from calibration file
maps = np.load('stereo_rect_maps.npz')  # contains left_map1,left_map2,right_map1,right_map2,Q

left_map1,left_map2 = maps['left_map1'], maps['left_map2']
right_map1,right_map2 = maps['right_map1'], maps['right_map2']
Q = maps['Q']

# Create SGBM matcher tuned for humanoid head baseline and range
matcher = cv2.StereoSGBM_create(
    minDisparity=0, numDisparities=128, blockSize=5,
    P1=8*3*5**2, P2=32*3*5**2, mode=cv2.STEREO_SGBM_MODE_HH
)

# Acquire and rectify frames (assume synchronized capture)
left_raw = cv2.imread('left.png', cv2.IMREAD_GRAYSCALE)   # camera capture in practice
right_raw = cv2.imread('right.png', cv2.IMREAD_GRAYSCALE)
left = cv2.remap(left_raw, left_map1, left_map2, cv2.INTER_LINEAR)
right = cv2.remap(right_raw, right_map1, right_map2, cv2.INTER_LINEAR)

disp = matcher.compute(left, right).astype(np.float32) / 16.0  # scale factor for SGBM
points_3d = cv2.reprojectImageTo3D(disp, Q)                    # metric XYZ points
depth = points_3d[...,2]                                      # depth channel
# Compute confidence mask from disparity and left-right consistency (not shown)