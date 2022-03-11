#!/usr/bin/env python

from urllib.request import DataHandler
import cv2
import numpy as np
import os
import glob

# Defining the dimensions of checkerboard
CHECKERBOARD = (9, 11)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + \
  cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []

# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                          0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Setup data collection
cv2.namedWindow("Camera")

# Extracting path of individual image stored in a given directory
images = glob.glob('cal_cam_data/*.png')

for fname in images:
  img = cv2.imread(fname)
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  print("Checking image for corners...")

  # Find the chess board corners
  # If desired number of corners are found in the image then ret = true
  ret, corners = cv2.findChessboardCorners(
      gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

  """
  If desired number of corner are detected,
  we refine the pixel coordinates and display 
  them on the images of checker board
  """
  if ret == True:
    print("Corners found!")
    # refining pixel coordinates for given 2d points.
    corners2 = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1), criteria)

    # Draw and display the corners
    img_corners = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    cv2.imshow('Camera', img_corners)

    # Check for user input
    print("Image OK? Press Space to continue and ESC to cancel.")
    k = cv2.waitKey(0)
    if k % 256 == 27:  # ESC pressed
      print("Escape hit, closing...")
      break
    
    elif k % 256 == 32:  # SPACE pressed
      # Save data
      objpoints.append(objp)
      imgpoints.append(corners2)
  else:
    print("Corners not found, skipping image...")
cv2.destroyAllWindows()

# h, w = img.shape[:2]

if len(objpoints) > 0 and len(imgpoints) > 0:
  """
  Performing camera calibration by 
  passing the value of known 3D points (objpoints)
  and corresponding pixel coordinates of the 
  detected corners (imgpoints)
  """

  N_OK = len(objpoints)
  K = np.zeros((3, 3))
  D = np.zeros((4, 1))
  rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
  tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

  ret, _, _, _, _ = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    K,
    D,
    rvecs,
    tvecs,
    calibration_flags,
    (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

  print("Camera matrix : \n")
  print(K)
  print("dist : \n")
  print(D)
  print("rvecs : \n")
  print(rvecs)
  print("tvecs : \n")
  print(tvecs)
