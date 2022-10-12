#!/usr/bin/env python

import os

import cv2
import numpy as np

# Defining the dimensions of checkerboard
CHECKERBOARD = (9, 11)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []

# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Setup data collection
cv2.namedWindow("Camera")
img_counter = 1

# Get file path to data direcotry and create the directory if it doesn't already exist
cal_cam_data = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "./onboard_1920/"
)
if not os.path.exists(cal_cam_data):
    os.makedirs(cal_cam_data)

# Read from the camera
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Set to YUYV to match the default on the usb_cam
fourcc_cap = cv2.VideoWriter_fourcc(*"YUYV")
cap.set(cv2.CAP_PROP_FOURCC, fourcc_cap)
print(cap.get(cv2.CAP_PROP_FOURCC))

# Extracting path of individual image stored in a given directory
print("\nTo take picture, click on video and press 'SPACE' (may lag after pressing)")
print("To quit, press 'ESC'\n")

while True:
    _, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Camera", img)
    img_cp = img.copy()

    # Wait for user input
    k = cv2.waitKey(1)
    if k % 256 == 27:  # ESC pressed
        print("Escape hit, closing...")
        break

    elif k % 256 == 32:  # SPACE pressed
        print("Checking image for corners...")

        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        """
    If desired number of corner are detected,
    we refine the pixel coordinates and display 
    them on the images of checker board
    """
        if ret:
            print("Corners found!")
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Draw and display the corners
            img_corners = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow("Camera", img_corners)

            # Check for user input
            print("Use image for calibration and save? Press 'y/n'")
            k = cv2.waitKey(0)

            if k % 256 == 110:  # n pressed
                print("'n' pressed, skipping image...")
                print(
                    "\nTo take picture, click on video and press 'SPACE' (may lag after pressing)"
                )
                print("To quit, press 'ESC'\n")
                continue

            elif k % 256 == 121:  # y pressed
                print("Saving image data and file...")
                # Save data
                objpoints.append(objp)
                imgpoints.append(corners2)

                # Save file
                img_name = "onboard_1920_{:03d}.png".format(img_counter)
                cv2.imwrite(cal_cam_data + img_name, img_cp)
                print(f"{img_name} written!")
                img_counter += 1
                print(
                    "\nTo take picture, click on video and press 'SPACE' (may lag after pressing)"
                )
                print("To quit, press 'ESC'\n")
        else:
            print("Corners not found, skipping image...")
            print(
                "\nTo take picture, click on video and press 'SPACE' (may lag after pressing)"
            )
            print("To quit, press 'ESC'\n")
cv2.destroyAllWindows()

# h, w = img.shape[:2]

if len(objpoints) > 0 and len(imgpoints) > 0:
    """
    Performing camera calibration by
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the
    detected corners (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)
    print("rvecs : \n")
    print(rvecs)
    print("tvecs : \n")
    print(tvecs)
