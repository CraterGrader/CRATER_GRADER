import cv2
import numpy as np

# Read from the camera
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# Set to YUYV to match the default on the usb_cam
fourcc_cap = cv2.VideoWriter_fourcc(*'YUYV')
cap.set(cv2.CAP_PROP_FOURCC, fourcc_cap)

# Setup data collection
cv2.namedWindow('Camera')

# Read one image and get dimensions
_, img = cap.read()
h,  w = img.shape[:2]

#  Camera Matrix
K = np.array([[ 359.23290304,    0.,          629.64159832],
 [   0.,          359.26041139,  321.40026019],
 [   0.,            0.,            1.        ]]
)

# Distortion Matrix
D = np.array([[ -4.20510300e-02],
 [ -3.43845925e-03],
 [ -7.62396222e-04],
 [ -9.83326585e-06]])

while(True):
    ret, img = cap.read()
    # Wait for user input
    k = cv2.waitKey(1)
    if k % 256 == 27:  # ESC pressed
        print("Escape hit, closing...")
        break
    if ret:
        # undistort
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, 
            D, np.eye(3), K, (w,h), cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, 
            interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imshow('Camera', undistorted_img)
        

# Closing all open windows 
cv2.destroyAllWindows() 