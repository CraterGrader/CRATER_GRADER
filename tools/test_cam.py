import cv2
import numpy as np

# Read from the camera
cap = cv2.VideoCapture(2)

# Setup data collection
cv2.namedWindow('Camera')

# Read one image and get dimensions
_, img = cap.read()
h,  w = img.shape[:2]

# Pinhole Camera Matrix
#mtx = np.array([[268.40353417,   0.,         313.35770022],
# [  0.,         268.76205534, 220.56018606],
# [  0.,           0.,           1.        ]])

# Fisheye Camera Matrix
K = np.array([[269.41299877,   0.,         311.52540063],
 [  0.,         269.60663548, 220.38433628],
 [  0.,           0.,           1.        ]]
)

# Pinholee Distortion Matrix
#dist = np.array([[-2.99931927e-01,  8.02614220e-02, 
#-2.06435047e-04, -3.69570374e-04,  -8.67699358e-03]])

# Fisheye Distortion Matrix
D = np.array([[-0.04884999],
    [ 0.01215806],
    [-0.01517078],
    [ 0.00470384]])

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