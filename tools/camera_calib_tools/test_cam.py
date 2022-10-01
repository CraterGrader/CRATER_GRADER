import cv2
import numpy as np

# Read from the camera
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
# Set to YUYV to match the default on the usb_cam
fourcc_cap = cv2.VideoWriter_fourcc(*'YUYV')
cap.set(cv2.CAP_PROP_FOURCC, fourcc_cap)

# Setup data collection
cv2.namedWindow('Camera')

# Read one image and get dimensions
_, img = cap.read()
h,  w = img.shape[:2]

#  Camera Matrix
K = np.array([  [553.7564405,   0.,              952.68355522],
                [0.,            553.91867457,   495.19384415],
                [0.,            0.,             1.        ]]
)

# Distortion Matrix
D = np.array([[-0.04027236],
            [-0.00342575],
            [-0.00311329],
            [ 0.00038835]])

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