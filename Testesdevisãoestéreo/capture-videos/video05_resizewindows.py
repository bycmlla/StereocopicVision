import cv2

cap = cv2.VideoCapture("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")

cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Camera', 640, 480)  

frame_limit = 100 
frame_count = 0
while frame_count < frame_limit:
    frame_count += 1


cap.release()
cv2.destroyAllWindows()
