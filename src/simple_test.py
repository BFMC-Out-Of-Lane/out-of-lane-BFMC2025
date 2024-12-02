import cv2
import time
from ultralytics import YOLO

WINDOW_NAME = 'frame'
FRAME_RATE = 30

EXIT_DELAY = 1
cap = cv2.VideoCapture(0)
model = YOLO("src/yolo11n.pt")  # pretrained YOLO11n model
prev_t = 0

while cap.isOpened():
    if cv2.waitKey(EXIT_DELAY) & 0xFF == ord('q'):
        break

    if time.time() - prev_t > 1. / FRAME_RATE:
        ret, frame = cap.read()
        model.predict(frame, show=True)
    

cv2.destroyWindow(WINDOW_NAME)
