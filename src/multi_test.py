import multiprocessing as mp
import time

import cv2
from ultralytics import YOLO

WINDOW_NAME = 'frame'
EXIT_DELAY = 1
FRAME_SKIP = 3


def capture(cap, conn):
    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if frame_count % FRAME_SKIP == 0:
            conn.send(frame)

        frame_count += 1


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    model = YOLO("src/yolo11n.pt")  # pretrained YOLO11n model

    # Export the model to NCNN format
    # model.export(format="ncnn")  # creates '/yolo11n_ncnn_model'
    # Load the exported NCNN model
    # model = YOLO("src/yolo11n_ncnn_model")

    parent_conn, child_conn = mp.Pipe()
    p = mp.Process(target=capture, args=(cap, child_conn))
    p.start()
    while p.is_alive():
        if cv2.waitKey(EXIT_DELAY) & 0xFF == ord('q'):
            break

        ini_t = time.time()

        frame = parent_conn.recv()
        model.predict(frame, show=True)

        print(f'FPS: {1 / (time.time() - ini_t)}')

    cv2.destroyWindow(WINDOW_NAME)
    p.terminate()
