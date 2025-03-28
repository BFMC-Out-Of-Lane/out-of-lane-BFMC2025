from ultralytics import YOLO

# Load a YOLO11n PyTorch model
model = YOLO(r"/home/ool/out-of-lane-BFMC2025/src/Brain/src/vision/yolo11n_traffic_signs_V3.pt")

# Export the model to NCNN format
model.export(format="ncnn")  # creates 'yolo11n_ncnn_model'

# Load the exported NCNN model
# ncnn_model = YOLO("../yolov8n_ncnn_model")

# Run inference
# results = ncnn_model(r"D:\0_videos\BOSCH\STOP-DARK.mp4", show= True, conf=0.60)