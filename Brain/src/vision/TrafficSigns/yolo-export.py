from ultralytics import YOLO

# Load a YOLO11n PyTorch model
model = YOLO("yolo11n_traffic_objects_V11.pt")

# source = "http://192.168.0.198:4747/video"

# resultS = model.predict(source=source, conf=0.25, device="cpu")

# # Export the model to TensorRT with DLA enabled (only works with FP16 or INT8)
# model.export(format="engine", device="dla:0", half=True)  # dla:0 or dla:1 corresponds to the DLA cores
model.export(format="engine")

# Load the exported TensorRT model
# trt_model = YOLO("yolo11n.engine")
print("TensorRT model loaded successfully.")

# Run inference
# results = trt_model("https://ultralytics.com/images/bus.jpg")