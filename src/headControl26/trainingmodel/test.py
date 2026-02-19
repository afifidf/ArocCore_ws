from ultralytics import YOLO

model = YOLO("/home/afifi/aroc26/src/headControl26/aroc.pt")

# Export the model
model.export(format="openvino")  # creates 'yolo26n_openvino_model/'

# Load the exported OpenVINO model
# ov_model = YOLO("best (2)_openvino_model")