from ultralytics import YOLO

model = YOLO("models/y11ndetect.pt")

model.export(format="ncnn")