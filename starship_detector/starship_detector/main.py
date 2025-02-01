from ultralytics import YOLO

model = YOLO("yolov8n.pt")

train_results = model.train(
    data="config.yaml",  
    epochs=100 ,
    device="cpu",
    optimizer="Adam",
    conf= 0.3,
    lr0=0.01,         
    lrf=0.1 
)

model.save("mymodel.pt")
