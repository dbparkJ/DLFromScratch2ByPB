from ultralytics import YOLO

model = YOLO('./best.pt')

# results = model(source='./data', save=True)

print(model.info(detailed=True))
