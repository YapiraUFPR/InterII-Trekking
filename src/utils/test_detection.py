import torch
import cv2

# load model
model = torch.hub.load('ultralytics/yolov5', 'custom', '/home/gab/projetos/yapira/bedman-trekker/Data/models/yolov5_finetuned/best.pt')

# example image
img = '/home/gab/projetos/yapira/bedman-trekker/Data/cone.jpg'

# inference
results = model(img)

print(len(results))

bbox = results.xyxy[0].numpy()

# draw bbox
img = cv2.imread(img)
for box in bbox:
    x1, y1, x2, y2, conf, cls = box
    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

cv2.imwrite('result.jpg', img)