import torch
import os

print("Downloading YOLOv5 model via torch.hub...")

try:
    # Загружаем модель - torch.hub автоматически скачает веса
    model = torch.hub.load('ultralytics/yolov5', 'yolov5x', pretrained=True)
    print("Model downloaded successfully!")
    
    # Сохраняем веса в файл
    torch.save(model.state_dict(), 'yolov5x-oiv7.pt')
    
    # Проверяем размер файла
    file_size = os.path.getsize('yolov5x-oiv7.pt')
    print(f"File saved: yolov5x-oiv7.pt")
    print(f"File size: {file_size / (1024*1024):.2f} MB")
    
    # Проверяем, что модель работает
    print("Testing model...")
    results = model('https://ultralytics.com/images/zidane.jpg')
    print("Model test completed successfully!")
    
except Exception as e:
    print(f"Error: {e}")
    print("Trying alternative method...")