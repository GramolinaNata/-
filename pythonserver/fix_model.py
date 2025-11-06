from ultralytics import YOLO
import torch

print("Исправляем модель...")

# Способ 1: Загружаем через ultralytics
model = YOLO('yolov5x.pt')
model.save('yolov5x-oiv7-fixed.pt')  # Сохраняем правильно
print("Модель сохранена правильно как 'yolov5x-oiv7-fixed.pt'")

# Затем в app.py используйте:
# model = YOLO('yolov5x-oiv7-fixed.pt')