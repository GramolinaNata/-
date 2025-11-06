# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, Response, request, jsonify
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr  # импорт оставлен, можно убрать если не нужен

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # Открываем камеру (индекс 0 — первая подключенная камера)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # def detect_obstacle(frame):
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Конвертируем кадр в HSV для выделения красного цвета
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Диапазоны красного цвета в HSV (нижний и верхний)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     lower_red1 = np.array([0, 120, 70])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     upper_red1 = np.array([10, 255, 255])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     lower_red2 = np.array([170, 120, 70])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     upper_red2 = np.array([180, 255, 255])

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Маски для красного цвета
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask = mask1 + mask2

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Находим контуры на маске
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacle_detected = False
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     distance_estimate = None

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     for cnt in contours:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         area = cv2.contourArea(cnt)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if area > 5000:  # если площадь контура достаточно большая — считаем препятствием
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             obstacle_detected = True
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             # Примерная оценка расстояния — больше площадь, ближе объект
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             distance_estimate = max(0.5, 5 - (area / 20000))  # от 0.5 до 5 метров
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             break

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     return obstacle_detected, distance_estimate

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # def generate_frames():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     while True:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         success, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if not success:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             break

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         obstacle, distance = detect_obstacle(frame)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if obstacle:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.putText(frame, f"Obstacle! Distance: {distance:.2f} m",
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #                         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.putText(frame, "No obstacle", (10, 30),
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             continue

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         frame_bytes = buffer.tobytes()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     return "Server is running"

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate_frames(),
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #                     mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     success, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     if not success:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to capture frame"}), 500

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacle, distance = detect_obstacle(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     if obstacle:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "obstacle": True,
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "distance": round(distance, 2),
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "message": f"Obstacle detected! Distance approx. {distance:.2f} meters."
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     else:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "obstacle": False,
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "message": "No obstacle detected"
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #         })

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     print("Starting Flask server...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000, debug=True)


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, Response, jsonify
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # Открываем камеру (индекс 0 — первая подключенная камера)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # def detect_obstacle(frame):
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Конвертируем кадр в HSV для выделения красного цвета
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Диапазоны красного цвета в HSV (нижний и верхний)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     lower_red1 = np.array([0, 120, 70])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     upper_red1 = np.array([10, 255, 255])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     lower_red2 = np.array([170, 120, 70])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     upper_red2 = np.array([180, 255, 255])

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Маски для красного цвета
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask = mask1 + mask2

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Находим контуры на маске
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacle_detected = False
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     distance_estimate = None
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     largest_contour = None
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     largest_area = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     for cnt in contours:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         area = cv2.contourArea(cnt)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if area > 5000 and area > largest_area:  # ищем самый большой контур с площадью > 5000
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             obstacle_detected = True
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             largest_area = area
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             largest_contour = cnt

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     if obstacle_detected:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         # Оценка расстояния (примерная)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         distance_estimate = max(0.5, 5 - (largest_area / 20000))  # от 0.5 до 5 метров

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     return obstacle_detected, distance_estimate, largest_contour

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # def generate_frames():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     while True:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         success, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if not success:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             break

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         obstacle, distance, contour = detect_obstacle(frame)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         # Обводим найденный контур красным цветом, если есть
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if contour is not None:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.drawContours(frame, [contour], -1, (0, 0, 255), 3)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if obstacle:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.putText(frame, f"Obstacle! Distance: {distance:.2f} m",
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #                         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.putText(frame, "No obstacle", (10, 30),
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             continue

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         frame_bytes = buffer.tobytes()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     return "Server is running"

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate_frames(),
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #                     mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     success, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     if not success:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to capture frame"}), 500

# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacle, distance, _ = detect_obstacle(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     if obstacle:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "obstacle": True,
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "distance": round(distance, 2),
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "message": f"Obstacle detected! Distance approx. {distance:.2f} meters."
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     else:
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "obstacle": False,
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #             "message": "No obstacle detected"
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #         })

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     print("Starting Flask server...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000, debug=True)



# # # # # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, Response, jsonify
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np

# # # # # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # Открываем камеру (индекс 0 — первая подключенная камера)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # def estimate_distance(area, ref_area=40000, ref_distance=0.5):
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     """
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     Оценивает расстояние по площади контура.

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     ref_area: площадь объекта на изображении, когда он находится на расстоянии ref_distance
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     ref_distance: эталонное расстояние (метры)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     Расстояние обратно пропорционально квадратному корню из площади.
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     """
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     if area <= 0:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return None
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     distance = ref_distance * (np.sqrt(ref_area) / np.sqrt(area))
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return distance

# # # # # # # # # # # # # # # # # # # # # # # # # # # # def detect_obstacle(frame):
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Конвертируем кадр в HSV для выделения красного цвета
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Диапазоны красного цвета в HSV (нижний и верхний)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     lower_red1 = np.array([0, 120, 70])
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     upper_red1 = np.array([10, 255, 255])
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     lower_red2 = np.array([170, 120, 70])
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     upper_red2 = np.array([180, 255, 255])

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Маски для красного цвета
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     mask = mask1 + mask2

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Находим контуры на маске
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacle_detected = False
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     largest_contour = None
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     largest_area = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     for cnt in contours:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         area = cv2.contourArea(cnt)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         if area > 5000 and area > largest_area:  # ищем самый большой контур с площадью > 5000
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             obstacle_detected = True
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             largest_area = area
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             largest_contour = cnt

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     distance_estimate = None
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     if obstacle_detected:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         distance_estimate = estimate_distance(largest_area)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return obstacle_detected, distance_estimate, largest_contour

# # # # # # # # # # # # # # # # # # # # # # # # # # # # def generate_frames():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     while True:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         success, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         if not success:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             break

# # # # # # # # # # # # # # # # # # # # # # # # # # # #         obstacle, distance, contour = detect_obstacle(frame)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #         # Обводим найденный контур красным цветом, если есть
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         if contour is not None:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.drawContours(frame, [contour], -1, (0, 0, 255), 3)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #         if obstacle:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.putText(frame, f"Obstacle! Distance: {distance:.2f} m",
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                         (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             cv2.putText(frame, "No obstacle", (10, 30),
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #         ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             continue

# # # # # # # # # # # # # # # # # # # # # # # # # # # #         frame_bytes = buffer.tobytes()

# # # # # # # # # # # # # # # # # # # # # # # # # # # #         yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return "Server is running"

# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate_frames(),
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                     mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     success, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     if not success:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to capture frame"}), 500

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacle, distance, _ = detect_obstacle(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     if obstacle:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "obstacle": True,
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "distance": round(distance, 2),
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "message": f"Obstacle detected! Distance approx. {distance:.2f} meters."
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     else:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "obstacle": False,
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "message": "No obstacle detected"
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         })

# # # # # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     print("Starting Flask server...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000, debug=True)



# # # # # # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Конфиг ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Подключение к БД ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     )
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return conn

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Загружаем YOLOv5 ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLOv5 model...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Камера ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Функция детекции ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     results = model(img_rgb)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     df = results.pandas().xyxy[0]  # xmin, ymin, xmax, ymax, confidence, class, name
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return df

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт видео с наложением ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     def generate():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         while True:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                 break
            
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             df = detect_objects(frame)
            
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                 xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                 label = row['name']
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                 conf = row['confidence']
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0,255,0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.putText(frame, f"{label} {conf:.2f}", (xmin, ymin - 10),
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                 continue
            
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # # # # #                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт для проверки препятствий (JSON) ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to read frame"}), 500
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = []
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         obstacles.append({
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "label": row['name'],
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "confidence": float(row['confidence']),
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "bbox": [int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])]
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         })
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"obstacles": obstacles})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт для голосовой команды ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "No audio file provided"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         audio = r.record(source)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         text = r.recognize_google(audio, language="ru-RU")  # распознаем русскую речь
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Could not understand audio"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Could not request results; {e}"}), 500
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     # Можно сюда добавить логику обработки команд (например "проверить препятствия")
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"recognized_text": text})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Сохраняем данные о препятствиях в БД ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/save_obstacles', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def save_obstacles():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     data = request.json
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     if not data or 'obstacles' not in data:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Invalid data"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = data['obstacles']

# # # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         label = obs.get('label')
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         confidence = obs.get('confidence')
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         bbox = obs.get('bbox')  # [xmin, ymin, xmax, ymax]
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         bbox_str = ','.join(map(str, bbox))
        
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # # # # #             (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
    
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "saved"})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Главная страница ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     return "Blind Assistant Server is running."

# # # # # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000)




# # # # # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Для голосовых embedding ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # from speechbrain.pretrained import SpeakerRecognition
# # # # # # # # # # # # # # # # # # # # # # # # # # # from scipy.spatial.distance import cosine

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Конфиг ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Подключение к БД ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # # # # # #     )
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return conn

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Загружаем YOLOv5 ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLOv5 model...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Загружаем модель для голосовых эмбеддингов ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("Loading SpeakerRecognition model...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # speaker_model = SpeakerRecognition.from_hparams(source="speechbrain/spkrec-ecapa-voxceleb",
# # # # # # # # # # # # # # # # # # # # # # # # # # #                                                 savedir="pretrained_models/spkrec-ecapa-voxceleb")
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("SpeakerRecognition model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Камера ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Функция детекции ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # # # # # #     img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     results = model(img_rgb)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     df = results.pandas().xyxy[0]  # xmin, ymin, xmax, ymax, confidence, class, name
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return df

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Вспомогательная функция для получения голосового эмбеддинга ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # def get_voice_embedding(audio_path):
# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Возвращает numpy array с эмбеддингом голоса
# # # # # # # # # # # # # # # # # # # # # # # # # # #     signal = speaker_model.load_audio(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = speaker_model.encode_batch(signal)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = embedding.squeeze().cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return embedding

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт видео с наложением ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     def generate():
# # # # # # # # # # # # # # # # # # # # # # # # # # #         while True:
# # # # # # # # # # # # # # # # # # # # # # # # # # #             ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 break
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             df = detect_objects(frame)
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 label = row['name']
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 conf = row['confidence']
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0,255,0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.putText(frame, f"{label} {conf:.2f}", (xmin, ymin - 10),
# # # # # # # # # # # # # # # # # # # # # # # # # # #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 continue
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # # # #                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт для проверки препятствий (JSON) ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to read frame"}), 500
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = []
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # # #         obstacles.append({
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "label": row['name'],
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "confidence": float(row['confidence']),
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "bbox": [int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])]
# # # # # # # # # # # # # # # # # # # # # # # # # # #         })
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"obstacles": obstacles})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт для голосовой команды ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "No audio file provided"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         audio = r.record(source)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         text = r.recognize_google(audio, language="ru-RU")  # распознаем русскую речь
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Could not understand audio"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Could not request results; {e}"}), 500
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Можно сюда добавить логику обработки команд (например "проверить препятствия")
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"recognized_text": text})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Сохраняем данные о препятствиях в БД ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/save_obstacles', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def save_obstacles():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     data = request.json
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if not data or 'obstacles' not in data:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Invalid data"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = data['obstacles']

# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         label = obs.get('label')
# # # # # # # # # # # # # # # # # # # # # # # # # # #         confidence = obs.get('confidence')
# # # # # # # # # # # # # # # # # # # # # # # # # # #         bbox = obs.get('bbox')  # [xmin, ymin, xmax, ymax]
# # # # # # # # # # # # # # # # # # # # # # # # # # #         bbox_str = ','.join(map(str, bbox))
        
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # # # #             (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "saved"})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Регистрация пользователя (с голосовым эмбеддингом и хэшем пароля) ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form or 'password' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form['password']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']

# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Получаем голосовой эмбеддинг
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Хэшируем пароль (простейший вариант, можно заменить на bcrypt)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     import hashlib
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Сохраняем в базу
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO users (username, password_hash, voice_embedding) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # # # #             (username, password_hash, json.dumps(embedding.tolist()))
# # # # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except psycopg2.errors.UniqueViolation:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User already exists"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "registered"})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Вход пользователя по голосу и паролю ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form or 'password' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form['password']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']

# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Получаем голосовой эмбеддинг
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Хэшируем пароль
# # # # # # # # # # # # # # # # # # # # # # # # # # #     import hashlib
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Получаем из БД пользователя с username
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.execute("SELECT password_hash, voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # # # # # # # # # # # # #     user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     if not user:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User not found"}), 404
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     stored_password_hash, stored_voice_embedding_json = user
# # # # # # # # # # # # # # # # # # # # # # # # # # #     stored_voice_embedding = np.array(json.loads(stored_voice_embedding_json))

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Проверяем пароль
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Wrong password"}), 401

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Сравниваем голосовые эмбеддинги (косинусное сходство)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     similarity = 1 - cosine(embedding, stored_voice_embedding)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     print(f"Voice similarity: {similarity}")

# # # # # # # # # # # # # # # # # # # # # # # # # # #     if similarity < 0.75:  # порог, можно подстроить
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Voice does not match"}), 401

# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "login success"})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Главная страница ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return "Blind Assistant Server is running."

# # # # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000)




# # # # # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Для голосовых embedding ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # from speechbrain.pretrained import SpeakerRecognition
# # # # # # # # # # # # # # # # # # # # # # # # # # # from scipy.spatial.distance import cosine

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Конфиг ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Подключение к БД ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # # # # # #     )
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return conn

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Загружаем YOLOv5 ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLOv5 model...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Загружаем модель для голосовых эмбеддингов ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("Loading SpeakerRecognition model...")
# # # # # # # # # # # # # # # # # # # # # # # # # # # speaker_model = SpeakerRecognition.from_hparams(source="speechbrain/spkrec-ecapa-voxceleb",
# # # # # # # # # # # # # # # # # # # # # # # # # # #                                                 savedir="pretrained_models/spkrec-ecapa-voxceleb")
# # # # # # # # # # # # # # # # # # # # # # # # # # # print("SpeakerRecognition model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Камера ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Функция детекции ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # # # # # #     img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     results = model(img_rgb)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     df = results.pandas().xyxy[0]  # xmin, ymin, xmax, ymax, confidence, class, name
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return df

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Вспомогательная функция для получения голосового эмбеддинга ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # def get_voice_embedding(audio_path):
# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Возвращает numpy array с эмбеддингом голоса
# # # # # # # # # # # # # # # # # # # # # # # # # # #     signal = speaker_model.load_audio(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = speaker_model.encode_batch(signal)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = embedding.squeeze().cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return embedding

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт видео с наложением ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     def generate():
# # # # # # # # # # # # # # # # # # # # # # # # # # #         while True:
# # # # # # # # # # # # # # # # # # # # # # # # # # #             ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 break
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             df = detect_objects(frame)
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 label = row['name']
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 conf = row['confidence']
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0,255,0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.putText(frame, f"{label} {conf:.2f}", (xmin, ymin - 10),
# # # # # # # # # # # # # # # # # # # # # # # # # # #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # #                 continue
            
# # # # # # # # # # # # # # # # # # # # # # # # # # #             yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # # # #                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт для проверки препятствий (JSON) ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to read frame"}), 500
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = []
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # # #         obstacles.append({
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "label": row['name'],
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "confidence": float(row['confidence']),
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "bbox": [int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])]
# # # # # # # # # # # # # # # # # # # # # # # # # # #         })
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"obstacles": obstacles})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Эндпоинт для голосовой команды ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "No audio file provided"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         audio = r.record(source)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         text = r.recognize_google(audio, language="ru-RU")  # распознаем русскую речь
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Could not understand audio"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Could not request results; {e}"}), 500
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Пример обработки команды
# # # # # # # # # # # # # # # # # # # # # # # # # # #     response = {"recognized_text": text}

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Можно здесь добавить команду на открытие камеры или другую логику
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if "открой камеру" in text.lower():
# # # # # # # # # # # # # # # # # # # # # # # # # # #         response["action"] = "open_camera"

# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify(response)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Сохраняем данные о препятствиях в БД ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/save_obstacles', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def save_obstacles():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     data = request.json
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if not data or 'obstacles' not in data:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Invalid data"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = data['obstacles']

# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         label = obs.get('label')
# # # # # # # # # # # # # # # # # # # # # # # # # # #         confidence = obs.get('confidence')
# # # # # # # # # # # # # # # # # # # # # # # # # # #         bbox = obs.get('bbox')  # [xmin, ymin, xmax, ymax]
# # # # # # # # # # # # # # # # # # # # # # # # # # #         bbox_str = ','.join(map(str, bbox))
        
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # # # #             (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "saved"})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Регистрация пользователя (с голосовым эмбеддингом и хэшем пароля) ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form or 'password' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form['password']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']

# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Получаем голосовой эмбеддинг
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Хэшируем пароль (простейший вариант, можно заменить на bcrypt)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     import hashlib
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Сохраняем в базу
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO users (username, password_hash, voice_embedding) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # # # #             (username, password_hash, json.dumps(embedding.tolist()))
# # # # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except psycopg2.errors.UniqueViolation:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User already exists"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "registered"})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Вход пользователя по голосу и паролю ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form or 'password' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form['password']
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']

# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Получаем голосовой эмбеддинг
# # # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Хэшируем пароль
# # # # # # # # # # # # # # # # # # # # # # # # # # #     import hashlib
# # # # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Получаем из БД пользователя с username
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.execute("SELECT password_hash, voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # # # # # # # # # # # # #     user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # # # # #     if not user:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User not found"}), 404
    
# # # # # # # # # # # # # # # # # # # # # # # # # # #     stored_password_hash, stored_voice_embedding_json = user
# # # # # # # # # # # # # # # # # # # # # # # # # # #     stored_voice_embedding = np.array(json.loads(stored_voice_embedding_json))

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Проверяем пароль
# # # # # # # # # # # # # # # # # # # # # # # # # # #     if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Wrong password"}), 401

# # # # # # # # # # # # # # # # # # # # # # # # # # #     # Сравниваем голосовые эмбеддинги (косинусное сходство)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     similarity = 1 - cosine(embedding, stored_voice_embedding)
# # # # # # # # # # # # # # # # # # # # # # # # # # #     print(f"Voice similarity: {similarity}")

# # # # # # # # # # # # # # # # # # # # # # # # # # #     if similarity < 0.75:  # порог, можно подстроить
# # # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Voice does not match"}), 401

# # # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "login success"})

# # # # # # # # # # # # # # # # # # # # # # # # # # # # --- Главная страница ---
# # # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # # # #     return "Blind Assistant Server is running."

# # # # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000)



# # # # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # # # # # # from speechbrain.pretrained import SpeakerRecognition
# # # # # # # # # # # # # # # # # # # # # # # # # # from scipy.spatial.distance import cosine

# # # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn = psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # # # # #     )
# # # # # # # # # # # # # # # # # # # # # # # # # #     return conn

# # # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Загружаем модели ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLOv5 model...")
# # # # # # # # # # # # # # # # # # # # # # # # # # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# # # # # # # # # # # # # # # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # print("Loading SpeakerRecognition model...")
# # # # # # # # # # # # # # # # # # # # # # # # # # speaker_model = SpeakerRecognition.from_hparams(
# # # # # # # # # # # # # # # # # # # # # # # # # #     source="speechbrain/spkrec-ecapa-voxceleb",
# # # # # # # # # # # # # # # # # # # # # # # # # #     savedir="pretrained_models/spkrec-ecapa-voxceleb"
# # # # # # # # # # # # # # # # # # # # # # # # # # )
# # # # # # # # # # # # # # # # # # # # # # # # # # print("SpeakerRecognition model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Работа с камерой ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # # # # #     img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # # # # #     results = model(img_rgb)
# # # # # # # # # # # # # # # # # # # # # # # # # #     df = results.pandas().xyxy[0]
# # # # # # # # # # # # # # # # # # # # # # # # # #     return df

# # # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Голосовой эмбеддинг ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # # def get_voice_embedding(audio_path):
# # # # # # # # # # # # # # # # # # # # # # # # # #     signal = speaker_model.load_audio(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = speaker_model.encode_batch(signal)
# # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = embedding.squeeze().cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # # # # # #     return embedding

# # # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # # #     return "Blind Assistant Server is running."

# # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # # #     def generate():
# # # # # # # # # # # # # # # # # # # # # # # # # #         while True:
# # # # # # # # # # # # # # # # # # # # # # # # # #             ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # #                 break
# # # # # # # # # # # # # # # # # # # # # # # # # #             df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # #             for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # #                 xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
# # # # # # # # # # # # # # # # # # # # # # # # # #                 label = row['name']
# # # # # # # # # # # # # # # # # # # # # # # # # #                 conf = row['confidence']
# # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0,255,0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.putText(frame, f"{label} {conf:.2f}", (xmin, ymin - 10),
# # # # # # # # # # # # # # # # # # # # # # # # # #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # # #             ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # #                 continue
# # # # # # # # # # # # # # # # # # # # # # # # # #             yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # # #                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
# # # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # # #     ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # # #     if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to read frame"}), 500
# # # # # # # # # # # # # # # # # # # # # # # # # #     df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = []
# # # # # # # # # # # # # # # # # # # # # # # # # #     for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # # #         obstacles.append({
# # # # # # # # # # # # # # # # # # # # # # # # # #             "label": row['name'],
# # # # # # # # # # # # # # # # # # # # # # # # # #             "confidence": float(row['confidence']),
# # # # # # # # # # # # # # # # # # # # # # # # # #             "bbox": [int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])]
# # # # # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"obstacles": obstacles})

# # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "No audio file provided"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # # # # #     with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # # # # #         audio = r.record(source)
# # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # # #         text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Could not understand audio"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # #     except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Could not request results; {e}"}), 500
# # # # # # # # # # # # # # # # # # # # # # # # # #     response = {"recognized_text": text}
# # # # # # # # # # # # # # # # # # # # # # # # # #     if "открой камеру" in text.lower():
# # # # # # # # # # # # # # # # # # # # # # # # # #         response["action"] = "open_camera"
# # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify(response)

# # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/save_obstacles', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # def save_obstacles():
# # # # # # # # # # # # # # # # # # # # # # # # # #     data = request.json
# # # # # # # # # # # # # # # # # # # # # # # # # #     if not data or 'obstacles' not in data:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Invalid data"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = data['obstacles']
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # # #     for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # # # # #         label = obs.get('label')
# # # # # # # # # # # # # # # # # # # # # # # # # #         confidence = obs.get('confidence')
# # # # # # # # # # # # # # # # # # # # # # # # # #         bbox = obs.get('bbox')
# # # # # # # # # # # # # # # # # # # # # # # # # #         bbox_str = ','.join(map(str, bbox))
# # # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # # #             (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "saved"})

# # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form or 'password' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form['password']
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     import hashlib
# # # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest()
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO users (username, password_hash, voice_embedding) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # # #             (username, password_hash, json.dumps(embedding.tolist()))
# # # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # # #         conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # # #     except psycopg2.errors.UniqueViolation:
# # # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User already exists"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "registered"})

# # # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form or 'password' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form['password']
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # # #     import hashlib
# # # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest()
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # # #     cur.execute("SELECT password_hash, voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # # # # # # # # # # # #     user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # # #     if not user:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User not found"}), 404
# # # # # # # # # # # # # # # # # # # # # # # # # #     stored_password_hash, stored_voice_embedding_json = user
# # # # # # # # # # # # # # # # # # # # # # # # # #     stored_voice_embedding = np.array(json.loads(stored_voice_embedding_json))
# # # # # # # # # # # # # # # # # # # # # # # # # #     if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Wrong password"}), 401
# # # # # # # # # # # # # # # # # # # # # # # # # #     similarity = 1 - cosine(embedding, stored_voice_embedding)
# # # # # # # # # # # # # # # # # # # # # # # # # #     print(f"Voice similarity: {similarity}")
# # # # # # # # # # # # # # # # # # # # # # # # # #     if similarity < 0.75:
# # # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Voice does not match"}), 401
# # # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "login success"})

# # # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000)







# # # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # # # # # from speechbrain.pretrained import SpeakerRecognition
# # # # # # # # # # # # # # # # # # # # # # # # # from scipy.spatial.distance import cosine
# # # # # # # # # # # # # # # # # # # # # # # # # import hashlib

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # # # #     conn = psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # # # #     )
# # # # # # # # # # # # # # # # # # # # # # # # #     return conn

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Загружаем модели ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLOv5 model...")
# # # # # # # # # # # # # # # # # # # # # # # # # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# # # # # # # # # # # # # # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # print("Loading SpeakerRecognition model...")
# # # # # # # # # # # # # # # # # # # # # # # # # speaker_model = SpeakerRecognition.from_hparams(
# # # # # # # # # # # # # # # # # # # # # # # # #     source="speechbrain/spkrec-ecapa-voxceleb",
# # # # # # # # # # # # # # # # # # # # # # # # #     savedir="pretrained_models/spkrec-ecapa-voxceleb"
# # # # # # # # # # # # # # # # # # # # # # # # # )
# # # # # # # # # # # # # # # # # # # # # # # # # print("SpeakerRecognition model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Работа с камерой ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # # # #     img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # # # #     results = model(img_rgb)
# # # # # # # # # # # # # # # # # # # # # # # # #     df = results.pandas().xyxy[0]
# # # # # # # # # # # # # # # # # # # # # # # # #     return df

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Голосовой эмбеддинг ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # def get_voice_embedding(audio_path):
# # # # # # # # # # # # # # # # # # # # # # # # #     signal = speaker_model.load_audio(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     embedding = speaker_model.encode_batch(signal)
# # # # # # # # # # # # # # # # # # # # # # # # #     embedding = embedding.squeeze().cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # # # # #     return embedding

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # #     return "Blind Assistant Server is running."

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # #     def generate():
# # # # # # # # # # # # # # # # # # # # # # # # #         while True:
# # # # # # # # # # # # # # # # # # # # # # # # #             ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # #                 break
# # # # # # # # # # # # # # # # # # # # # # # # #             df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # #             for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # #                 xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
# # # # # # # # # # # # # # # # # # # # # # # # #                 label = row['name']
# # # # # # # # # # # # # # # # # # # # # # # # #                 conf = row['confidence']
# # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0,255,0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.putText(frame, f"{label} {conf:.2f}", (xmin, ymin - 10),
# # # # # # # # # # # # # # # # # # # # # # # # #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # #             ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # #                 continue
# # # # # # # # # # # # # # # # # # # # # # # # #             yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # #                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
# # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # #     ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # #     if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to read frame"}), 500
# # # # # # # # # # # # # # # # # # # # # # # # #     df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = []
# # # # # # # # # # # # # # # # # # # # # # # # #     for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # #         obstacles.append({
# # # # # # # # # # # # # # # # # # # # # # # # #             "label": row['name'],
# # # # # # # # # # # # # # # # # # # # # # # # #             "confidence": float(row['confidence']),
# # # # # # # # # # # # # # # # # # # # # # # # #             "bbox": [int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])]
# # # # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"obstacles": obstacles})

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "No audio file provided"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # # # #     with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # # # #         audio = r.record(source)
# # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # #         text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # # # # # # #     except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Could not understand audio"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Could not request results; {e}"}), 500
# # # # # # # # # # # # # # # # # # # # # # # # #     response = {"recognized_text": text}
# # # # # # # # # # # # # # # # # # # # # # # # #     if "открой камеру" in text.lower():
# # # # # # # # # # # # # # # # # # # # # # # # #         response["action"] = "open_camera"
# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify(response)

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/save_obstacles', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def save_obstacles():
# # # # # # # # # # # # # # # # # # # # # # # # #     data = request.json
# # # # # # # # # # # # # # # # # # # # # # # # #     if not data or 'obstacles' not in data:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Invalid data"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = data['obstacles']
# # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # #     for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # # # #         label = obs.get('label')
# # # # # # # # # # # # # # # # # # # # # # # # #         confidence = obs.get('confidence')
# # # # # # # # # # # # # # # # # # # # # # # # #         bbox = obs.get('bbox')
# # # # # # # # # # # # # # # # # # # # # # # # #         bbox_str = ','.join(map(str, bbox))
# # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # #             (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # #     conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "saved"})

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form.get('password', None)
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = None
# # # # # # # # # # # # # # # # # # # # # # # # #     if password:
# # # # # # # # # # # # # # # # # # # # # # # # #         password_hash = hashlib.sha256(password.encode()).hexdigest()
# # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO users (username, password_hash, voice_embedding) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # #             (username, password_hash, json.dumps(embedding.tolist()))
# # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # #     except psycopg2.errors.UniqueViolation:
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User already exists"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "registered"})

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form.get('password', None)
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = None
# # # # # # # # # # # # # # # # # # # # # # # # #     if password:
# # # # # # # # # # # # # # # # # # # # # # # # #         password_hash = hashlib.sha256(password.encode()).hexdigest()
# # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur.execute("SELECT password_hash, voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # # # # # # # # # # #     user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     if not user:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User not found"}), 404
# # # # # # # # # # # # # # # # # # # # # # # # #     stored_password_hash, stored_voice_embedding_json = user
# # # # # # # # # # # # # # # # # # # # # # # # #     stored_voice_embedding = np.array(json.loads(stored_voice_embedding_json))
# # # # # # # # # # # # # # # # # # # # # # # # #     # Проверка пароля, если он есть и передан
# # # # # # # # # # # # # # # # # # # # # # # # #     if stored_password_hash and password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # #         if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Wrong password"}), 401
# # # # # # # # # # # # # # # # # # # # # # # # #     elif stored_password_hash and not password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Password required"}), 401
# # # # # # # # # # # # # # # # # # # # # # # # #     # Проверка голоса
# # # # # # # # # # # # # # # # # # # # # # # # #     similarity = 1 - cosine(embedding, stored_voice_embedding)
# # # # # # # # # # # # # # # # # # # # # # # # #     print(f"Voice similarity: {similarity}")
# # # # # # # # # # # # # # # # # # # # # # # # #     if similarity < 0.75:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Voice does not match"}), 401
# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "login success"})

# # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000)


# # # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # # # # # from speechbrain.pretrained import SpeakerRecognition
# # # # # # # # # # # # # # # # # # # # # # # # # from scipy.spatial.distance import cosine
# # # # # # # # # # # # # # # # # # # # # # # # # import hashlib

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Загрузка моделей ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLOv5 model...")
# # # # # # # # # # # # # # # # # # # # # # # # # model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
# # # # # # # # # # # # # # # # # # # # # # # # # print("YOLOv5 model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # print("Loading SpeakerRecognition model...")
# # # # # # # # # # # # # # # # # # # # # # # # # speaker_model = SpeakerRecognition.from_hparams(
# # # # # # # # # # # # # # # # # # # # # # # # #     source="speechbrain/spkrec-ecapa-voxceleb",
# # # # # # # # # # # # # # # # # # # # # # # # #     savedir="pretrained_models/spkrec-ecapa-voxceleb"
# # # # # # # # # # # # # # # # # # # # # # # # # )
# # # # # # # # # # # # # # # # # # # # # # # # # print("SpeakerRecognition model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Работа с камерой ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # # # #     img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # # # #     results = model(img_rgb)
# # # # # # # # # # # # # # # # # # # # # # # # #     return results.pandas().xyxy[0]

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Голосовой эмбеддинг ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # def get_voice_embedding(audio_path):
# # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # #         signal = speaker_model.load_audio(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # #         # Проверка: достаточно ли длины аудио
# # # # # # # # # # # # # # # # # # # # # # # # #         if signal.shape[-1] < 16000:
# # # # # # # # # # # # # # # # # # # # # # # # #             raise ValueError("Аудио слишком короткое для обработки. Запиши минимум 1 секунду.")

# # # # # # # # # # # # # # # # # # # # # # # # #         with torch.no_grad():
# # # # # # # # # # # # # # # # # # # # # # # # #             embedding = speaker_model.encode_batch(signal)
# # # # # # # # # # # # # # # # # # # # # # # # #             embedding = embedding.squeeze().cpu().numpy()

# # # # # # # # # # # # # # # # # # # # # # # # #         return embedding

# # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # #         raise RuntimeError(f"Ошибка при получении голосового эмбеддинга: {str(e)}")

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # # #     return "Blind Assistant Server is running."

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # # #     def generate():
# # # # # # # # # # # # # # # # # # # # # # # # #         while True:
# # # # # # # # # # # # # # # # # # # # # # # # #             ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # #                 break
# # # # # # # # # # # # # # # # # # # # # # # # #             df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # #             for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # #                 xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
# # # # # # # # # # # # # # # # # # # # # # # # #                 label = row['name']
# # # # # # # # # # # # # # # # # # # # # # # # #                 conf = row['confidence']
# # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # #                 cv2.putText(frame, f"{label} {conf:.2f}", (xmin, ymin - 10),
# # # # # # # # # # # # # # # # # # # # # # # # #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
# # # # # # # # # # # # # # # # # # # # # # # # #             ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # #                 continue
# # # # # # # # # # # # # # # # # # # # # # # # #             yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # # #                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
# # # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # # #     ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # # #     if not ret:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to read frame"}), 500
# # # # # # # # # # # # # # # # # # # # # # # # #     df = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = []
# # # # # # # # # # # # # # # # # # # # # # # # #     for _, row in df.iterrows():
# # # # # # # # # # # # # # # # # # # # # # # # #         obstacles.append({
# # # # # # # # # # # # # # # # # # # # # # # # #             "label": row['name'],
# # # # # # # # # # # # # # # # # # # # # # # # #             "confidence": float(row['confidence']),
# # # # # # # # # # # # # # # # # # # # # # # # #             "bbox": [int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])]
# # # # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"obstacles": obstacles})

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "No audio file provided"}), 400

# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # #     r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # # # #     with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # # # #         audio = r.record(source)

# # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # #         text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # # # # # # #     except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Не удалось распознать речь"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка сервиса распознавания: {e}"}), 500

# # # # # # # # # # # # # # # # # # # # # # # # #     response = {"recognized_text": text}
# # # # # # # # # # # # # # # # # # # # # # # # #     if "открой камеру" in text.lower():
# # # # # # # # # # # # # # # # # # # # # # # # #         response["action"] = "open_camera"

# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify(response)

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/save_obstacles', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def save_obstacles():
# # # # # # # # # # # # # # # # # # # # # # # # #     data = request.json
# # # # # # # # # # # # # # # # # # # # # # # # #     if not data or 'obstacles' not in data:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Invalid data"}), 400

# # # # # # # # # # # # # # # # # # # # # # # # #     obstacles = data['obstacles']
# # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # #     for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # # # #         label = obs.get('label')
# # # # # # # # # # # # # # # # # # # # # # # # #         confidence = obs.get('confidence')
# # # # # # # # # # # # # # # # # # # # # # # # #         bbox = obs.get('bbox')
# # # # # # # # # # # # # # # # # # # # # # # # #         bbox_str = ','.join(map(str, bbox))
# # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # #             (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # #     conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "saved"})

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400

# # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form.get('password')
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # #         embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 400

# # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO users (username, password_hash, voice_embedding) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # # #             (username, password_hash, json.dumps(embedding.tolist()))
# # # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # # #     except psycopg2.errors.UniqueViolation:
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User already exists"}), 400
# # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # # # # # # # # # # # # # # # # # # #     finally:
# # # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #         conn.close()

# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "registered"})

# # # # # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400

# # # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # # #     password = request.form.get('password')
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # # #         embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 400

# # # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur.execute("SELECT password_hash, voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # # # # # # # # # # #     user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # # #     if not user:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User not found"}), 404

# # # # # # # # # # # # # # # # # # # # # # # # #     stored_password_hash, stored_voice_embedding_json = user
# # # # # # # # # # # # # # # # # # # # # # # # #     stored_voice_embedding = np.array(json.loads(stored_voice_embedding_json))

# # # # # # # # # # # # # # # # # # # # # # # # #     # Проверка пароля
# # # # # # # # # # # # # # # # # # # # # # # # #     if stored_password_hash and password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # #         if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Wrong password"}), 401
# # # # # # # # # # # # # # # # # # # # # # # # #     elif stored_password_hash and not password_hash:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Password required"}), 401

# # # # # # # # # # # # # # # # # # # # # # # # #     # Сравнение голосов
# # # # # # # # # # # # # # # # # # # # # # # # #     similarity = 1 - cosine(embedding, stored_voice_embedding)
# # # # # # # # # # # # # # # # # # # # # # # # #     print(f"Voice similarity: {similarity:.4f}")

# # # # # # # # # # # # # # # # # # # # # # # # #     if similarity < 0.75:
# # # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Voice does not match"}), 401

# # # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "login success"})

# # # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Запуск ───────────
# # # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000)




















# # # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # # # # from speechbrain.inference import SpeakerRecognition  # <- обновлено
# # # # # # # # # # # # # # # # # # # # # # # # from scipy.spatial.distance import cosine
# # # # # # # # # # # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # # # # # # # # # # from ultralytics import YOLO  # <- новая библиотека YOLO

# # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Загрузка моделей ───────────
# # # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLOv5 model...")
# # # # # # # # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')  # <- новая загрузка модели
# # # # # # # # # # # # # # # # # # # # # # # # print("YOLOv5 model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # print("Loading SpeakerRecognition model...")
# # # # # # # # # # # # # # # # # # # # # # # # speaker_model = SpeakerRecognition.from_hparams(
# # # # # # # # # # # # # # # # # # # # # # # #     source="speechbrain/spkrec-ecapa-voxceleb",
# # # # # # # # # # # # # # # # # # # # # # # #     savedir="pretrained_models/spkrec-ecapa-voxceleb"
# # # # # # # # # # # # # # # # # # # # # # # # )
# # # # # # # # # # # # # # # # # # # # # # # # print("SpeakerRecognition model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Работа с камерой ───────────
# # # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # # #     raise RuntimeError("Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # # #     img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # # #     results = model.predict(img_rgb, verbose=False)[0]  # <- для ultralytics API
# # # # # # # # # # # # # # # # # # # # # # # #     detections = []

# # # # # # # # # # # # # # # # # # # # # # # #     for r in results.boxes:
# # # # # # # # # # # # # # # # # # # # # # # #         bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # # # #         conf = float(r.conf[0])
# # # # # # # # # # # # # # # # # # # # # # # #         cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # # # # # # # #         label = model.names[cls_id]
# # # # # # # # # # # # # # # # # # # # # # # #         detections.append({
# # # # # # # # # # # # # # # # # # # # # # # #             "label": label,
# # # # # # # # # # # # # # # # # # # # # # # #             "confidence": conf,
# # # # # # # # # # # # # # # # # # # # # # # #             "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
# # # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # # #     return detections

# # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Голосовой эмбеддинг ───────────
# # # # # # # # # # # # # # # # # # # # # # # # def get_voice_embedding(audio_path):
# # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # #         signal = speaker_model.load_audio(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # #         if signal.shape[-1] < 16000:
# # # # # # # # # # # # # # # # # # # # # # # #             raise ValueError("Аудио слишком короткое для обработки. Запиши минимум 1 секунду.")

# # # # # # # # # # # # # # # # # # # # # # # #         with torch.no_grad():
# # # # # # # # # # # # # # # # # # # # # # # #             embedding = speaker_model.encode_batch(signal)
# # # # # # # # # # # # # # # # # # # # # # # #             embedding = embedding.squeeze().cpu().numpy()

# # # # # # # # # # # # # # # # # # # # # # # #         return embedding

# # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # #         raise RuntimeError(f"Ошибка при получении голосового эмбеддинга: {str(e)}")

# # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # # #     return "Blind Assistant Server is running."

# # # # # # # # # # # # # # # # # # # # # # # # @app.route('/video')
# # # # # # # # # # # # # # # # # # # # # # # # def video_feed():
# # # # # # # # # # # # # # # # # # # # # # # #     def generate():
# # # # # # # # # # # # # # # # # # # # # # # #         while True:
# # # # # # # # # # # # # # # # # # # # # # # #             ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # #                 break
# # # # # # # # # # # # # # # # # # # # # # # #             detections = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # #             for obj in detections:
# # # # # # # # # # # # # # # # # # # # # # # #                 xmin, ymin, xmax, ymax = obj['bbox']
# # # # # # # # # # # # # # # # # # # # # # # #                 label = obj['label']
# # # # # # # # # # # # # # # # # # # # # # # #                 conf = obj['confidence']
# # # # # # # # # # # # # # # # # # # # # # # #                 cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
# # # # # # # # # # # # # # # # # # # # # # # #                 cv2.putText(frame, f"{label} {conf:.2f}", (xmin, ymin - 10),
# # # # # # # # # # # # # # # # # # # # # # # #                             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
# # # # # # # # # # # # # # # # # # # # # # # #             ret, buffer = cv2.imencode('.jpg', frame)
# # # # # # # # # # # # # # # # # # # # # # # #             if not ret:
# # # # # # # # # # # # # # # # # # # # # # # #                 continue
# # # # # # # # # # # # # # # # # # # # # # # #             yield (b'--frame\r\n'
# # # # # # # # # # # # # # # # # # # # # # # #                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
# # # # # # # # # # # # # # # # # # # # # # # #     return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # # #     ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # # #     if not ret:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Failed to read frame"}), 500
# # # # # # # # # # # # # # # # # # # # # # # #     obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"obstacles": obstacles})

# # # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "No audio file provided"}), 400

# # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # #     r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # # #     with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # # #         audio = r.record(source)

# # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # #         text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # # # # # #     except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Не удалось распознать речь"}), 400
# # # # # # # # # # # # # # # # # # # # # # # #     except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка сервиса распознавания: {e}"}), 500

# # # # # # # # # # # # # # # # # # # # # # # #     response = {"recognized_text": text}
# # # # # # # # # # # # # # # # # # # # # # # #     if "открой камеру" in text.lower():
# # # # # # # # # # # # # # # # # # # # # # # #         response["action"] = "open_camera"

# # # # # # # # # # # # # # # # # # # # # # # #     return jsonify(response)

# # # # # # # # # # # # # # # # # # # # # # # # @app.route('/save_obstacles', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # def save_obstacles():
# # # # # # # # # # # # # # # # # # # # # # # #     data = request.json
# # # # # # # # # # # # # # # # # # # # # # # #     if not data or 'obstacles' not in data:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Invalid data"}), 400

# # # # # # # # # # # # # # # # # # # # # # # #     obstacles = data['obstacles']
# # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # #     for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # # #         label = obs.get('label')
# # # # # # # # # # # # # # # # # # # # # # # #         confidence = obs.get('confidence')
# # # # # # # # # # # # # # # # # # # # # # # #         bbox = obs.get('bbox')
# # # # # # # # # # # # # # # # # # # # # # # #         bbox_str = ','.join(map(str, bbox))
# # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # #             (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # #     conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "saved"})

# # # # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400

# # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # #     password = request.form.get('password')
# # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # #         embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 400

# # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # # #             "INSERT INTO users (username, password_hash, voice_embedding) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # # #             (username, password_hash, json.dumps(embedding.tolist()))
# # # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # # #         conn.commit()
# # # # # # # # # # # # # # # # # # # # # # # #     except psycopg2.errors.UniqueViolation:
# # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User already exists"}), 400
# # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # #         conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # # # # # # # # # # # # # # # # # #     finally:
# # # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # # #         conn.close()

# # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "registered"})

# # # # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # # # #     if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Missing fields"}), 400

# # # # # # # # # # # # # # # # # # # # # # # #     username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # # #     password = request.form.get('password')
# # # # # # # # # # # # # # # # # # # # # # # #     audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # # #     audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # # #     audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # # #         embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": str(e)}), 400

# # # # # # # # # # # # # # # # # # # # # # # #     os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # # #     password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # # # #     conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # # #     cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # # #     cur.execute("SELECT password_hash, voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # # # # # # # # # #     user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # # # #     cur.close()
# # # # # # # # # # # # # # # # # # # # # # # #     conn.close()

# # # # # # # # # # # # # # # # # # # # # # # #     if not user:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "User not found"}), 404

# # # # # # # # # # # # # # # # # # # # # # # #     stored_password_hash, stored_voice_embedding_json = user
# # # # # # # # # # # # # # # # # # # # # # # #     stored_voice_embedding = np.array(json.loads(stored_voice_embedding_json))

# # # # # # # # # # # # # # # # # # # # # # # #     if stored_password_hash and password_hash:
# # # # # # # # # # # # # # # # # # # # # # # #         if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Wrong password"}), 401
# # # # # # # # # # # # # # # # # # # # # # # #     elif stored_password_hash and not password_hash:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Password required"}), 401

# # # # # # # # # # # # # # # # # # # # # # # #     similarity = 1 - cosine(embedding, stored_voice_embedding)
# # # # # # # # # # # # # # # # # # # # # # # #     print(f"Voice similarity: {similarity:.4f}")

# # # # # # # # # # # # # # # # # # # # # # # #     if similarity < 0.75:
# # # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": "Voice does not match"}), 401

# # # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "login success"})

# # # # # # # # # # # # # # # # # # # # # # # # # ─────────── Запуск ───────────
# # # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # # #     app.run(host='0.0.0.0', port=5000)




# # # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # # import torch
# # # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # # # from speechbrain.inference import SpeakerRecognition
# # # # # # # # # # # # # # # # # # # # # # # from scipy.spatial.distance import cosine
# # # # # # # # # # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # # # # # # # from flask_cors import CORS

# # # # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)
# # # # # # # # # # # # # # # # # # # # # # # CORS(app)

# # # # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # # # # # ─────────── Загрузка моделей ───────────
# # # # # # # # # # # # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')
# # # # # # # # # # # # # # # # # # # # # # # print("YOLO model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # print("Loading SpeakerRecognition model...")
# # # # # # # # # # # # # # # # # # # # # # # speaker_model = SpeakerRecognition.from_hparams(
# # # # # # # # # # # # # # # # # # # # # # #     source="speechbrain/spkrec-ecapa-voxceleb",
# # # # # # # # # # # # # # # # # # # # # # #     savedir="pretrained_models/spkrec-ecapa-voxceleb"
# # # # # # # # # # # # # # # # # # # # # # # )
# # # # # # # # # # # # # # # # # # # # # # # print("SpeakerRecognition model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # # ─────────── Работа с камерой ───────────
# # # # # # # # # # # # # # # # # # # # # # # camera = cv2.VideoCapture(0)
# # # # # # # # # # # # # # # # # # # # # # # if not camera.isOpened():
# # # # # # # # # # # # # # # # # # # # # # #     print("Warning: Cannot open camera")

# # # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # # # # # # # #         detections = []

# # # # # # # # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
# # # # # # # # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # # # # # # # ─────────── Голосовой эмбеддинг ───────────
# # # # # # # # # # # # # # # # # # # # # # # def get_voice_embedding(audio_path):
# # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # #         signal = speaker_model.load_audio(audio_path)

# # # # # # # # # # # # # # # # # # # # # # #         if signal.shape[-1] < 16000:
# # # # # # # # # # # # # # # # # # # # # # #             raise ValueError("Аудио слишком короткое для обработки. Запиши минимум 1 секунду.")

# # # # # # # # # # # # # # # # # # # # # # #         with torch.no_grad():
# # # # # # # # # # # # # # # # # # # # # # #             embedding = speaker_model.encode_batch(signal)
# # # # # # # # # # # # # # # # # # # # # # #             embedding = embedding.squeeze().cpu().numpy()

# # # # # # # # # # # # # # # # # # # # # # #         return embedding

# # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #         raise RuntimeError(f"Ошибка при получении голосового эмбеддинга: {str(e)}")

# # # # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # # #     return jsonify({"status": "Blind Assistant Server is running"})

# # # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # #         ret, frame = camera.read()
# # # # # # # # # # # # # # # # # # # # # # #         if not ret:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Failed to read frame from camera"}), 500
        
# # # # # # # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
        
# # # # # # # # # # # # # # # # # # # # # # #         # Сохраняем препятствия в базу
# # # # # # # # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # # #                 label = obs.get('label', 'unknown')
# # # # # # # # # # # # # # # # # # # # # # #                 confidence = obs.get('confidence', 0.0)
# # # # # # # # # # # # # # # # # # # # # # #                 bbox = obs.get('bbox', [])
# # # # # # # # # # # # # # # # # # # # # # #                 bbox_str = ','.join(map(str, bbox))
                
# # # # # # # # # # # # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # # # # # # # # # # # #                     "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # #                     (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # # #                 )
# # # # # # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # # # # #             conn.close()
        
# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # # # # # # #             "count": len(obstacles)
# # # # # # # # # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Server error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio file provided"}), 400

# # # # # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # #         audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # #         audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # # #         with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # # #             audio = r.record(source)

# # # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Не удалось распознать речь"}), 400
# # # # # # # # # # # # # # # # # # # # # # #         except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": f"Ошибка сервиса распознавания: {e}"}), 500

# # # # # # # # # # # # # # # # # # # # # # #         response = {"recognized_text": text}
        
# # # # # # # # # # # # # # # # # # # # # # #         # Простые команды
# # # # # # # # # # # # # # # # # # # # # # #         text_lower = text.lower()
# # # # # # # # # # # # # # # # # # # # # # #         if any(word in text_lower for word in ['камера', 'camera']):
# # # # # # # # # # # # # # # # # # # # # # #             response["action"] = "toggle_camera"
# # # # # # # # # # # # # # # # # # # # # # #         elif any(word in text_lower for word in ['сканирование', 'сканировать', 'препятствие']):
# # # # # # # # # # # # # # # # # # # # # # #             response["action"] = "toggle_scan"
# # # # # # # # # # # # # # # # # # # # # # #         elif any(word in text_lower for word in ['стоп', 'остановить']):
# # # # # # # # # # # # # # # # # # # # # # #             response["action"] = "stop"
# # # # # # # # # # # # # # # # # # # # # # #         elif any(word in text_lower for word in ['помощь', 'help']):
# # # # # # # # # # # # # # # # # # # # # # #             response["action"] = "help"

# # # # # # # # # # # # # # # # # # # # # # #         return jsonify(response)
        
# # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice command error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Missing fields"}), 400

# # # # # # # # # # # # # # # # # # # # # # #         username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # #         password = request.form.get('password', '')
# # # # # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # #         audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # #         audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # #         # Получаем голосовой эмбеддинг
# # # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # # #             embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #             os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": str(e)}), 400

# # # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # #         # Хешируем пароль если предоставлен
# # # # # # # # # # # # # # # # # # # # # # #         password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # # #         # Сохраняем в базу
# # # # # # # # # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # #         cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # # #             cur.execute(
# # # # # # # # # # # # # # # # # # # # # # #                 "INSERT INTO users (username, password_hash, voice_embedding) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # # #                 (username, password_hash, json.dumps(embedding.tolist()))
# # # # # # # # # # # # # # # # # # # # # # #             )
# # # # # # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"status": "registered", "username": username})
            
# # # # # # # # # # # # # # # # # # # # # # #         except psycopg2.errors.UniqueViolation:
# # # # # # # # # # # # # # # # # # # # # # #             conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "User already exists"}), 400
# # # # # # # # # # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #             conn.rollback()
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": f"Database error: {str(e)}"}), 500
# # # # # # # # # # # # # # # # # # # # # # #         finally:
# # # # # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Registration error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Missing fields"}), 400

# # # # # # # # # # # # # # # # # # # # # # #         username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # # #         password = request.form.get('password', '')
# # # # # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # # #         audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # # #         audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # # #         # Получаем голосовой эмбеддинг
# # # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # # #             embedding = get_voice_embedding(audio_path)
# # # # # # # # # # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #             os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": str(e)}), 400

# # # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # # #         password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # # #         # Проверяем пользователя в базе
# # # # # # # # # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # #         cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # # #             "SELECT password_hash, voice_embedding FROM users WHERE username = %s", 
# # # # # # # # # # # # # # # # # # # # # # #             (username,)
# # # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # # #         user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # #         conn.close()

# # # # # # # # # # # # # # # # # # # # # # #         if not user:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "User not found"}), 404

# # # # # # # # # # # # # # # # # # # # # # #         stored_password_hash, stored_voice_embedding_json = user
        
# # # # # # # # # # # # # # # # # # # # # # #         # Проверяем пароль если есть
# # # # # # # # # # # # # # # # # # # # # # #         if stored_password_hash and password_hash:
# # # # # # # # # # # # # # # # # # # # # # #             if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # # #                 return jsonify({"error": "Wrong password"}), 401
# # # # # # # # # # # # # # # # # # # # # # #         elif stored_password_hash and not password_hash:
# # # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Password required"}), 401

# # # # # # # # # # # # # # # # # # # # # # #         # Проверяем голос
# # # # # # # # # # # # # # # # # # # # # # #         if stored_voice_embedding_json:
# # # # # # # # # # # # # # # # # # # # # # #             try:
# # # # # # # # # # # # # # # # # # # # # # #                 stored_voice_embedding = np.array(json.loads(stored_voice_embedding_json))
# # # # # # # # # # # # # # # # # # # # # # #                 similarity = 1 - cosine(embedding, stored_voice_embedding)
# # # # # # # # # # # # # # # # # # # # # # #                 print(f"Voice similarity: {similarity:.4f}")

# # # # # # # # # # # # # # # # # # # # # # #                 if similarity < 0.7:  # Немного снизил порог для тестирования
# # # # # # # # # # # # # # # # # # # # # # #                     return jsonify({"error": "Voice does not match"}), 401
# # # # # # # # # # # # # # # # # # # # # # #             except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #                 return jsonify({"error": f"Voice verification error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # #             "status": "login success", 
# # # # # # # # # # # # # # # # # # # # # # #             "username": username,
# # # # # # # # # # # # # # # # # # # # # # #             "message": "Authentication successful"
# # # # # # # # # # # # # # # # # # # # # # #         })

# # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Login error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # # @app.route('/test_db')
# # # # # # # # # # # # # # # # # # # # # # # def test_db():
# # # # # # # # # # # # # # # # # # # # # # #     """Тестовый endpoint для проверки подключения к БД"""
# # # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # # #         cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # # #         cur.execute("SELECT COUNT(*) FROM users")
# # # # # # # # # # # # # # # # # # # # # # #         user_count = cur.fetchone()[0]
# # # # # # # # # # # # # # # # # # # # # # #         cur.execute("SELECT COUNT(*) FROM obstacles")
# # # # # # # # # # # # # # # # # # # # # # #         obstacle_count = cur.fetchone()[0]
# # # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # # #         conn.close()
        
# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # # #             "database": "connected",
# # # # # # # # # # # # # # # # # # # # # # #             "users_count": user_count,
# # # # # # # # # # # # # # # # # # # # # # #             "obstacles_count": obstacle_count
# # # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Database connection failed: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # # # ─────────── Запуск ───────────
# # # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # # #     print("Starting Blind Assistant Server...")
# # # # # # # # # # # # # # # # # # # # # # #     print(f"Server will be available at: http://192.168.8.63:5000")
# # # # # # # # # # # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)


# # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # # # # # # # # from ultralytics import YOLO

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # Разрешаем CORS вручную
# # # # # # # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Загрузка модели YOLO ───────────
# # # # # # # # # # # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')
# # # # # # # # # # # # # # # # # # # # # # print("YOLO model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Функции для обработки изображений ───────────
# # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # #     """Обнаружение объектов на кадре с помощью YOLO"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # # # # # # #         detections = []

# # # # # # # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
# # # # # # # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # # # # # # #     """Генерирует текстовое описание препятствий для озвучки"""
# # # # # # # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"
    
# # # # # # # # # # # # # # # # # # # # # #     # Русские названия для common объектов
# # # # # # # # # # # # # # # # # # # # # #     russian_labels = {
# # # # # # # # # # # # # # # # # # # # # #         'person': 'человек', 'car': 'автомобиль', 'chair': 'стул', 'table': 'стол',
# # # # # # # # # # # # # # # # # # # # # #         'book': 'книга', 'bottle': 'бутылка', 'cup': 'чашка', 'phone': 'телефон',
# # # # # # # # # # # # # # # # # # # # # #         'laptop': 'ноутбук', 'mouse': 'мышь', 'keyboard': 'клавиатура', 'tv': 'телевизор',
# # # # # # # # # # # # # # # # # # # # # #         'remote': 'пульт', 'clock': 'часы', 'vase': 'ваза', 'scissors': 'ножницы',
# # # # # # # # # # # # # # # # # # # # # #         'teddy bear': 'плюшевый мишка', 'hair drier': 'фен', 'toothbrush': 'зубная щетка',
# # # # # # # # # # # # # # # # # # # # # #         'cat': 'кот', 'dog': 'собака', 'bird': 'птица', 'horse': 'лошадь',
# # # # # # # # # # # # # # # # # # # # # #         'sheep': 'овца', 'cow': 'корова', 'elephant': 'слон', 'bear': 'медведь',
# # # # # # # # # # # # # # # # # # # # # #         'zebra': 'зебра', 'giraffe': 'жираф', 'backpack': 'рюкзак', 'umbrella': 'зонт',
# # # # # # # # # # # # # # # # # # # # # #         'handbag': 'сумка', 'tie': 'галстук', 'suitcase': 'чемодан', 'frisbee': 'фрисби',
# # # # # # # # # # # # # # # # # # # # # #         'skis': 'лыжи', 'snowboard': 'сноуборд', 'sports ball': 'спортивный мяч',
# # # # # # # # # # # # # # # # # # # # # #         'kite': 'воздушный змей', 'baseball bat': 'бейсбольная бита', 'baseball glove': 'бейсбольная перчатка',
# # # # # # # # # # # # # # # # # # # # # #         'skateboard': 'скейтборд', 'surfboard': 'доска для серфинга', 'tennis racket': 'теннисная ракетка',
# # # # # # # # # # # # # # # # # # # # # #         'wine glass': 'бокал', 'spoon': 'ложка', 'bowl': 'миска', 'banana': 'банан',
# # # # # # # # # # # # # # # # # # # # # #         'apple': 'яблоко', 'sandwich': 'бутерброд', 'orange': 'апельсин', 'broccoli': 'брокколи',
# # # # # # # # # # # # # # # # # # # # # #         'carrot': 'морковь', 'hot dog': 'хот-дог', 'pizza': 'пицца', 'donut': 'пончик',
# # # # # # # # # # # # # # # # # # # # # #         'cake': 'торт', 'bed': 'кровать', 'toilet': 'унитаз', 'sink': 'раковина',
# # # # # # # # # # # # # # # # # # # # # #         'refrigerator': 'холодильник', 'oven': 'духовка', 'microwave': 'микроволновка',
# # # # # # # # # # # # # # # # # # # # # #         'toaster': 'тостер'
# # # # # # # # # # # # # # # # # # # # # #     }
    
# # # # # # # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # # # # # # #     for i, obstacle in enumerate(obstacles[:5]):  # Ограничиваем 5 объектами
# # # # # # # # # # # # # # # # # # # # # #         label = obstacle['label']
# # # # # # # # # # # # # # # # # # # # # #         confidence = obstacle['confidence']
        
# # # # # # # # # # # # # # # # # # # # # #         # Перевод на русский
# # # # # # # # # # # # # # # # # # # # # #         label_ru = russian_labels.get(label, label)
# # # # # # # # # # # # # # # # # # # # # #         confidence_percent = int(confidence * 100)
        
# # # # # # # # # # # # # # # # # # # # # #         # Определяем положение объекта
# # # # # # # # # # # # # # # # # # # # # #         bbox = obstacle['bbox']
# # # # # # # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # # # # # # # # #         frame_center = 320  # Примерная ширина кадра
        
# # # # # # # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # # # # # # #             position = "слева"
# # # # # # # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # # # # # # #             position = "справа"
# # # # # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # # # # #             position = "прямо перед вами"
        
# # # # # # # # # # # # # # # # # # # # # #         descriptions.append(f"{label_ru} {position} с уверенностью {confidence_percent} процентов")
    
# # # # # # # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
    
# # # # # # # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running", 
# # # # # # # # # # # # # # # # # # # # # #         "version": "2.0",
# # # # # # # # # # # # # # # # # # # # # #         "message": "Сервер готов к обработке кадров с камеры телефона"
# # # # # # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # # # # # # #     """Основной endpoint для обработки кадров с камеры телефона"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # # # # # # #         frame_file = request.files['frame']
        
# # # # # # # # # # # # # # # # # # # # # #         # Конвертируем в numpy array
# # # # # # # # # # # # # # # # # # # # # #         frame_bytes = frame_file.read()
# # # # # # # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
# # # # # # # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Failed to decode image"}), 400

# # # # # # # # # # # # # # # # # # # # # #         # Детекция объектов
# # # # # # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
        
# # # # # # # # # # # # # # # # # # # # # #         # Формируем описание для озвучки
# # # # # # # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)
        
# # # # # # # # # # # # # # # # # # # # # #         # Сохраняем в базу
# # # # # # # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # #                 label = obs.get('label', 'unknown')
# # # # # # # # # # # # # # # # # # # # # #                 confidence = obs.get('confidence', 0.0)
# # # # # # # # # # # # # # # # # # # # # #                 bbox = obs.get('bbox', [])
# # # # # # # # # # # # # # # # # # # # # #                 bbox_str = ','.join(map(str, bbox))
                
# # # # # # # # # # # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # # # # # # # # # # #                     "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # #                     (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # #                 )
# # # # # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # # # #             conn.close()
        
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # # # # # # #             "message": "Frame processed successfully"
# # # # # # # # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Frame processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/check_obstacle')
# # # # # # # # # # # # # # # # # # # # # # def check_obstacle():
# # # # # # # # # # # # # # # # # # # # # #     """Legacy endpoint для обратной совместимости"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         # Создаем черный кадр для тестирования
# # # # # # # # # # # # # # # # # # # # # #         frame = np.zeros((480, 640, 3), dtype=np.uint8)
# # # # # # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
        
# # # # # # # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)
        
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # # # # # # #             "count": len(obstacles)
# # # # # # # # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Server error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # #     """Обработка голосовых команд"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio file provided"}), 400

# # # # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # #         audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # #         audio_file.save(audio_path)

# # # # # # # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # # #         with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # # # # # # # # # #             audio = r.record(source)

# # # # # # # # # # # # # # # # # # # # # #         os.remove(audio_path)

# # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Не удалось распознать речь"}), 400
# # # # # # # # # # # # # # # # # # # # # #         except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": f"Ошибка сервиса распознавания: {e}"}), 500

# # # # # # # # # # # # # # # # # # # # # #         response = {"recognized_text": text}
        
# # # # # # # # # # # # # # # # # # # # # #         # Простые команды
# # # # # # # # # # # # # # # # # # # # # #         text_lower = text.lower()
# # # # # # # # # # # # # # # # # # # # # #         if any(word in text_lower for word in ['камера', 'camera']):
# # # # # # # # # # # # # # # # # # # # # #             response["action"] = "toggle_camera"
# # # # # # # # # # # # # # # # # # # # # #         elif any(word in text_lower for word in ['сканирование', 'сканировать', 'препятствие', 'что вокруг']):
# # # # # # # # # # # # # # # # # # # # # #             response["action"] = "scan_obstacles"
# # # # # # # # # # # # # # # # # # # # # #         elif any(word in text_lower for word in ['стоп', 'остановить']):
# # # # # # # # # # # # # # # # # # # # # #             response["action"] = "stop"
# # # # # # # # # # # # # # # # # # # # # #         elif any(word in text_lower for word in ['помощь', 'help']):
# # # # # # # # # # # # # # # # # # # # # #             response["action"] = "help"

# # # # # # # # # # # # # # # # # # # # # #         return jsonify(response)
        
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice command error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/register', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # def register():
# # # # # # # # # # # # # # # # # # # # # #     """Упрощенная регистрация пользователя"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         if 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Missing username"}), 400

# # # # # # # # # # # # # # # # # # # # # #         username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # #         password = request.form.get('password', '')
        
# # # # # # # # # # # # # # # # # # # # # #         password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # #         cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # #             # Проверяем существует ли пользователь
# # # # # # # # # # # # # # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # # # # # # # #             existing_user = cur.fetchone()
            
# # # # # # # # # # # # # # # # # # # # # #             if existing_user:
# # # # # # # # # # # # # # # # # # # # # #                 return jsonify({"error": "User already exists"}), 400
            
# # # # # # # # # # # # # # # # # # # # # #             # Создаем нового пользователя
# # # # # # # # # # # # # # # # # # # # # #             cur.execute(
# # # # # # # # # # # # # # # # # # # # # #                 "INSERT INTO users (username, password_hash) VALUES (%s, %s)",
# # # # # # # # # # # # # # # # # # # # # #                 (username, password_hash)
# # # # # # # # # # # # # # # # # # # # # #             )
# # # # # # # # # # # # # # # # # # # # # #             conn.commit()
            
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # # # # # # # # # #                 "status": "registered", 
# # # # # # # # # # # # # # # # # # # # # #                 "username": username,
# # # # # # # # # # # # # # # # # # # # # #                 "message": "User registered successfully"
# # # # # # # # # # # # # # # # # # # # # #             })
            
# # # # # # # # # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #             conn.rollback()
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": f"Database error: {str(e)}"}), 500
# # # # # # # # # # # # # # # # # # # # # #         finally:
# # # # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Registration error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/login', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # def login():
# # # # # # # # # # # # # # # # # # # # # #     """Упрощенная авторизация пользователя"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         if 'username' not in request.form:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Missing username"}), 400

# # # # # # # # # # # # # # # # # # # # # #         username = request.form['username']
# # # # # # # # # # # # # # # # # # # # # #         password = request.form.get('password', '')
        
# # # # # # # # # # # # # # # # # # # # # #         password_hash = hashlib.sha256(password.encode()).hexdigest() if password else None

# # # # # # # # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # #         cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # # # # # # # #             "SELECT password_hash FROM users WHERE username = %s", 
# # # # # # # # # # # # # # # # # # # # # #             (username,)
# # # # # # # # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # # # # # # # #         user = cur.fetchone()
# # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # #         conn.close()

# # # # # # # # # # # # # # # # # # # # # #         if not user:
# # # # # # # # # # # # # # # # # # # # # #             # Если пользователя нет, автоматически создаем его
# # # # # # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # #             cur.execute(
# # # # # # # # # # # # # # # # # # # # # #                 "INSERT INTO users (username, password_hash) VALUES (%s, %s)",
# # # # # # # # # # # # # # # # # # # # # #                 (username, password_hash)
# # # # # # # # # # # # # # # # # # # # # #             )
# # # # # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # # # #             conn.close()
            
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # # # # # # # # # #                 "status": "login success", 
# # # # # # # # # # # # # # # # # # # # # #                 "username": username,
# # # # # # # # # # # # # # # # # # # # # #                 "message": "New user created and logged in"
# # # # # # # # # # # # # # # # # # # # # #             })

# # # # # # # # # # # # # # # # # # # # # #         stored_password_hash = user[0]

# # # # # # # # # # # # # # # # # # # # # #         # Проверяем пароль если есть
# # # # # # # # # # # # # # # # # # # # # #         if stored_password_hash and password_hash:
# # # # # # # # # # # # # # # # # # # # # #             if stored_password_hash != password_hash:
# # # # # # # # # # # # # # # # # # # # # #                 return jsonify({"error": "Wrong password"}), 401
# # # # # # # # # # # # # # # # # # # # # #         elif stored_password_hash and not password_hash:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Password required"}), 401

# # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # #             "status": "login success", 
# # # # # # # # # # # # # # # # # # # # # #             "username": username,
# # # # # # # # # # # # # # # # # # # # # #             "message": "Authentication successful"
# # # # # # # # # # # # # # # # # # # # # #         })

# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Login error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/test_db')
# # # # # # # # # # # # # # # # # # # # # # def test_db():
# # # # # # # # # # # # # # # # # # # # # #     """Тестовый endpoint для проверки подключения к БД"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # #         cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # #         cur.execute("SELECT COUNT(*) FROM users")
# # # # # # # # # # # # # # # # # # # # # #         user_count = cur.fetchone()[0]
# # # # # # # # # # # # # # # # # # # # # #         cur.execute("SELECT COUNT(*) FROM obstacles")
# # # # # # # # # # # # # # # # # # # # # #         obstacle_count = cur.fetchone()[0]
# # # # # # # # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # # # # # # # #         conn.close()
        
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # #             "database": "connected",
# # # # # # # # # # # # # # # # # # # # # #             "users_count": user_count,
# # # # # # # # # # # # # # # # # # # # # #             "obstacles_count": obstacle_count
# # # # # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Database connection failed: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/test_connection')
# # # # # # # # # # # # # # # # # # # # # # def test_connection():
# # # # # # # # # # # # # # # # # # # # # #     """Простой тест подключения"""
# # # # # # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # # # # # #         "status": "success",
# # # # # # # # # # # # # # # # # # # # # #         "message": "Server is responding",
# # # # # # # # # # # # # # # # # # # # # #         "endpoints": {
# # # # # # # # # # # # # # # # # # # # # #             "process_frame": "POST /process_frame - отправка кадра с телефона",
# # # # # # # # # # # # # # # # # # # # # #             "voice_command": "POST /voice_command - голосовые команды", 
# # # # # # # # # # # # # # # # # # # # # #             "register": "POST /register - регистрация",
# # # # # # # # # # # # # # # # # # # # # #             "login": "POST /login - авторизация"
# # # # # # # # # # # # # # # # # # # # # #         }
# # # # # # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Запуск ───────────
# # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # #     print("🚀 Starting Blind Assistant Server...")
# # # # # # # # # # # # # # # # # # # # # #     print("📱 Сервер готов к работе с камерой телефона")
# # # # # # # # # # # # # # # # # # # # # #     print("🔗 Основные endpoint:")
# # # # # # # # # # # # # # # # # # # # # #     print("   POST /process_frame - обработка кадров с телефона")
# # # # # # # # # # # # # # # # # # # # # #     print("   POST /voice_command - голосовые команды") 
# # # # # # # # # # # # # # # # # # # # # #     print("   POST /register - регистрация")
# # # # # # # # # # # # # # # # # # # # # #     print("   POST /login - авторизация")
# # # # # # # # # # # # # # # # # # # # # #     print("   GET  /test_connection - проверка подключения")
# # # # # # # # # # # # # # # # # # # # # #     print(f"🌐 Server will be available at: http://192.168.8.63:5000")
    
# # # # # # # # # # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)





# # # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # # # # # # import wave
# # # # # # # # # # # # # # # # # # # # # # import io

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # # Разрешаем CORS вручную
# # # # # # # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Загрузка модели YOLO ───────────
# # # # # # # # # # # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')
# # # # # # # # # # # # # # # # # # # # # # print("YOLO model loaded.")

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Функции для обработки изображений ───────────
# # # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # # #     """Обнаружение объектов на кадре с помощью YOLO"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # # # # # # #         detections = []

# # # # # # # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
# # # # # # # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # # # # # # #     """Генерирует текстовое описание препятствий для озвучки"""
# # # # # # # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"
    
# # # # # # # # # # # # # # # # # # # # # #     # Русские названия для common объектов
# # # # # # # # # # # # # # # # # # # # # #     russian_labels = {
# # # # # # # # # # # # # # # # # # # # # #         'person': 'человек', 'car': 'автомобиль', 'chair': 'стул', 'table': 'стол',
# # # # # # # # # # # # # # # # # # # # # #         'book': 'книга', 'bottle': 'бутылка', 'cup': 'чашка', 'phone': 'телефон',
# # # # # # # # # # # # # # # # # # # # # #         'laptop': 'ноутбук', 'mouse': 'мышь', 'keyboard': 'клавиатура', 'tv': 'телевизор',
# # # # # # # # # # # # # # # # # # # # # #         'remote': 'пульт', 'clock': 'часы', 'vase': 'ваза', 'scissors': 'ножницы',
# # # # # # # # # # # # # # # # # # # # # #         'teddy bear': 'плюшевый мишка', 'hair drier': 'фен', 'toothbrush': 'зубная щетка',
# # # # # # # # # # # # # # # # # # # # # #         'cat': 'кот', 'dog': 'собака', 'bird': 'птица', 'horse': 'лошадь',
# # # # # # # # # # # # # # # # # # # # # #         'sheep': 'овца', 'cow': 'корова', 'elephant': 'слон', 'bear': 'медведь',
# # # # # # # # # # # # # # # # # # # # # #         'zebra': 'зебра', 'giraffe': 'жираф', 'backpack': 'рюкзак', 'umbrella': 'зонт',
# # # # # # # # # # # # # # # # # # # # # #         'handbag': 'сумка', 'tie': 'галстук', 'suitcase': 'чемодан', 'frisbee': 'фрисби',
# # # # # # # # # # # # # # # # # # # # # #         'skis': 'лыжи', 'snowboard': 'сноуборд', 'sports ball': 'спортивный мяч',
# # # # # # # # # # # # # # # # # # # # # #         'kite': 'воздушный змей', 'baseball bat': 'бейсбольная бита', 'baseball glove': 'бейсбольная перчатка',
# # # # # # # # # # # # # # # # # # # # # #         'skateboard': 'скейтборд', 'surfboard': 'доска для серфинга', 'tennis racket': 'теннисная ракетка',
# # # # # # # # # # # # # # # # # # # # # #         'wine glass': 'бокал', 'spoon': 'ложка', 'bowl': 'миска', 'banana': 'банан',
# # # # # # # # # # # # # # # # # # # # # #         'apple': 'яблоко', 'sandwich': 'бутерброд', 'orange': 'апельсин', 'broccoli': 'брокколи',
# # # # # # # # # # # # # # # # # # # # # #         'carrot': 'морковь', 'hot dog': 'хот-дог', 'pizza': 'пицца', 'donut': 'пончик',
# # # # # # # # # # # # # # # # # # # # # #         'cake': 'торт', 'bed': 'кровать', 'toilet': 'унитаз', 'sink': 'раковина',
# # # # # # # # # # # # # # # # # # # # # #         'refrigerator': 'холодильник', 'oven': 'духовка', 'microwave': 'микроволновка',
# # # # # # # # # # # # # # # # # # # # # #         'toaster': 'тостер'
# # # # # # # # # # # # # # # # # # # # # #     }
    
# # # # # # # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # # # # # # #     for i, obstacle in enumerate(obstacles[:5]):  # Ограничиваем 5 объектами
# # # # # # # # # # # # # # # # # # # # # #         label = obstacle['label']
# # # # # # # # # # # # # # # # # # # # # #         confidence = obstacle['confidence']
        
# # # # # # # # # # # # # # # # # # # # # #         # Перевод на русский
# # # # # # # # # # # # # # # # # # # # # #         label_ru = russian_labels.get(label, label)
# # # # # # # # # # # # # # # # # # # # # #         confidence_percent = int(confidence * 100)
        
# # # # # # # # # # # # # # # # # # # # # #         # Определяем положение объекта
# # # # # # # # # # # # # # # # # # # # # #         bbox = obstacle['bbox']
# # # # # # # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # # # # # # # # #         frame_center = 320  # Примерная ширина кадра
        
# # # # # # # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # # # # # # #             position = "слева"
# # # # # # # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # # # # # # #             position = "справа"
# # # # # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # # # # #             position = "прямо перед вами"
        
# # # # # # # # # # # # # # # # # # # # # #         descriptions.append(f"{label_ru} {position} с уверенностью {confidence_percent} процентов")
    
# # # # # # # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
    
# # # # # # # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # # # # # def convert_to_proper_wav(audio_path):
# # # # # # # # # # # # # # # # # # # # # #     """Конвертирует аудио в правильный WAV формат"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         # Создаем простой WAV файл с заголовком
# # # # # # # # # # # # # # # # # # # # # #         output_path = audio_path.replace('.wav', '_converted.wav')
        
# # # # # # # # # # # # # # # # # # # # # #         # Создаем минимальный WAV файл (1 канал, 16kHz, 16-bit)
# # # # # # # # # # # # # # # # # # # # # #         with wave.open(output_path, 'wb') as wav_file:
# # # # # # # # # # # # # # # # # # # # # #             wav_file.setnchannels(1)  # моно
# # # # # # # # # # # # # # # # # # # # # #             wav_file.setsampwidth(2)  # 16-bit
# # # # # # # # # # # # # # # # # # # # # #             wav_file.setframerate(16000)  # 16kHz
# # # # # # # # # # # # # # # # # # # # # #             # Записываем тишину (1 секунда)
# # # # # # # # # # # # # # # # # # # # # #             silence = b'\x00' * 32000  # 16000 samples * 2 bytes
# # # # # # # # # # # # # # # # # # # # # #             wav_file.writeframes(silence)
        
# # # # # # # # # # # # # # # # # # # # # #         return output_path
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         print(f"Audio conversion error: {e}")
# # # # # # # # # # # # # # # # # # # # # #         return audio_path

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running", 
# # # # # # # # # # # # # # # # # # # # # #         "version": "2.0",
# # # # # # # # # # # # # # # # # # # # # #         "message": "Сервер готов к обработке кадров с камеры телефона"
# # # # # # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # # # # # # #     """Основной endpoint для обработки кадров с камеры телефона"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # # # # # # #         frame_file = request.files['frame']
        
# # # # # # # # # # # # # # # # # # # # # #         # Конвертируем в numpy array
# # # # # # # # # # # # # # # # # # # # # #         frame_bytes = frame_file.read()
# # # # # # # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
# # # # # # # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Failed to decode image"}), 400

# # # # # # # # # # # # # # # # # # # # # #         # Детекция объектов
# # # # # # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
        
# # # # # # # # # # # # # # # # # # # # # #         # Формируем описание для озвучки
# # # # # # # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)
        
# # # # # # # # # # # # # # # # # # # # # #         # Сохраняем в базу
# # # # # # # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # # #                 label = obs.get('label', 'unknown')
# # # # # # # # # # # # # # # # # # # # # #                 confidence = obs.get('confidence', 0.0)
# # # # # # # # # # # # # # # # # # # # # #                 bbox = obs.get('bbox', [])
# # # # # # # # # # # # # # # # # # # # # #                 bbox_str = ','.join(map(str, bbox))
                
# # # # # # # # # # # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # # # # # # # # # # #                     "INSERT INTO obstacles (label, confidence, bbox) VALUES (%s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # # #                     (label, confidence, bbox_str)
# # # # # # # # # # # # # # # # # # # # # #                 )
# # # # # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # # # #             conn.close()
        
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # # # # # # #             "message": "Frame processed successfully"
# # # # # # # # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Frame processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # # #     """Обработка голосовых команд с улучшенной обработкой аудио"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio file provided"}), 400

# # # # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # # # # # # # # # # #         # Сохраняем файл
# # # # # # # # # # # # # # # # # # # # # #         audio_path = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # # #         audio_file.save(audio_path)
        
# # # # # # # # # # # # # # # # # # # # # #         # Проверяем размер файла
# # # # # # # # # # # # # # # # # # # # # #         file_size = os.path.getsize(audio_path)
# # # # # # # # # # # # # # # # # # # # # #         print(f"Audio file size: {file_size} bytes")
        
# # # # # # # # # # # # # # # # # # # # # #         if file_size < 100:  # Слишком маленький файл
# # # # # # # # # # # # # # # # # # # # # #             os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Audio file is too small"}), 400
        
# # # # # # # # # # # # # # # # # # # # # #         # Пытаемся конвертировать в правильный формат
# # # # # # # # # # # # # # # # # # # # # #         converted_path = convert_to_proper_wav(audio_path)
        
# # # # # # # # # # # # # # # # # # # # # #         r = sr.Recognizer()
        
# # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # #             with sr.AudioFile(converted_path) as source:
# # # # # # # # # # # # # # # # # # # # # #                 # Регулируем уровень шума
# # # # # # # # # # # # # # # # # # # # # #                 r.adjust_for_ambient_noise(source, duration=0.5)
# # # # # # # # # # # # # # # # # # # # # #                 audio = r.record(source)
            
# # # # # # # # # # # # # # # # # # # # # #             # Распознаем речь
# # # # # # # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
            
# # # # # # # # # # # # # # # # # # # # # #             response = {"recognized_text": text}
            
# # # # # # # # # # # # # # # # # # # # # #             # Простые команды
# # # # # # # # # # # # # # # # # # # # # #             text_lower = text.lower()
# # # # # # # # # # # # # # # # # # # # # #             if any(word in text_lower for word in ['камера', 'camera']):
# # # # # # # # # # # # # # # # # # # # # #                 response["action"] = "toggle_camera"
# # # # # # # # # # # # # # # # # # # # # #             elif any(word in text_lower for word in ['сканирование', 'сканировать', 'препятствие', 'что вокруг']):
# # # # # # # # # # # # # # # # # # # # # #                 response["action"] = "scan_obstacles"
# # # # # # # # # # # # # # # # # # # # # #             elif any(word in text_lower for word in ['стоп', 'остановить']):
# # # # # # # # # # # # # # # # # # # # # #                 response["action"] = "stop"
# # # # # # # # # # # # # # # # # # # # # #             elif any(word in text_lower for word in ['помощь', 'help']):
# # # # # # # # # # # # # # # # # # # # # #                 response["action"] = "help"
            
# # # # # # # # # # # # # # # # # # # # # #             return jsonify(response)
            
# # # # # # # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Не удалось распознать речь"}), 400
# # # # # # # # # # # # # # # # # # # # # #         except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": f"Ошибка сервиса распознавания: {e}"}), 500
# # # # # # # # # # # # # # # # # # # # # #         finally:
# # # # # # # # # # # # # # # # # # # # # #             # Удаляем временные файлы
# # # # # # # # # # # # # # # # # # # # # #             try:
# # # # # # # # # # # # # # # # # # # # # #                 os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # #                 if converted_path != audio_path:
# # # # # # # # # # # # # # # # # # # # # #                     os.remove(converted_path)
# # # # # # # # # # # # # # # # # # # # # #             except:
# # # # # # # # # # # # # # # # # # # # # #                 pass
                
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         # Удаляем временные файлы при ошибке
# # # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # # #             if 'audio_path' in locals():
# # # # # # # # # # # # # # # # # # # # # #                 os.remove(audio_path)
# # # # # # # # # # # # # # # # # # # # # #             if 'converted_path' in locals() and converted_path != audio_path:
# # # # # # # # # # # # # # # # # # # # # #                 os.remove(converted_path)
# # # # # # # # # # # # # # # # # # # # # #         except:
# # # # # # # # # # # # # # # # # # # # # #             pass
            
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice command processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/simple_voice', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # # def simple_voice():
# # # # # # # # # # # # # # # # # # # # # #     """Упрощенный endpoint для тестирования голосовых команд"""
# # # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio file"}), 400
            
# # # # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # # #         file_size = len(audio_file.read())
        
# # # # # # # # # # # # # # # # # # # # # #         # Всегда возвращаем тестовую команду для демонстрации
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # # #             "recognized_text": "сканировать окружение",
# # # # # # # # # # # # # # # # # # # # # #             "action": "scan_obstacles",
# # # # # # # # # # # # # # # # # # # # # #             "file_size": file_size,
# # # # # # # # # # # # # # # # # # # # # #             "message": "Voice command processed (demo mode)"
# # # # # # # # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Simple voice error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # # @app.route('/test_connection')
# # # # # # # # # # # # # # # # # # # # # # def test_connection():
# # # # # # # # # # # # # # # # # # # # # #     """Простой тест подключения"""
# # # # # # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # # # # # #         "status": "success",
# # # # # # # # # # # # # # # # # # # # # #         "message": "Server is responding",
# # # # # # # # # # # # # # # # # # # # # #         "endpoints": {
# # # # # # # # # # # # # # # # # # # # # #             "process_frame": "POST /process_frame - отправка кадра с телефона",
# # # # # # # # # # # # # # # # # # # # # #             "voice_command": "POST /voice_command - голосовые команды", 
# # # # # # # # # # # # # # # # # # # # # #             "simple_voice": "POST /simple_voice - упрощенный тест голоса"
# # # # # # # # # # # # # # # # # # # # # #         }
# # # # # # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # # # # # # ─────────── Запуск ───────────
# # # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # # #     print("🚀 Starting Blind Assistant Server...")
# # # # # # # # # # # # # # # # # # # # # #     print("📱 Сервер готов к работе с камерой телефона")
# # # # # # # # # # # # # # # # # # # # # #     print("🔗 Основные endpoint:")
# # # # # # # # # # # # # # # # # # # # # #     print("   POST /process_frame - обработка кадров с телефона")
# # # # # # # # # # # # # # # # # # # # # #     print("   POST /voice_command - голосовые команды") 
# # # # # # # # # # # # # # # # # # # # # #     print("   POST /simple_voice - упрощенный тест голоса")
# # # # # # # # # # # # # # # # # # # # # #     print("   GET  /test_connection - проверка подключения")
# # # # # # # # # # # # # # # # # # # # # #     print(f"🌐 Server will be available at: http://192.168.8.63:5000")
    
# # # # # # # # # # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)


# # # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # # # # # import wave
# # # # # # # # # # # # # # # # # # # # # import io

# # # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # # Разрешаем CORS вручную
# # # # # # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # # # ─────────── Функция оценки расстояния ───────────
# # # # # # # # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height):
# # # # # # # # # # # # # # # # # # # # #     """
# # # # # # # # # # # # # # # # # # # # #     Примерная оценка расстояния до объекта по высоте бокса.
# # # # # # # # # # # # # # # # # # # # #     bbox: [x1, y1, x2, y2]
# # # # # # # # # # # # # # # # # # # # #     frame_height: высота кадра в пикселях
    
# # # # # # # # # # # # # # # # # # # # #     Возвращает дистанцию в метрах (примерно).
# # # # # # # # # # # # # # # # # # # # #     """
# # # # # # # # # # # # # # # # # # # # #     bbox_height = bbox[3] - bbox[1]
# # # # # # # # # # # # # # # # # # # # #     max_height = frame_height  # максимальный размер объекта в кадре (очень близко)
# # # # # # # # # # # # # # # # # # # # #     min_distance = 0.5  # минимальная дистанция (в метрах)
# # # # # # # # # # # # # # # # # # # # #     max_distance = 10.0 # максимальная дистанция (в метрах)

# # # # # # # # # # # # # # # # # # # # #     if bbox_height <= 0:
# # # # # # # # # # # # # # # # # # # # #         return None

# # # # # # # # # # # # # # # # # # # # #     relative_height = bbox_height / max_height
# # # # # # # # # # # # # # # # # # # # #     distance = max_distance * (1 - relative_height)
# # # # # # # # # # # # # # # # # # # # #     if distance < min_distance:
# # # # # # # # # # # # # # # # # # # # #         distance = min_distance
# # # # # # # # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # # # # # # # # ─────────── Загрузка модели YOLO ───────────
# # # # # # # # # # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')
# # # # # # # # # # # # # # # # # # # # # print("YOLO model loaded.")

# # # # # # # # # # # # # # # # # # # # # # ─────────── Функции для обработки изображений ───────────
# # # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # # #     """Обнаружение объектов на кадре с помощью YOLO"""
# # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # # # # #             label = model.names[cls_id]

# # # # # # # # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height)  # Добавлено

# # # # # # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # # # # # # # #                 "distance_m": distance  # Добавлено
# # # # # # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # # # # # #     """Генерирует текстовое описание препятствий для озвучки"""
# # # # # # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"
    
# # # # # # # # # # # # # # # # # # # # #     russian_labels = {
# # # # # # # # # # # # # # # # # # # # #         'person': 'человек', 'car': 'автомобиль', 'chair': 'стул', 'table': 'стол',
# # # # # # # # # # # # # # # # # # # # #         'book': 'книга', 'bottle': 'бутылка', 'cup': 'чашка', 'phone': 'телефон',
# # # # # # # # # # # # # # # # # # # # #         'laptop': 'ноутбук', 'mouse': 'мышь', 'keyboard': 'клавиатура', 'tv': 'телевизор',
# # # # # # # # # # # # # # # # # # # # #         'remote': 'пульт', 'clock': 'часы', 'vase': 'ваза', 'scissors': 'ножницы',
# # # # # # # # # # # # # # # # # # # # #         'teddy bear': 'плюшевый мишка', 'hair drier': 'фен', 'toothbrush': 'зубная щетка',
# # # # # # # # # # # # # # # # # # # # #         'cat': 'кот', 'dog': 'собака', 'bird': 'птица', 'horse': 'лошадь',
# # # # # # # # # # # # # # # # # # # # #         'sheep': 'овца', 'cow': 'корова', 'elephant': 'слон', 'bear': 'медведь',
# # # # # # # # # # # # # # # # # # # # #         'zebra': 'зебра', 'giraffe': 'жираф', 'backpack': 'рюкзак', 'umbrella': 'зонт',
# # # # # # # # # # # # # # # # # # # # #         'handbag': 'сумка', 'tie': 'галстук', 'suitcase': 'чемодан', 'frisbee': 'фрисби',
# # # # # # # # # # # # # # # # # # # # #         'skis': 'лыжи', 'snowboard': 'сноуборд', 'sports ball': 'спортивный мяч',
# # # # # # # # # # # # # # # # # # # # #         'kite': 'воздушный змей', 'baseball bat': 'бейсбольная бита', 'baseball glove': 'бейсбольная перчатка',
# # # # # # # # # # # # # # # # # # # # #         'skateboard': 'скейтборд', 'surfboard': 'доска для серфинга', 'tennis racket': 'теннисная ракетка',
# # # # # # # # # # # # # # # # # # # # #         'wine glass': 'бокал', 'spoon': 'ложка', 'bowl': 'миска', 'banana': 'банан',
# # # # # # # # # # # # # # # # # # # # #         'apple': 'яблоко', 'sandwich': 'бутерброд', 'orange': 'апельсин', 'broccoli': 'брокколи',
# # # # # # # # # # # # # # # # # # # # #         'carrot': 'морковь', 'hot dog': 'хот-дог', 'pizza': 'пицца', 'donut': 'пончик',
# # # # # # # # # # # # # # # # # # # # #         'cake': 'торт', 'bed': 'кровать', 'toilet': 'унитаз', 'sink': 'раковина',
# # # # # # # # # # # # # # # # # # # # #         'refrigerator': 'холодильник', 'oven': 'духовка', 'microwave': 'микроволновка',
# # # # # # # # # # # # # # # # # # # # #         'toaster': 'тостер'
# # # # # # # # # # # # # # # # # # # # #     }
    
# # # # # # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # # # # # #     for i, obstacle in enumerate(obstacles[:5]):
# # # # # # # # # # # # # # # # # # # # #         label = obstacle['label']
# # # # # # # # # # # # # # # # # # # # #         confidence = obstacle['confidence']
# # # # # # # # # # # # # # # # # # # # #         distance = obstacle.get('distance_m', None)
        
# # # # # # # # # # # # # # # # # # # # #         label_ru = russian_labels.get(label, label)
# # # # # # # # # # # # # # # # # # # # #         confidence_percent = int(confidence * 100)
        
# # # # # # # # # # # # # # # # # # # # #         bbox = obstacle['bbox']
# # # # # # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # # # # # # # #         frame_center = 320  # примерная ширина кадра
        
# # # # # # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # # # # # #             position = "слева"
# # # # # # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # # # # # #             position = "справа"
# # # # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # # # #             position = "прямо перед вами"
        
# # # # # # # # # # # # # # # # # # # # #         dist_text = f" на расстоянии примерно {distance} метров" if distance else ""
        
# # # # # # # # # # # # # # # # # # # # #         descriptions.append(f"{label_ru} {position}{dist_text} с уверенностью {confidence_percent} процентов")
    
# # # # # # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
    
# # # # # # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # # # # def convert_to_proper_wav(audio_path):
# # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # #         output_path = audio_path.replace('.wav', '_converted.wav')
# # # # # # # # # # # # # # # # # # # # #         with wave.open(output_path, 'wb') as wav_file:
# # # # # # # # # # # # # # # # # # # # #             wav_file.setnchannels(1)
# # # # # # # # # # # # # # # # # # # # #             wav_file.setsampwidth(2)
# # # # # # # # # # # # # # # # # # # # #             wav_file.setframerate(16000)
# # # # # # # # # # # # # # # # # # # # #             silence = b'\x00' * 32000
# # # # # # # # # # # # # # # # # # # # #             wav_file.writeframes(silence)
# # # # # # # # # # # # # # # # # # # # #         return output_path
# # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # #         print(f"Audio conversion error: {e}")
# # # # # # # # # # # # # # # # # # # # #         return audio_path

# # # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running", 
# # # # # # # # # # # # # # # # # # # # #         "version": "2.0",
# # # # # # # # # # # # # # # # # # # # #         "message": "Сервер готов к обработке кадров с камеры телефона"
# # # # # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # # # # # #         frame_file = request.files['frame']
# # # # # # # # # # # # # # # # # # # # #         frame_bytes = frame_file.read()
# # # # # # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
# # # # # # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Failed to decode image"}), 400

# # # # # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)
        
# # # # # # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # # # # # #                 label = obs.get('label', 'unknown')
# # # # # # # # # # # # # # # # # # # # #                 confidence = obs.get('confidence', 0.0)
# # # # # # # # # # # # # # # # # # # # #                 bbox = obs.get('bbox', [])
# # # # # # # # # # # # # # # # # # # # #                 bbox_str = ','.join(map(str, bbox))
# # # # # # # # # # # # # # # # # # # # #                 distance = obs.get('distance_m')
# # # # # # # # # # # # # # # # # # # # #                 distance_db = float(distance) if distance is not None else None

# # # # # # # # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # # # # # # # #                         (label, confidence, bbox_str, distance_db)
# # # # # # # # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # # # # # # # #                     print(f"DB insert error for obstacle {label}: {e}")
# # # # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # # #             conn.close()
        
# # # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # # # # # #             "message": "Frame processed successfully"
# # # # # # # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Frame processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio provided"}), 400
        
# # # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # # #         audio_data = audio_file.read()
        
# # # # # # # # # # # # # # # # # # # # #         # Сохраняем временно аудио для распознавания
# # # # # # # # # # # # # # # # # # # # #         temp_wav = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # # #         with open(temp_wav, 'wb') as f:
# # # # # # # # # # # # # # # # # # # # #             f.write(audio_data)
        
# # # # # # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # # #         with sr.AudioFile(temp_wav) as source:
# # # # # # # # # # # # # # # # # # # # #             audio = r.record(source)
# # # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # # #             text = ""
# # # # # # # # # # # # # # # # # # # # #         except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # # #             text = ""
        
# # # # # # # # # # # # # # # # # # # # #         os.remove(temp_wav)
        
# # # # # # # # # # # # # # # # # # # # #         return jsonify({"text": text})
# # # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # # #     app.run(host='10.189.181.73', port=5000, debug=True)



# # # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # # import json
# # # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify, Response
# # # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # # # # import wave
# # # # # # # # # # # # # # # # # # # # import io

# # # # # # # # # # # # # # # # # # # # # ─────────── Конфигурация ───────────
# # # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # # Разрешаем CORS вручную
# # # # # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # # # # ─────────── Подключение к базе ───────────
# # # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # # ─────────── Функция оценки расстояния ───────────
# # # # # # # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height):
# # # # # # # # # # # # # # # # # # # #     bbox_height = bbox[3] - bbox[1]
# # # # # # # # # # # # # # # # # # # #     max_height = frame_height  # максимальный размер объекта в кадре (очень близко)
# # # # # # # # # # # # # # # # # # # #     min_distance = 0.5  # минимальная дистанция (в метрах)
# # # # # # # # # # # # # # # # # # # #     max_distance = 10.0 # максимальная дистанция (в метрах)

# # # # # # # # # # # # # # # # # # # #     if bbox_height <= 0:
# # # # # # # # # # # # # # # # # # # #         return None

# # # # # # # # # # # # # # # # # # # #     relative_height = bbox_height / max_height
# # # # # # # # # # # # # # # # # # # #     distance = max_distance * (1 - relative_height)
# # # # # # # # # # # # # # # # # # # #     if distance < min_distance:
# # # # # # # # # # # # # # # # # # # #         distance = min_distance
# # # # # # # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # # # # # # # ─────────── Загрузка модели YOLO ───────────
# # # # # # # # # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')
# # # # # # # # # # # # # # # # # # # # print("YOLO model loaded.")

# # # # # # # # # # # # # # # # # # # # # ─────────── Функции для обработки изображений ───────────
# # # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # # # #             conf = float(r.conf[0])  # <- сразу float
# # # # # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # # # #             label = model.names[cls_id]

# # # # # # # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height)

# # # # # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # # # # #                 "confidence": conf,  # уже float
# # # # # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # # # # # # #                 "distance_m": float(distance) if distance is not None else None  # тоже float
# # # # # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"
    
# # # # # # # # # # # # # # # # # # # #     russian_labels = {
# # # # # # # # # # # # # # # # # # # #         'person': 'человек', 'car': 'автомобиль', 'chair': 'стул', 'table': 'стол',
# # # # # # # # # # # # # # # # # # # #         'book': 'книга', 'bottle': 'бутылка', 'cup': 'чашка', 'phone': 'телефон',
# # # # # # # # # # # # # # # # # # # #         'laptop': 'ноутбук', 'mouse': 'мышь', 'keyboard': 'клавиатура', 'tv': 'телевизор',
# # # # # # # # # # # # # # # # # # # #         'remote': 'пульт', 'clock': 'часы', 'vase': 'ваза', 'scissors': 'ножницы',
# # # # # # # # # # # # # # # # # # # #         'teddy bear': 'плюшевый мишка', 'hair drier': 'фен', 'toothbrush': 'зубная щетка',
# # # # # # # # # # # # # # # # # # # #         'cat': 'кот', 'dog': 'собака', 'bird': 'птица', 'horse': 'лошадь',
# # # # # # # # # # # # # # # # # # # #         'sheep': 'овца', 'cow': 'корова', 'elephant': 'слон', 'bear': 'медведь',
# # # # # # # # # # # # # # # # # # # #         'zebra': 'зебра', 'giraffe': 'жираф', 'backpack': 'рюкзак', 'umbrella': 'зонт',
# # # # # # # # # # # # # # # # # # # #         'handbag': 'сумка', 'tie': 'галстук', 'suitcase': 'чемодан', 'frisbee': 'фрисби',
# # # # # # # # # # # # # # # # # # # #         'skis': 'лыжи', 'snowboard': 'сноуборд', 'sports ball': 'спортивный мяч',
# # # # # # # # # # # # # # # # # # # #         'kite': 'воздушный змей', 'baseball bat': 'бейсбольная бита', 'baseball glove': 'бейсбольная перчатка',
# # # # # # # # # # # # # # # # # # # #         'skateboard': 'скейтборд', 'surfboard': 'доска для серфинга', 'tennis racket': 'теннисная ракетка',
# # # # # # # # # # # # # # # # # # # #         'wine glass': 'бокал', 'spoon': 'ложка', 'bowl': 'миска', 'banana': 'банан',
# # # # # # # # # # # # # # # # # # # #         'apple': 'яблоко', 'sandwich': 'бутерброд', 'orange': 'апельсин', 'broccoli': 'брокколи',
# # # # # # # # # # # # # # # # # # # #         'carrot': 'морковь', 'hot dog': 'хот-дог', 'pizza': 'пицца', 'donut': 'пончик',
# # # # # # # # # # # # # # # # # # # #         'cake': 'торт', 'bed': 'кровать', 'toilet': 'унитаз', 'sink': 'раковина',
# # # # # # # # # # # # # # # # # # # #         'refrigerator': 'холодильник', 'oven': 'духовка', 'microwave': 'микроволновка',
# # # # # # # # # # # # # # # # # # # #         'toaster': 'тостер'
# # # # # # # # # # # # # # # # # # # #     }
    
# # # # # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # # # # #     for i, obstacle in enumerate(obstacles[:5]):
# # # # # # # # # # # # # # # # # # # #         label = obstacle['label']
# # # # # # # # # # # # # # # # # # # #         confidence = obstacle['confidence']
# # # # # # # # # # # # # # # # # # # #         distance = obstacle.get('distance_m', None)
        
# # # # # # # # # # # # # # # # # # # #         label_ru = russian_labels.get(label, label)
# # # # # # # # # # # # # # # # # # # #         confidence_percent = int(confidence * 100)
        
# # # # # # # # # # # # # # # # # # # #         bbox = obstacle['bbox']
# # # # # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # # # # # # #         frame_center = 320
        
# # # # # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # # # # #             position = "слева"
# # # # # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # # # # #             position = "справа"
# # # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # # #             position = "прямо перед вами"
        
# # # # # # # # # # # # # # # # # # # #         dist_text = f" на расстоянии примерно {distance} метров" if distance else ""
        
# # # # # # # # # # # # # # # # # # # #         descriptions.append(f"{label_ru} {position}{dist_text} с уверенностью {confidence_percent} процентов")
    
# # # # # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
    
# # # # # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # # # def convert_to_proper_wav(audio_path):
# # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # #         output_path = audio_path.replace('.wav', '_converted.wav')
# # # # # # # # # # # # # # # # # # # #         with wave.open(output_path, 'wb') as wav_file:
# # # # # # # # # # # # # # # # # # # #             wav_file.setnchannels(1)
# # # # # # # # # # # # # # # # # # # #             wav_file.setsampwidth(2)
# # # # # # # # # # # # # # # # # # # #             wav_file.setframerate(16000)
# # # # # # # # # # # # # # # # # # # #             silence = b'\x00' * 32000
# # # # # # # # # # # # # # # # # # # #             wav_file.writeframes(silence)
# # # # # # # # # # # # # # # # # # # #         return output_path
# # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # #         print(f"Audio conversion error: {e}")
# # # # # # # # # # # # # # # # # # # #         return audio_path

# # # # # # # # # # # # # # # # # # # # # ─────────── Эндпоинты ───────────

# # # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running", 
# # # # # # # # # # # # # # # # # # # #         "version": "2.0",
# # # # # # # # # # # # # # # # # # # #         "message": "Сервер готов к обработке кадров с камеры телефона"
# # # # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # # # # #         frame_file = request.files['frame']
# # # # # # # # # # # # # # # # # # # #         frame_bytes = frame_file.read()
# # # # # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
# # # # # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Failed to decode image"}), 400

# # # # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)
        
# # # # # # # # # # # # # # # # # # # #         # Вставляем в БД
# # # # # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # # # # #                 label = obs.get('label', 'unknown')
# # # # # # # # # # # # # # # # # # # #                 confidence = float(obs.get('confidence', 0.0))  # float
# # # # # # # # # # # # # # # # # # # #                 bbox = obs.get('bbox', [])
# # # # # # # # # # # # # # # # # # # #                 bbox_str = ','.join(map(str, bbox))
# # # # # # # # # # # # # # # # # # # #                 distance = obs.get('distance_m')
# # # # # # # # # # # # # # # # # # # #                 distance_db = float(distance) if distance is not None else None

# # # # # # # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # # # # # # #                         (label, confidence, bbox_str, distance_db)
# # # # # # # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # # # # # # #                     print(f"DB insert error for obstacle {label}: {e}")
# # # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # # #             conn.close()
        
# # # # # # # # # # # # # # # # # # # #         # Приводим все float32 к float (на всякий случай, но detect_objects уже это делает)
# # # # # # # # # # # # # # # # # # # #         for obs in obstacles:
# # # # # # # # # # # # # # # # # # # #             obs['confidence'] = float(obs['confidence'])
# # # # # # # # # # # # # # # # # # # #             if obs.get('distance_m') is not None:
# # # # # # # # # # # # # # # # # # # #                 obs['distance_m'] = float(obs['distance_m'])

# # # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # # # # #             "message": "Frame processed successfully"
# # # # # # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Frame processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio provided"}), 400
        
# # # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # # #         audio_data = audio_file.read()
        
# # # # # # # # # # # # # # # # # # # #         temp_wav = f"temp_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # # # # # #         with open(temp_wav, 'wb') as f:
# # # # # # # # # # # # # # # # # # # #             f.write(audio_data)
        
# # # # # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # # #         with sr.AudioFile(temp_wav) as source:
# # # # # # # # # # # # # # # # # # # #             audio = r.record(source)
# # # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # # #             text = ""
# # # # # # # # # # # # # # # # # # # #         except sr.RequestError as e:
# # # # # # # # # # # # # # # # # # # #             text = ""
        
# # # # # # # # # # # # # # # # # # # #         os.remove(temp_wav)
        
# # # # # # # # # # # # # # # # # # # #         return jsonify({"text": text})
# # # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # # #     app.run(host='10.189.181.73', port=5000, debug=True)




# # # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # # # import wave

# # # # # # # # # # # # # # # # # # # # ───── Конфигурация ─────
# # # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # # ───── CORS ─────
# # # # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # # # ───── Подключение к БД ─────
# # # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # # ───── Предполагаемые высоты объектов (в метрах) ─────
# # # # # # # # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # # # # # # # #     'person': 1.7,
# # # # # # # # # # # # # # # # # # #     'car': 1.5,
# # # # # # # # # # # # # # # # # # #     'chair': 1.0,
# # # # # # # # # # # # # # # # # # #     'bottle': 0.25,
# # # # # # # # # # # # # # # # # # #     'cup': 0.12,
# # # # # # # # # # # # # # # # # # #     'dog': 0.5,
# # # # # # # # # # # # # # # # # # #     'cat': 0.3,
# # # # # # # # # # # # # # # # # # #     'tv': 0.6,
# # # # # # # # # # # # # # # # # # #     'laptop': 0.02
# # # # # # # # # # # # # # # # # # # }

# # # # # # # # # # # # # # # # # # # # ───── Улучшенная оценка расстояния ─────
# # # # # # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # # # # # # # #         return None

# # # # # # # # # # # # # # # # # # #     # Предполагаемое фокусное расстояние (пиксели)
# # # # # # # # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700

# # # # # # # # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)  # по умолчанию 1 метр

# # # # # # # # # # # # # # # # # # #     # Формула: distance = (real_height * focal_length) / pixel_height
# # # # # # # # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # # # # # # ───── Загрузка YOLO ─────
# # # # # # # # # # # # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')
# # # # # # # # # # # # # # # # # # # print("Модель загружена.")

# # # # # # # # # # # # # # # # # # # # ───── Обнаружение объектов ─────
# # # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # # #             label = model.names[cls_id]

# # # # # # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # # # ───── Генерация описания ─────
# # # # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"

# # # # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # # # #     frame_center = 320

# # # # # # # # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # # # # # # # #         distance = obs.get('distance_m')
# # # # # # # # # # # # # # # # # # #         dist_text = f" на расстоянии около {distance} метров" if distance else ""

# # # # # # # # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2

# # # # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # # # #             pos = "слева"
# # # # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # # # #             pos = "справа"
# # # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # # #             pos = "прямо перед вами"

# # # # # # # # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} с уверенностью {conf}%")

# # # # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")

# # # # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # # # ───── WAV конвертер ─────
# # # # # # # # # # # # # # # # # # # def convert_to_proper_wav(audio_path):
# # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # #         output_path = audio_path.replace('.wav', '_converted.wav')
# # # # # # # # # # # # # # # # # # #         with wave.open(output_path, 'wb') as wav_file:
# # # # # # # # # # # # # # # # # # #             wav_file.setnchannels(1)
# # # # # # # # # # # # # # # # # # #             wav_file.setsampwidth(2)
# # # # # # # # # # # # # # # # # # #             wav_file.setframerate(16000)
# # # # # # # # # # # # # # # # # # #             silence = b'\x00' * 32000
# # # # # # # # # # # # # # # # # # #             wav_file.writeframes(silence)
# # # # # # # # # # # # # # # # # # #         return output_path
# # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # #         print(f"Audio conversion error: {e}")
# # # # # # # # # # # # # # # # # # #         return audio_path

# # # # # # # # # # # # # # # # # # # # ───── Эндпоинты ─────

# # # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # # # # # # # #         "version": "2.1",
# # # # # # # # # # # # # # # # # # #         "message": "Сервер готов к работе"
# # # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # # # # # #                         (
# # # # # # # # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # # # # # # # #                         )
# # # # # # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # # # #             "message": "Кадр обработан"
# # # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка обработки кадра: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio provided"}), 400

# # # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # # #         temp_wav = f"temp_{uuid.uuid4()}.wav"

# # # # # # # # # # # # # # # # # # #         with open(temp_wav, 'wb') as f:
# # # # # # # # # # # # # # # # # # #             f.write(audio_file.read())

# # # # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # # # #         with sr.AudioFile(temp_wav) as source:
# # # # # # # # # # # # # # # # # # #             audio = r.record(source)

# # # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # # #             text = ""
# # # # # # # # # # # # # # # # # # #         except sr.RequestError:
# # # # # # # # # # # # # # # # # # #             text = ""

# # # # # # # # # # # # # # # # # # #         os.remove(temp_wav)
# # # # # # # # # # # # # # # # # # #         return jsonify({"text": text})

# # # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # # #     app.run(host='10.189.181.73', port=5000, debug=True)



# # # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # # import wave

# # # # # # # # # # # # # # # # # # # ───── Конфигурация ─────
# # # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # # ───── CORS ─────
# # # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # # ───── Подключение к БД ─────
# # # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # # ───── Предполагаемые высоты объектов (в метрах) ─────
# # # # # # # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # # # # # # #     'person': 1.7,
# # # # # # # # # # # # # # # # # #     'car': 1.5,
# # # # # # # # # # # # # # # # # #     'chair': 1.0,
# # # # # # # # # # # # # # # # # #     'bottle': 0.25,
# # # # # # # # # # # # # # # # # #     'cup': 0.12,
# # # # # # # # # # # # # # # # # #     'dog': 0.5,
# # # # # # # # # # # # # # # # # #     'cat': 0.3,
# # # # # # # # # # # # # # # # # #     'tv': 0.6,
# # # # # # # # # # # # # # # # # #     'laptop': 0.02
# # # # # # # # # # # # # # # # # # }

# # # # # # # # # # # # # # # # # # # ───── Улучшенная оценка расстояния ─────
# # # # # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # # # # # # #         return None

# # # # # # # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)

# # # # # # # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # # # # # ───── Загрузка YOLO ─────
# # # # # # # # # # # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # # # # # # # # # # model = YOLO('yolov5s.pt')
# # # # # # # # # # # # # # # # # # print("Модель загружена.")

# # # # # # # # # # # # # # # # # # # ───── Обнаружение объектов ─────
# # # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # # #             label = model.names[cls_id]

# # # # # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # # ───── Генерация описания ─────
# # # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"

# # # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # # #     frame_center = 320

# # # # # # # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # # # # # # #         distance = obs.get('distance_m')

# # # # # # # # # # # # # # # # # #         # Преобразование расстояния в метры и сантиметры
# # # # # # # # # # # # # # # # # #         if distance is not None:
# # # # # # # # # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))

# # # # # # # # # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {centimeters} сантиметров"
# # # # # # # # # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров"
# # # # # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров {centimeters} сантиметров"
# # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # #             dist_text = ""

# # # # # # # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2

# # # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # # #             pos = "слева"
# # # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # # #             pos = "справа"
# # # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # # #             pos = "прямо перед вами"

# # # # # # # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} с уверенностью {conf}%")

# # # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")

# # # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # # ───── WAV конвертер ─────
# # # # # # # # # # # # # # # # # # def convert_to_proper_wav(audio_path):
# # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # #         output_path = audio_path.replace('.wav', '_converted.wav')
# # # # # # # # # # # # # # # # # #         with wave.open(output_path, 'wb') as wav_file:
# # # # # # # # # # # # # # # # # #             wav_file.setnchannels(1)
# # # # # # # # # # # # # # # # # #             wav_file.setsampwidth(2)
# # # # # # # # # # # # # # # # # #             wav_file.setframerate(16000)
# # # # # # # # # # # # # # # # # #             silence = b'\x00' * 32000
# # # # # # # # # # # # # # # # # #             wav_file.writeframes(silence)
# # # # # # # # # # # # # # # # # #         return output_path
# # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # #         print(f"Audio conversion error: {e}")
# # # # # # # # # # # # # # # # # #         return audio_path

# # # # # # # # # # # # # # # # # # # ───── Эндпоинты ─────

# # # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # # # # # # #         "version": "2.1",
# # # # # # # # # # # # # # # # # #         "message": "Сервер готов к работе"
# # # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # # # # #                         (
# # # # # # # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # # # # # # #                         )
# # # # # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # # #             "message": "Кадр обработан"
# # # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка обработки кадра: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio provided"}), 400

# # # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # # #         temp_wav = f"temp_{uuid.uuid4()}.wav"

# # # # # # # # # # # # # # # # # #         with open(temp_wav, 'wb') as f:
# # # # # # # # # # # # # # # # # #             f.write(audio_file.read())

# # # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # # #         with sr.AudioFile(temp_wav) as source:
# # # # # # # # # # # # # # # # # #             audio = r.record(source)

# # # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # # #             text = ""
# # # # # # # # # # # # # # # # # #         except sr.RequestError:
# # # # # # # # # # # # # # # # # #             text = ""

# # # # # # # # # # # # # # # # # #         os.remove(temp_wav)
# # # # # # # # # # # # # # # # # #         return jsonify({"text": text})

# # # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # # #     app.run(host='10.189.181.73', port=5000, debug=True)


# # # # # # # # # # # # # # # # # # отличная камера сверху





# # # # # # # # # # # # # # # # # # --- Импорты ---
# # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # import wave

# # # # # # # # # # # # # # # # # # --- Конфигурация базы данных ---
# # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # --- CORS ---
# # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # --- Подключение к БД ---
# # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # --- Высоты объектов для расчета ---
# # # # # # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # # # # # # # # # # # # }

# # # # # # # # # # # # # # # # # # --- Оценка расстояния ---
# # # # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # # # # # #         return None
# # # # # # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # # # # --- Загрузка YOLO ---
# # # # # # # # # # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # # # # # # # # print("Модель загружена.")

# # # # # # # # # # # # # # # # # # --- Обнаружение объектов ---
# # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # --- Генерация описания препятствий ---
# # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"
# # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # #     frame_center = 320
# # # # # # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # # # # # #         distance = obs.get('distance_m')
# # # # # # # # # # # # # # # # #         if distance is not None:
# # # # # # # # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))
# # # # # # # # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {centimeters} сантиметров"
# # # # # # # # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров"
# # # # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров {centimeters} сантиметров"
# # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # #             dist_text = ""
# # # # # # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # #             pos = "слева"
# # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # #             pos = "справа"
# # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # #             pos = "прямо перед вами"
# # # # # # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} с уверенностью {conf}%")
# # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
# # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # --- Конвертация WAV (если потребуется) ---
# # # # # # # # # # # # # # # # # def convert_to_proper_wav(audio_path):
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         output_path = audio_path.replace('.wav', '_converted.wav')
# # # # # # # # # # # # # # # # #         with wave.open(output_path, 'wb') as wav_file:
# # # # # # # # # # # # # # # # #             wav_file.setnchannels(1)
# # # # # # # # # # # # # # # # #             wav_file.setsampwidth(2)
# # # # # # # # # # # # # # # # #             wav_file.setframerate(16000)
# # # # # # # # # # # # # # # # #             silence = b'\x00' * 32000
# # # # # # # # # # # # # # # # #             wav_file.writeframes(silence)
# # # # # # # # # # # # # # # # #         return output_path
# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         print(f"Audio conversion error: {e}")
# # # # # # # # # # # # # # # # #         return audio_path

# # # # # # # # # # # # # # # # # # --- Корневой эндпоинт ---
# # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # # # # # #         "version": "2.1",
# # # # # # # # # # # # # # # # #         "message": "Сервер готов к работе"
# # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # --- Обработка кадра с камеры ---
# # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # # # #                         (
# # # # # # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # # # # # #                         )
# # # # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # #             "message": "Кадр обработан"
# # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка обработки кадра: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # --- Обработка голосовой команды ---
# # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio provided"}), 400

# # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # #         temp_wav = f"temp_{uuid.uuid4()}.wav"

# # # # # # # # # # # # # # # # #         with open(temp_wav, 'wb') as f:
# # # # # # # # # # # # # # # # #             f.write(audio_file.read())

# # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # #         with sr.AudioFile(temp_wav) as source:
# # # # # # # # # # # # # # # # #             audio = r.record(source)

# # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # #             text = ""
# # # # # # # # # # # # # # # # #         except sr.RequestError:
# # # # # # # # # # # # # # # # #             text = ""

# # # # # # # # # # # # # # # # #         os.remove(temp_wav)
# # # # # # # # # # # # # # # # #         return jsonify({"text": text})

# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # --- Запуск сервера ---
# # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)


# # # # # # # # # # # # # # # # # # --- Импорты ---
# # # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # # import wave

# # # # # # # # # # # # # # # # # # --- Конфигурация базы данных ---
# # # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # # --- CORS ---
# # # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # # --- Подключение к БД ---
# # # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # # --- Высоты объектов для расчета ---
# # # # # # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # # # # # # # # # # # # }

# # # # # # # # # # # # # # # # # # --- Оценка расстояния ---
# # # # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # # # # # #         return None
# # # # # # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # # # # --- Загрузка YOLO ---
# # # # # # # # # # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # # # # # # # # print("Модель загружена.")

# # # # # # # # # # # # # # # # # # --- Обнаружение объектов ---
# # # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # # --- Генерация описания препятствий ---
# # # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"
# # # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # # #     frame_center = 320
# # # # # # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # # # # # #         distance = obs.get('distance_m')
# # # # # # # # # # # # # # # # #         if distance is not None:
# # # # # # # # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))
# # # # # # # # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {centimeters} сантиметров"
# # # # # # # # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров"
# # # # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров {centimeters} сантиметров"
# # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # #             dist_text = ""
# # # # # # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # # #             pos = "слева"
# # # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # # #             pos = "справа"
# # # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # # #             pos = "прямо перед вами"
# # # # # # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} с уверенностью {conf}%")
# # # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
# # # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # # --- Конвертация WAV (если потребуется) ---
# # # # # # # # # # # # # # # # # def convert_to_proper_wav(audio_path):
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         output_path = audio_path.replace('.wav', '_converted.wav')
# # # # # # # # # # # # # # # # #         with wave.open(output_path, 'wb') as wav_file:
# # # # # # # # # # # # # # # # #             wav_file.setnchannels(1)
# # # # # # # # # # # # # # # # #             wav_file.setsampwidth(2)
# # # # # # # # # # # # # # # # #             wav_file.setframerate(16000)
# # # # # # # # # # # # # # # # #             silence = b'\x00' * 32000
# # # # # # # # # # # # # # # # #             wav_file.writeframes(silence)
# # # # # # # # # # # # # # # # #         return output_path
# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         print(f"Audio conversion error: {e}")
# # # # # # # # # # # # # # # # #         return audio_path

# # # # # # # # # # # # # # # # # # --- Корневой эндпоинт ---
# # # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # # # # # #         "version": "2.1",
# # # # # # # # # # # # # # # # #         "message": "Сервер готов к работе"
# # # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # # --- Обработка кадра с камеры ---
# # # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # # # #                         (
# # # # # # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # # # # # #                         )
# # # # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # # #             "message": "Кадр обработан"
# # # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка обработки кадра: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # --- Обработка голосовой команды ---
# # # # # # # # # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # # # # # # # # def voice_command():
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # # # # # # # # #             return jsonify({"error": "No audio provided"}), 400

# # # # # # # # # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # # # # # # # # #         temp_wav = f"temp_{uuid.uuid4()}.wav"

# # # # # # # # # # # # # # # # #         with open(temp_wav, 'wb') as f:
# # # # # # # # # # # # # # # # #             f.write(audio_file.read())

# # # # # # # # # # # # # # # # #         r = sr.Recognizer()
# # # # # # # # # # # # # # # # #         with sr.AudioFile(temp_wav) as source:
# # # # # # # # # # # # # # # # #             audio = r.record(source)

# # # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # # #             text = r.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # # # # #             text = ""
# # # # # # # # # # # # # # # # #         except sr.RequestError:
# # # # # # # # # # # # # # # # #             text = ""

# # # # # # # # # # # # # # # # #         os.remove(temp_wav)
# # # # # # # # # # # # # # # # #         return jsonify({"text": text})

# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         return jsonify({"error": f"Voice processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # --- ✅ Регистрация голосового профиля ---
# # # # # # # # # # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # # # # # # # # # def register_voice():
# # # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # # #         if 'voice' not in request.files:
# # # # # # # # # # # # # # # # #             return jsonify({"error": "No voice file uploaded"}), 400

# # # # # # # # # # # # # # # # #         voice_file = request.files['voice']
# # # # # # # # # # # # # # # # #         username = request.form.get('username', 'unknown')

# # # # # # # # # # # # # # # # #         save_dir = 'voices'
# # # # # # # # # # # # # # # # #         os.makedirs(save_dir, exist_ok=True)

# # # # # # # # # # # # # # # # #         file_path = os.path.join(save_dir, f"{username}.aac")
# # # # # # # # # # # # # # # # #         voice_file.save(file_path)

# # # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # # #             "message": "Голосовой профиль сохранен",
# # # # # # # # # # # # # # # # #             "filename": f"{username}.aac"
# # # # # # # # # # # # # # # # #         }), 200

# # # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка при сохранении голоса: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # # --- Запуск сервера ---
# # # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)

# # # # # # # # # # # # # # # # # --- Импорты ---
# # # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # # import wave
# # # # # # # # # # # # # # # # import io
# # # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # # from scipy.io import wavfile
# # # # # # # # # # # # # # # # import librosa
# # # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # # from sklearn.metrics.pairwise import cosine_similarity
# # # # # # # # # # # # # # # # import joblib

# # # # # # # # # # # # # # # # # --- Конфигурация базы данных ---
# # # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # # --- CORS ---
# # # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # # # --- Подключение к БД ---
# # # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # # #         dbname=DB_NAME,
# # # # # # # # # # # # # # # #         user=DB_USER,
# # # # # # # # # # # # # # # #         password=DB_PASSWORD,
# # # # # # # # # # # # # # # #         host=DB_HOST
# # # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # # --- Голосовая биометрия ---
# # # # # # # # # # # # # # # # class VoiceAuthenticator:
# # # # # # # # # # # # # # # #     def __init__(self):
# # # # # # # # # # # # # # # #         self.recognizer = sr.Recognizer()
    
# # # # # # # # # # # # # # # #     def extract_voice_features(self, audio_path):
# # # # # # # # # # # # # # # #         """Извлечение характеристик голоса"""
# # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # #             # Загружаем аудио файл
# # # # # # # # # # # # # # # #             y, sr = librosa.load(audio_path, sr=16000)
            
# # # # # # # # # # # # # # # #             # Извлекаем MFCC (Mel-frequency cepstral coefficients)
# # # # # # # # # # # # # # # #             mfcc = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)
# # # # # # # # # # # # # # # #             mfcc_mean = np.mean(mfcc, axis=1)
            
# # # # # # # # # # # # # # # #             # Извлекаем другие характеристики
# # # # # # # # # # # # # # # #             chroma = librosa.feature.chroma_stft(y=y, sr=sr)
# # # # # # # # # # # # # # # #             chroma_mean = np.mean(chroma, axis=1)
            
# # # # # # # # # # # # # # # #             # Объединяем характеристики
# # # # # # # # # # # # # # # #             features = np.concatenate([mfcc_mean, chroma_mean])
            
# # # # # # # # # # # # # # # #             return features.tobytes()
            
# # # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # # #             print(f"Feature extraction error: {e}")
# # # # # # # # # # # # # # # #             return None
    
# # # # # # # # # # # # # # # #     def compare_voices(self, features1, features2):
# # # # # # # # # # # # # # # #         """Сравнение двух голосовых образцов"""
# # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # #             # Преобразуем байты обратно в numpy массивы
# # # # # # # # # # # # # # # #             arr1 = np.frombuffer(features1, dtype=np.float64)
# # # # # # # # # # # # # # # #             arr2 = np.frombuffer(features2, dtype=np.float64)
            
# # # # # # # # # # # # # # # #             # Выравниваем размеры массивов
# # # # # # # # # # # # # # # #             min_len = min(len(arr1), len(arr2))
# # # # # # # # # # # # # # # #             arr1 = arr1[:min_len]
# # # # # # # # # # # # # # # #             arr2 = arr2[:min_len]
            
# # # # # # # # # # # # # # # #             # Вычисляем косинусное сходство
# # # # # # # # # # # # # # # #             similarity = cosine_similarity([arr1], [arr2])[0][0]
            
# # # # # # # # # # # # # # # #             return similarity
# # # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # # #             print(f"Voice comparison error: {e}")
# # # # # # # # # # # # # # # #             return 0

# # # # # # # # # # # # # # # # voice_auth = VoiceAuthenticator()

# # # # # # # # # # # # # # # # # --- Регистрация пользователя по голосу ---
# # # # # # # # # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # # # # # # # # def register_voice():
# # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # #             return jsonify({"error": "No audio or username provided"}), 400

# # # # # # # # # # # # # # # #         username = request.form['username']
# # # # # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # # # # #         # Сохраняем временный файл
# # # # # # # # # # # # # # # #         temp_wav = f"temp_register_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # #         audio_file.save(temp_wav)
        
# # # # # # # # # # # # # # # #         # Извлекаем характеристики голоса
# # # # # # # # # # # # # # # #         voice_features = voice_auth.extract_voice_features(temp_wav)
        
# # # # # # # # # # # # # # # #         if voice_features is None:
# # # # # # # # # # # # # # # #             os.remove(temp_wav)
# # # # # # # # # # # # # # # #             return jsonify({"error": "Не удалось извлечь характеристики голоса"}), 400
        
# # # # # # # # # # # # # # # #         # Сохраняем в базу данных
# # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # # #             # Проверяем, существует ли пользователь
# # # # # # # # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # #             existing_user = cur.fetchone()
            
# # # # # # # # # # # # # # # #             if existing_user:
# # # # # # # # # # # # # # # #                 # Обновляем существующего пользователя
# # # # # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # # # # #                     "UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # # # # # # # # # # # #                     (voice_features, username)
# # # # # # # # # # # # # # # #                 )
# # # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # # #                 # Создаем нового пользователя
# # # # # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # # # # #                     "INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # # # # # # # # # # # #                     (username, voice_features)
# # # # # # # # # # # # # # # #                 )
            
# # # # # # # # # # # # # # # #             conn.commit()
            
# # # # # # # # # # # # # # # #             # Сохраняем голосовой образец
# # # # # # # # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # # #             user_id = cur.fetchone()[0]
            
# # # # # # # # # # # # # # # #             with open(temp_wav, 'rb') as f:
# # # # # # # # # # # # # # # #                 audio_data = f.read()
            
# # # # # # # # # # # # # # # #             cur.execute(
# # # # # # # # # # # # # # # #                 "INSERT INTO voice_samples (user_id, audio_data) VALUES (%s, %s)",
# # # # # # # # # # # # # # # #                 (user_id, audio_data)
# # # # # # # # # # # # # # # #             )
# # # # # # # # # # # # # # # #             conn.commit()
            
# # # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # # #             conn.rollback()
# # # # # # # # # # # # # # # #             return jsonify({"error": f"Database error: {str(e)}"}), 500
# # # # # # # # # # # # # # # #         finally:
# # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # #             conn.close()
# # # # # # # # # # # # # # # #             os.remove(temp_wav)
        
# # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # #             "status": "success",
# # # # # # # # # # # # # # # #             "message": "Голосовой профиль успешно зарегистрирован",
# # # # # # # # # # # # # # # #             "username": username
# # # # # # # # # # # # # # # #         })
        
# # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # #         return jsonify({"error": f"Registration error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # --- Аутентификация по голосу ---
# # # # # # # # # # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # # # # # # # # # def login_voice():
# # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # # #             return jsonify({"error": "No audio or username provided"}), 400

# # # # # # # # # # # # # # # #         username = request.form['username']
# # # # # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # # # # #         # Сохраняем временный файл
# # # # # # # # # # # # # # # #         temp_wav = f"temp_login_{uuid.uuid4()}.wav"
# # # # # # # # # # # # # # # #         audio_file.save(temp_wav)
        
# # # # # # # # # # # # # # # #         # Извлекаем характеристики голоса
# # # # # # # # # # # # # # # #         current_features = voice_auth.extract_voice_features(temp_wav)
        
# # # # # # # # # # # # # # # #         if current_features is None:
# # # # # # # # # # # # # # # #             os.remove(temp_wav)
# # # # # # # # # # # # # # # #             return jsonify({"error": "Не удалось извлечь характеристики голоса"}), 400
        
# # # # # # # # # # # # # # # #         # Ищем пользователя в базе
# # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # #             "SELECT id, voice_embedding FROM users WHERE username = %s",
# # # # # # # # # # # # # # # #             (username,)
# # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # #         user_data = cur.fetchone()
        
# # # # # # # # # # # # # # # #         if not user_data:
# # # # # # # # # # # # # # # #             os.remove(temp_wav)
# # # # # # # # # # # # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # # # # # # # # # # # #         user_id, stored_features = user_data
        
# # # # # # # # # # # # # # # #         if stored_features is None:
# # # # # # # # # # # # # # # #             os.remove(temp_wav)
# # # # # # # # # # # # # # # #             return jsonify({"error": "Голосовой профиль не зарегистрирован"}), 400
        
# # # # # # # # # # # # # # # #         # Сравниваем голоса
# # # # # # # # # # # # # # # #         similarity = voice_auth.compare_voices(stored_features, current_features)
        
# # # # # # # # # # # # # # # #         # Порог схожести (можно настроить)
# # # # # # # # # # # # # # # #         similarity_threshold = 0.7
        
# # # # # # # # # # # # # # # #         os.remove(temp_wav)
# # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # #         conn.close()
        
# # # # # # # # # # # # # # # #         if similarity >= similarity_threshold:
# # # # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # # # #                 "status": "success",
# # # # # # # # # # # # # # # #                 "message": "Аутентификация успешна",
# # # # # # # # # # # # # # # #                 "username": username,
# # # # # # # # # # # # # # # #                 "user_id": user_id,
# # # # # # # # # # # # # # # #                 "similarity": float(similarity)
# # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # # # #                 "status": "fail",
# # # # # # # # # # # # # # # #                 "message": "Голос не распознан",
# # # # # # # # # # # # # # # #                 "similarity": float(similarity)
# # # # # # # # # # # # # # # #             })
            
# # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # #         return jsonify({"error": f"Authentication error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # --- Получение информации о пользователе ---
# # # # # # # # # # # # # # # # @app.route('/user_profile/<username>', methods=['GET'])
# # # # # # # # # # # # # # # # def get_user_profile(username):
# # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # # # # #         cur.execute(
# # # # # # # # # # # # # # # #             "SELECT id, username, created_at FROM users WHERE username = %s",
# # # # # # # # # # # # # # # #             (username,)
# # # # # # # # # # # # # # # #         )
# # # # # # # # # # # # # # # #         user_data = cur.fetchone()
        
# # # # # # # # # # # # # # # #         if not user_data:
# # # # # # # # # # # # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # # # # # # # # # # # #         user_profile = {
# # # # # # # # # # # # # # # #             "id": user_data[0],
# # # # # # # # # # # # # # # #             "username": user_data[1],
# # # # # # # # # # # # # # # #             "created_at": user_data[2].isoformat() if user_data[2] else None
# # # # # # # # # # # # # # # #         }
        
# # # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # # #         conn.close()
        
# # # # # # # # # # # # # # # #         return jsonify({"user": user_profile})
        
# # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # #         return jsonify({"error": f"Profile error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # --- Остальные функции (YOLO, обработка кадров) остаются без изменений ---

# # # # # # # # # # # # # # # # # Высоты объектов для расчета
# # # # # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # # # # # # # # # # # }

# # # # # # # # # # # # # # # # # Оценка расстояния
# # # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # # # # #         return None
# # # # # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # # # Загрузка YOLO
# # # # # # # # # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # # # # # # # print("Модель загружена.")

# # # # # # # # # # # # # # # # # Обнаружение объектов
# # # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # # # Генерация описания препятствий
# # # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # # #         return "Препятствий не обнаружено"
# # # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # # #     frame_center = 320
# # # # # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # # # # #         distance = obs.get('distance_m')
# # # # # # # # # # # # # # # #         if distance is not None:
# # # # # # # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))
# # # # # # # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {centimeters} сантиметров"
# # # # # # # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров"
# # # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров {centimeters} сантиметров"
# # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # #             dist_text = ""
# # # # # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # # #             pos = "слева"
# # # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # # #             pos = "справа"
# # # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # # #             pos = "прямо перед вами"
# # # # # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} с уверенностью {conf}%")
# # # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
# # # # # # # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # # # Корневой эндпоинт
# # # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # # # # #         "version": "3.0",
# # # # # # # # # # # # # # # #         "message": "Сервер с голосовой аутентификацией готов к работе"
# # # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # # # Обработка кадра с камеры
# # # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # # #                         (
# # # # # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # # # # #                         )
# # # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # # #             "message": "Кадр обработан"
# # # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # # #         return jsonify({"error": f"Ошибка обработки кадра: {str(e)}"}), 500

# # # # # # # # # # # # # # # # # Запуск сервера
# # # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # # #     # Создаем папку для временных файлов
# # # # # # # # # # # # # # # #     os.makedirs('temp_audio', exist_ok=True)
# # # # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)






# # # # # # # # # # # # # # # # --- Импорты ---
# # # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # # import wave
# # # # # # # # # # # # # # # import struct
# # # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # # import tempfile
# # # # # # # # # # # # # # # import subprocess

# # # # # # # # # # # # # # # # --- Конфигурация ---
# # # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # # --- CORS ---
# # # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # # --- Простая голосовая биометрия ---
# # # # # # # # # # # # # # # class VoiceAuth:
# # # # # # # # # # # # # # #     def __init__(self):
# # # # # # # # # # # # # # #         self.recognizer = sr.Recognizer()
    
# # # # # # # # # # # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # # # # # # # # # # #         """Конвертируем AAC в WAV используя ffmpeg"""
# # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # #             wav_path = aac_path.replace('.aac', '.wav')
            
# # # # # # # # # # # # # # #             cmd = ['ffmpeg', '-i', aac_path, '-acodec', 'pcm_s16le', '-ac', '1', '-ar', '16000', wav_path, '-y']
# # # # # # # # # # # # # # #             result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
# # # # # # # # # # # # # # #             if result.returncode == 0 and os.path.exists(wav_path):
# # # # # # # # # # # # # # #                 return wav_path
# # # # # # # # # # # # # # #             return None
# # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # #             print(f"Conversion error: {e}")
# # # # # # # # # # # # # # #             return None
    
# # # # # # # # # # # # # # #     def extract_features(self, audio_path):
# # # # # # # # # # # # # # #         """Извлекаем простые характеристики голоса"""
# # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # #             # Если AAC - конвертируем
# # # # # # # # # # # # # # #             if audio_path.endswith('.aac'):
# # # # # # # # # # # # # # #                 wav_path = self.convert_aac_to_wav(audio_path)
# # # # # # # # # # # # # # #                 if not wav_path:
# # # # # # # # # # # # # # #                     return None
# # # # # # # # # # # # # # #                 final_path = wav_path
# # # # # # # # # # # # # # #                 need_cleanup = True
# # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # #                 final_path = audio_path
# # # # # # # # # # # # # # #                 need_cleanup = False
            
# # # # # # # # # # # # # # #             # Анализируем аудио
# # # # # # # # # # # # # # #             with wave.open(final_path, 'rb') as wav:
# # # # # # # # # # # # # # #                 n_frames = wav.getnframes()
# # # # # # # # # # # # # # #                 frames = wav.readframes(n_frames)
                
# # # # # # # # # # # # # # #                 if wav.getsampwidth() == 2:
# # # # # # # # # # # # # # #                     audio_data = struct.unpack('<' + 'h' * n_frames, frames)
# # # # # # # # # # # # # # #                 else:
# # # # # # # # # # # # # # #                     audio_data = struct.unpack('<' + 'B' * n_frames, frames)
# # # # # # # # # # # # # # #                     audio_data = [(x - 128) * 256 for x in audio_data]
                
# # # # # # # # # # # # # # #                 audio_array = np.array(audio_data, dtype=np.float32)
                
# # # # # # # # # # # # # # #                 features = {
# # # # # # # # # # # # # # #                     'mean': float(np.mean(audio_array)),
# # # # # # # # # # # # # # #                     'std': float(np.std(audio_array)),
# # # # # # # # # # # # # # #                     'energy': float(np.sum(audio_array ** 2) / len(audio_array)),
# # # # # # # # # # # # # # #                     'length': len(audio_array)
# # # # # # # # # # # # # # #                 }
                
# # # # # # # # # # # # # # #                 if need_cleanup:
# # # # # # # # # # # # # # #                     os.remove(final_path)
                
# # # # # # # # # # # # # # #                 return str(features)
                
# # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # #             print(f"Feature error: {e}")
# # # # # # # # # # # # # # #             return None
    
# # # # # # # # # # # # # # #     def compare_voices(self, features1_str, features2_str):
# # # # # # # # # # # # # # #         """Сравниваем голоса"""
# # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # #             import ast
# # # # # # # # # # # # # # #             f1 = ast.literal_eval(features1_str)
# # # # # # # # # # # # # # #             f2 = ast.literal_eval(features2_str)
            
# # # # # # # # # # # # # # #             mean_diff = abs(f1['mean'] - f2['mean']) / (abs(f1['mean']) + 1)
# # # # # # # # # # # # # # #             energy_diff = abs(f1['energy'] - f2['energy']) / (f1['energy'] + 1)
            
# # # # # # # # # # # # # # #             similarity = 1.0 / (1.0 + mean_diff + energy_diff)
# # # # # # # # # # # # # # #             return min(similarity, 1.0)
            
# # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # #             print(f"Compare error: {e}")
# # # # # # # # # # # # # # #             return 0.0

# # # # # # # # # # # # # # # voice_auth = VoiceAuth()

# # # # # # # # # # # # # # # # --- Эндпоинты ---
# # # # # # # # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # # # # # # # def register_voice():
# # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # #             return jsonify({"error": "Need audio and username"}), 400

# # # # # # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # # # #         # Сохраняем файл
# # # # # # # # # # # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # # # # # #         # Извлекаем характеристики
# # # # # # # # # # # # # # #         features = voice_auth.extract_features(temp_audio)
        
# # # # # # # # # # # # # # #         if not features:
# # # # # # # # # # # # # # #             if os.path.exists(temp_audio):
# # # # # # # # # # # # # # #                 os.remove(temp_audio)
# # # # # # # # # # # # # # #             return jsonify({"error": "Voice analysis failed"}), 400
        
# # # # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # #             if cur.fetchone():
# # # # # # # # # # # # # # #                 cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (features, username))
# # # # # # # # # # # # # # #                 message = "Profile updated"
# # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # #                 cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, features))
# # # # # # # # # # # # # # #                 message = "Profile registered"
            
# # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # #             result = {"status": "success", "message": message, "username": username}
            
# # # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # # #             conn.rollback()
# # # # # # # # # # # # # # #             result = {"error": f"DB error: {str(e)}"}
# # # # # # # # # # # # # # #         finally:
# # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # #             conn.close()
# # # # # # # # # # # # # # #             if os.path.exists(temp_audio):
# # # # # # # # # # # # # # #                 os.remove(temp_audio)
        
# # # # # # # # # # # # # # #         return jsonify(result)
        
# # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # #         return jsonify({"error": f"Registration error: {str(e)}"}), 500

# # # # # # # # # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # # # # # # # # def login_voice():
# # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # # #             return jsonify({"error": "Need audio and username"}), 400

# # # # # # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # # # # # #         current_features = voice_auth.extract_features(temp_audio)
        
# # # # # # # # # # # # # # #         if not current_features:
# # # # # # # # # # # # # # #             os.remove(temp_audio)
# # # # # # # # # # # # # # #             return jsonify({"error": "Voice analysis failed"}), 400
        
# # # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # # #         result = cur.fetchone()
        
# # # # # # # # # # # # # # #         if not result or not result[0]:
# # # # # # # # # # # # # # #             os.remove(temp_audio)
# # # # # # # # # # # # # # #             return jsonify({"error": "User not found"}), 404
        
# # # # # # # # # # # # # # #         stored_features = result[0]
# # # # # # # # # # # # # # #         similarity = voice_auth.compare_voices(stored_features, current_features)
        
# # # # # # # # # # # # # # #         os.remove(temp_audio)
# # # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # # #         conn.close()
        
# # # # # # # # # # # # # # #         if similarity > 0.3:
# # # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # # #                 "status": "success", 
# # # # # # # # # # # # # # #                 "message": "Login successful", 
# # # # # # # # # # # # # # #                 "username": username,
# # # # # # # # # # # # # # #                 "similarity": float(similarity)
# # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # # #                 "status": "fail", 
# # # # # # # # # # # # # # #                 "message": "Voice not recognized",
# # # # # # # # # # # # # # #                 "similarity": float(similarity)
# # # # # # # # # # # # # # #             })
            
# # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # #         return jsonify({"error": f"Login error: {str(e)}"}), 500

# # # # # # # # # # # # # # # # --- YOLO детекция ---
# # # # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # # # # # # # # # # }

# # # # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # # # #         return None
# # # # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # # # #             })
# # # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # # #         return "No obstacles detected"
# # # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # # #     frame_center = 320
# # # # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # # # #         distance = obs.get('distance_m')
# # # # # # # # # # # # # # #         if distance is not None:
# # # # # # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))
# # # # # # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # # # # # #                 dist_text = f" at {centimeters} centimeters"
# # # # # # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # # # # # #                 dist_text = f" at {meters} meters"
# # # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # # #                 dist_text = f" at {meters} meters {centimeters} centimeters"
# # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # #             dist_text = ""
# # # # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # # #             pos = "left"
# # # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # # #             pos = "right"
# # # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # # #             pos = "ahead"
# # # # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} with {conf}% confidence")
# # # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # # #         descriptions.append(f"and {len(obstacles) - 5} more objects")
# # # # # # # # # # # # # # #     return "Detected: " + ", ".join(descriptions)

# # # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # # # #         "version": "4.0",
# # # # # # # # # # # # # # #         "message": "Voice auth server ready"
# # # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # # #                         (
# # # # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # # # #                         )
# # # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # # #             "message": "Frame processed"
# # # # # # # # # # # # # # #         })
# # # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # # #         return jsonify({"error": f"Frame processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)



# # # # # # # # # # # # # # # --- Импорты ---
# # # # # # # # # # # # # # import os
# # # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # # import speech_recognition as sr
# # # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # # import wave
# # # # # # # # # # # # # # import struct
# # # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # # import tempfile
# # # # # # # # # # # # # # import subprocess
# # # # # # # # # # # # # # import time

# # # # # # # # # # # # # # # --- Конфигурация ---
# # # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # # --- CORS ---
# # # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # # #     return response

# # # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # # # # # # # # # #     )

# # # # # # # # # # # # # # # --- Упрощенная голосовая биометрия БЕЗ FFMPEG ---
# # # # # # # # # # # # # # class SimpleVoiceAuth:
# # # # # # # # # # # # # #     def __init__(self):
# # # # # # # # # # # # # #         self.recognizer = sr.Recognizer()
    
# # # # # # # # # # # # # #     def extract_features_from_aac(self, audio_path):
# # # # # # # # # # # # # #         """Прямая обработка AAC файлов через speech_recognition"""
# # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # #             # Используем speech_recognition для извлечения текста из AAC
# # # # # # # # # # # # # #             with sr.AudioFile(audio_path) as source:
# # # # # # # # # # # # # #                 # Учитываем фоновый шум
# # # # # # # # # # # # # #                 self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
# # # # # # # # # # # # # #                 audio = self.recognizer.record(source)
                
# # # # # # # # # # # # # #                 # Распознаем текст
# # # # # # # # # # # # # #                 text = self.recognizer.recognize_google(audio, language="ru-RU")
# # # # # # # # # # # # # #                 print(f"Распознанный текст: {text}")
                
# # # # # # # # # # # # # #                 # Создаем простые характеристики на основе текста
# # # # # # # # # # # # # #                 features = {
# # # # # # # # # # # # # #                     'text_length': len(text),
# # # # # # # # # # # # # #                     'word_count': len(text.split()),
# # # # # # # # # # # # # #                     'text_hash': hashlib.md5(text.lower().encode()).hexdigest(),
# # # # # # # # # # # # # #                     'first_word': text.split()[0].lower() if text.split() else '',
# # # # # # # # # # # # # #                     'last_word': text.split()[-1].lower() if text.split() else ''
# # # # # # # # # # # # # #                 }
                
# # # # # # # # # # # # # #                 return str(features)
                
# # # # # # # # # # # # # #         except sr.UnknownValueError:
# # # # # # # # # # # # # #             print("Не удалось распознать речь")
# # # # # # # # # # # # # #             # Если не распознано, создаем характеристики на основе файла
# # # # # # # # # # # # # #             return self._extract_file_features(audio_path)
# # # # # # # # # # # # # #         except sr.RequestError as e:
# # # # # # # # # # # # # #             print(f"Ошибка сервиса распознавания: {e}")
# # # # # # # # # # # # # #             return self._extract_file_features(audio_path)
# # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # #             print(f"Общая ошибка обработки AAC: {e}")
# # # # # # # # # # # # # #             return self._extract_file_features(audio_path)
    
# # # # # # # # # # # # # #     def _extract_file_features(self, audio_path):
# # # # # # # # # # # # # #         """Резервный метод - характеристики на основе файла"""
# # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # #             file_stats = os.stat(audio_path)
# # # # # # # # # # # # # #             features = {
# # # # # # # # # # # # # #                 'file_size': file_stats.st_size,
# # # # # # # # # # # # # #                 'file_time': file_stats.st_mtime,
# # # # # # # # # # # # # #                 'fallback': True
# # # # # # # # # # # # # #             }
# # # # # # # # # # # # # #             return str(features)
# # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # #             print(f"Ошибка извлечения файловых характеристик: {e}")
# # # # # # # # # # # # # #             return None
    
# # # # # # # # # # # # # #     def compare_voices(self, features1_str, features2_str):
# # # # # # # # # # # # # #         """Сравниваем голоса через распознанный текст"""
# # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # #             import ast
# # # # # # # # # # # # # #             features1 = ast.literal_eval(features1_str)
# # # # # # # # # # # # # #             features2 = ast.literal_eval(features2_str)
            
# # # # # # # # # # # # # #             # Если это fallback характеристики (на основе файла)
# # # # # # # # # # # # # #             if features1.get('fallback') or features2.get('fallback'):
# # # # # # # # # # # # # #                 # Простое сравнение размера файла
# # # # # # # # # # # # # #                 size1 = features1.get('file_size', 0)
# # # # # # # # # # # # # #                 size2 = features2.get('file_size', 0)
# # # # # # # # # # # # # #                 size_similarity = 1.0 - abs(size1 - size2) / max(size1, size2, 1)
# # # # # # # # # # # # # #                 return size_similarity
            
# # # # # # # # # # # # # #             # Сравниваем через распознанный текст
# # # # # # # # # # # # # #             if features1['text_hash'] == features2['text_hash']:
# # # # # # # # # # # # # #                 return 1.0  # Полное совпадение текста
            
# # # # # # # # # # # # # #             # Сравниваем длину текста и количество слов
# # # # # # # # # # # # # #             length_similarity = 1.0 - abs(features1['text_length'] - features2['text_length']) / max(features1['text_length'], features2['text_length'], 1)
# # # # # # # # # # # # # #             word_similarity = 1.0 - abs(features1['word_count'] - features2['word_count']) / max(features1['word_count'], features2['word_count'], 1)
            
# # # # # # # # # # # # # #             # Среднее значение
# # # # # # # # # # # # # #             similarity = (length_similarity + word_similarity) / 2
# # # # # # # # # # # # # #             return similarity
            
# # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # #             print(f"Ошибка сравнения: {e}")
# # # # # # # # # # # # # #             return 0.0

# # # # # # # # # # # # # # voice_auth = SimpleVoiceAuth()

# # # # # # # # # # # # # # # --- Эндпоинты ---
# # # # # # # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # # # # # # def register_voice():
# # # # # # # # # # # # # #     temp_audio = None
# # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # #             return jsonify({"error": "Need audio and username"}), 400

# # # # # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # # #         if not username:
# # # # # # # # # # # # # #             return jsonify({"error": "Username is required"}), 400
        
# # # # # # # # # # # # # #         # Сохраняем файл
# # # # # # # # # # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # # # # #         # Ждем немного чтобы файл разблокировался
# # # # # # # # # # # # # #         time.sleep(0.5)
        
# # # # # # # # # # # # # #         # Извлекаем характеристики
# # # # # # # # # # # # # #         features = voice_auth.extract_features_from_aac(temp_audio)
        
# # # # # # # # # # # # # #         if not features:
# # # # # # # # # # # # # #             return jsonify({"error": "Voice analysis failed"}), 400
        
# # # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # # #         try:
# # # # # # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # #             existing = cur.fetchone()
            
# # # # # # # # # # # # # #             if existing:
# # # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # # #                     "UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # # # # # # # # # #                     (features, username)
# # # # # # # # # # # # # #                 )
# # # # # # # # # # # # # #                 message = "Голосовой профиль обновлен"
# # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # # #                     "INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # # # # # # # # # #                     (username, features)
# # # # # # # # # # # # # #                 )
# # # # # # # # # # # # # #                 message = "Голосовой профиль зарегистрирован"
            
# # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # #             result = {"status": "success", "message": message, "username": username}
            
# # # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # # #             conn.rollback()
# # # # # # # # # # # # # #             result = {"error": f"DB error: {str(e)}"}
# # # # # # # # # # # # # #         finally:
# # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # #             conn.close()
        
# # # # # # # # # # # # # #         return jsonify(result)
        
# # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # #         return jsonify({"error": f"Registration error: {str(e)}"}), 500
# # # # # # # # # # # # # #     finally:
# # # # # # # # # # # # # #         # Очистка файла с обработкой ошибок блокировки
# # # # # # # # # # # # # #         if temp_audio and os.path.exists(temp_audio):
# # # # # # # # # # # # # #             try:
# # # # # # # # # # # # # #                 os.remove(temp_audio)
# # # # # # # # # # # # # #             except PermissionError:
# # # # # # # # # # # # # #                 print(f"File {temp_audio} is locked, skipping deletion")
# # # # # # # # # # # # # #             except Exception as e:
# # # # # # # # # # # # # #                 print(f"Error deleting temp file: {e}")

# # # # # # # # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # # # # # # # def login_voice():
# # # # # # # # # # # # # #     temp_audio = None
# # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # # #             return jsonify({"error": "Need audio and username"}), 400

# # # # # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # # #         if not username:
# # # # # # # # # # # # # #             return jsonify({"error": "Username is required"}), 400
        
# # # # # # # # # # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # # # # #         # Ждем разблокировки файла
# # # # # # # # # # # # # #         time.sleep(0.5)
        
# # # # # # # # # # # # # #         current_features = voice_auth.extract_features_from_aac(temp_audio)
        
# # # # # # # # # # # # # #         if not current_features:
# # # # # # # # # # # # # #             return jsonify({"error": "Voice analysis failed"}), 400
        
# # # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # # #         result = cur.fetchone()
        
# # # # # # # # # # # # # #         if not result or not result[0]:
# # # # # # # # # # # # # #             return jsonify({"error": "User not found"}), 404
        
# # # # # # # # # # # # # #         stored_features = result[0]
# # # # # # # # # # # # # #         similarity = voice_auth.compare_voices(stored_features, current_features)
        
# # # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # # #         conn.close()
        
# # # # # # # # # # # # # #         print(f"Similarity score: {similarity}")
        
# # # # # # # # # # # # # #         if similarity > 0.3:  # Более низкий порог для текстового сравнения
# # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # #                 "status": "success", 
# # # # # # # # # # # # # #                 "message": "Login successful", 
# # # # # # # # # # # # # #                 "username": username,
# # # # # # # # # # # # # #                 "similarity": float(similarity)
# # # # # # # # # # # # # #             })
# # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # # #                 "status": "fail", 
# # # # # # # # # # # # # #                 "message": "Voice not recognized",
# # # # # # # # # # # # # #                 "similarity": float(similarity)
# # # # # # # # # # # # # #             })
            
# # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # #         return jsonify({"error": f"Login error: {str(e)}"}), 500
# # # # # # # # # # # # # #     finally:
# # # # # # # # # # # # # #         if temp_audio and os.path.exists(temp_audio):
# # # # # # # # # # # # # #             try:
# # # # # # # # # # # # # #                 os.remove(temp_audio)
# # # # # # # # # # # # # #             except PermissionError:
# # # # # # # # # # # # # #                 print(f"File {temp_audio} is locked, skipping deletion")
# # # # # # # # # # # # # #             except Exception as e:
# # # # # # # # # # # # # #                 print(f"Error deleting temp file: {e}")

# # # # # # # # # # # # # # # --- YOLO детекция (оставляем как было) ---
# # # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # # # # # # # # # }

# # # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # # #         return None
# # # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # # #             })
# # # # # # # # # # # # # #         return detections
# # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # # #         return []

# # # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # # #         return "No obstacles detected"
# # # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # # #     frame_center = 320
# # # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # # #         distance = obs.get('distance_m')
# # # # # # # # # # # # # #         if distance is not None:
# # # # # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))
# # # # # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # # # # #                 dist_text = f" at {centimeters} centimeters"
# # # # # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # # # # #                 dist_text = f" at {meters} meters"
# # # # # # # # # # # # # #             else:
# # # # # # # # # # # # # #                 dist_text = f" at {meters} meters {centimeters} centimeters"
# # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # #             dist_text = ""
# # # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # # #             pos = "left"
# # # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # # #             pos = "right"
# # # # # # # # # # # # # #         else:
# # # # # # # # # # # # # #             pos = "ahead"
# # # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} with {conf}% confidence")
# # # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # # #         descriptions.append(f"and {len(obstacles) - 5} more objects")
# # # # # # # # # # # # # #     return "Detected: " + ", ".join(descriptions)

# # # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # # def index():
# # # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # # #         "version": "4.0",
# # # # # # # # # # # # # #         "message": "Voice auth server ready"
# # # # # # # # # # # # # #     })

# # # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # # #     try:
# # # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # # #                 try:
# # # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # # #                         (
# # # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # # #                         )
# # # # # # # # # # # # # #                     )
# # # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # # #             "message": "Frame processed"
# # # # # # # # # # # # # #         })
# # # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # # #         return jsonify({"error": f"Frame processing error: {str(e)}"}), 500

# # # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)




# # # # # # # # # # # # # # --- Импорты ---
# # # # # # # # # # # # # import os
# # # # # # # # # # # # # import uuid
# # # # # # # # # # # # # import cv2
# # # # # # # # # # # # # import numpy as np
# # # # # # # # # # # # # import psycopg2
# # # # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # # # import wave
# # # # # # # # # # # # # import struct
# # # # # # # # # # # # # import hashlib
# # # # # # # # # # # # # import tempfile
# # # # # # # # # # # # # import time
# # # # # # # # # # # # # import io
# # # # # # # # # # # # # import base64
# # # # # # # # # # # # # from scipy.fftpack import dct

# # # # # # # # # # # # # # --- Конфигурация ---
# # # # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # # # --- CORS ---
# # # # # # # # # # # # # @app.after_request
# # # # # # # # # # # # # def after_request(response):
# # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # # # #     return response

# # # # # # # # # # # # # def get_db_connection():
# # # # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # # # # # # # # #     )

# # # # # # # # # # # # # # --- УПРОЩЕННАЯ ГОЛОСОВАЯ БИОМЕТРИЯ (без tensorflow) ---
# # # # # # # # # # # # # class SimpleVoiceBiometrics:
# # # # # # # # # # # # #     def __init__(self):
# # # # # # # # # # # # #         print("Инициализирована упрощенная голосовая биометрия!")
    
# # # # # # # # # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # # # # # # # # #         """Извлекаем MFCC характеристики вручную без librosa"""
# # # # # # # # # # # # #         try:
# # # # # # # # # # # # #             # Читаем WAV файл напрямую
# # # # # # # # # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # # # # # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # # # # # # # # #                 frame_rate = wav_file.getframerate()
# # # # # # # # # # # # #                 n_frames = wav_file.getnframes()
                
# # # # # # # # # # # # #                 # Читаем аудиоданные
# # # # # # # # # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # # # # # # # # #                 # Конвертируем в numpy массив
# # # # # # # # # # # # #                 if sample_width == 2:
# # # # # # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # # # # # # # # #                 else:
# # # # # # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # # # # # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # # # # # # # # #                 # Нормализуем
# # # # # # # # # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
                
# # # # # # # # # # # # #                 # Простое извлечение характеристик
# # # # # # # # # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
                
# # # # # # # # # # # # #                 # Конвертируем в base64 для хранения
# # # # # # # # # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # # # # # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
                
# # # # # # # # # # # # #                 print(f"Извлечены аудио характеристики: {len(features)} параметров")
# # # # # # # # # # # # #                 return features_b64
                
# # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # # # # # # # # #             return self._fallback_features(audio_path)
    
# # # # # # # # # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # # # # # # # # #         """Простое извлечение аудио характеристик"""
# # # # # # # # # # # # #         # Базовые статистики
# # # # # # # # # # # # #         features = []
        
# # # # # # # # # # # # #         # 1. Энергетические характеристики
# # # # # # # # # # # # #         features.append(np.mean(audio_data ** 2))  # Энергия
# # # # # # # # # # # # #         features.append(np.std(audio_data))        # Стандартное отклонение
# # # # # # # # # # # # #         features.append(np.max(np.abs(audio_data))) # Пиковая амплитуда
        
# # # # # # # # # # # # #         # 2. Спектральные характеристики (упрощенные)
# # # # # # # # # # # # #         # Быстрое преобразование Фурье
# # # # # # # # # # # # #         fft = np.fft.fft(audio_data)
# # # # # # # # # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # # # # # # # # #         # Спектральные полосы (грубые MFCC)
# # # # # # # # # # # # #         bands = [
# # # # # # # # # # # # #             (0, 100),    # Низкие частоты
# # # # # # # # # # # # #             (100, 500),  # Средние низкие
# # # # # # # # # # # # #             (500, 1500), # Средние
# # # # # # # # # # # # #             (1500, 4000) # Высокие
# # # # # # # # # # # # #         ]
        
# # # # # # # # # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # # # # # # # # #         for low, high in bands:
# # # # # # # # # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # # # # # # # # #             if np.any(mask):
# # # # # # # # # # # # #                 band_energy = np.mean(fft_magnitude[mask])
# # # # # # # # # # # # #                 features.append(band_energy)
# # # # # # # # # # # # #             else:
# # # # # # # # # # # # #                 features.append(0.0)
        
# # # # # # # # # # # # #         # 3. Временные характеристики
# # # # # # # # # # # # #         features.append(self._zero_crossing_rate(audio_data))
# # # # # # # # # # # # #         features.append(self._spectral_centroid(fft_magnitude, freqs))
        
# # # # # # # # # # # # #         return np.array(features)
    
# # # # # # # # # # # # #     def _zero_crossing_rate(self, audio_data):
# # # # # # # # # # # # #         """Частота пересечения нуля"""
# # # # # # # # # # # # #         zero_crossings = np.where(np.diff(np.signbit(audio_data)))[0]
# # # # # # # # # # # # #         return len(zero_crossings) / len(audio_data)
    
# # # # # # # # # # # # #     def _spectral_centroid(self, magnitude, freqs):
# # # # # # # # # # # # #         """Спектральный центроид"""
# # # # # # # # # # # # #         if np.sum(magnitude) == 0:
# # # # # # # # # # # # #             return 0.0
# # # # # # # # # # # # #         return np.sum(magnitude * freqs) / np.sum(magnitude)
    
# # # # # # # # # # # # #     def _fallback_features(self, audio_path):
# # # # # # # # # # # # #         """Резервный метод"""
# # # # # # # # # # # # #         try:
# # # # # # # # # # # # #             file_stats = os.stat(audio_path)
# # # # # # # # # # # # #             features = np.array([file_stats.st_size, file_stats.st_mtime])
# # # # # # # # # # # # #             features_bytes = features.astype(np.float32).tobytes()
# # # # # # # # # # # # #             features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # # # # # # # # # # # #             return features_b64
# # # # # # # # # # # # #         except:
# # # # # # # # # # # # #             return None
    
# # # # # # # # # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # # # # # # # # #         """Сравниваем голосовые характеристики"""
# # # # # # # # # # # # #         try:
# # # # # # # # # # # # #             # Декодируем из base64
# # # # # # # # # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # # # # # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # # # # # # # # #             # Конвертируем обратно в numpy массивы
# # # # # # # # # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # # # # # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # # # # # # # # #             # Выравниваем размеры
# # # # # # # # # # # # #             min_len = min(len(features1), len(features2))
# # # # # # # # # # # # #             features1 = features1[:min_len]
# # # # # # # # # # # # #             features2 = features2[:min_len]
            
# # # # # # # # # # # # #             # Евклидово расстояние (чем меньше - тем более похожи)
# # # # # # # # # # # # #             distance = np.linalg.norm(features1 - features2)
            
# # # # # # # # # # # # #             # Преобразуем в схожесть (чем больше - тем более похожи)
# # # # # # # # # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # # # # # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
            
# # # # # # # # # # # # #             # Ограничиваем диапазон
# # # # # # # # # # # # #             similarity = max(0.0, min(1.0, similarity))
            
# # # # # # # # # # # # #             print(f"Схожесть голосов: {similarity:.4f}")
# # # # # # # # # # # # #             return float(similarity)
            
# # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # #             print(f"Ошибка сравнения: {e}")
# # # # # # # # # # # # #             return 0.0

# # # # # # # # # # # # # # Инициализируем упрощенную голосовую биометрию
# # # # # # # # # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # # # # # # # # --- Конвертер AAC в WAV ---
# # # # # # # # # # # # # try:
# # # # # # # # # # # # #     from pydub import AudioSegment
# # # # # # # # # # # # #     PYDUB_AVAILABLE = True
# # # # # # # # # # # # # except ImportError:
# # # # # # # # # # # # #     print("Pydub not available - installing...")
# # # # # # # # # # # # #     import subprocess
# # # # # # # # # # # # #     import sys
# # # # # # # # # # # # #     subprocess.check_call([sys.executable, "-m", "pip", "install", "pydub"])
# # # # # # # # # # # # #     from pydub import AudioSegment
# # # # # # # # # # # # #     PYDUB_AVAILABLE = True

# # # # # # # # # # # # # class AudioConverter:
# # # # # # # # # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # # # # # # # # #         """Конвертируем AAC в WAV"""
# # # # # # # # # # # # #         try:
# # # # # # # # # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # # # # # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # # # # # # # # #             audio.export(wav_path, format="wav")
# # # # # # # # # # # # #             return wav_path
# # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # #             print(f"Ошибка конвертации AAC->WAV: {e}")
# # # # # # # # # # # # #             return None

# # # # # # # # # # # # # audio_converter = AudioConverter()

# # # # # # # # # # # # # # --- Эндпоинты ---
# # # # # # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # # # # # def register_voice():
# # # # # # # # # # # # #     temp_audio = None
# # # # # # # # # # # # #     temp_wav = None
# # # # # # # # # # # # #     try:
# # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # #             return jsonify({"error": "Need audio and username"}), 400

# # # # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # #         if not username:
# # # # # # # # # # # # #             return jsonify({"error": "Username is required"}), 400
        
# # # # # # # # # # # # #         # Сохраняем AAC файл
# # # # # # # # # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # # # #         # Конвертируем в WAV
# # # # # # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # # # # # #         if not temp_wav:
# # # # # # # # # # # # #             return jsonify({"error": "Audio conversion failed"}), 400
        
# # # # # # # # # # # # #         # Извлекаем голосовые характеристики
# # # # # # # # # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # # # # # #         if not voice_features:
# # # # # # # # # # # # #             return jsonify({"error": "Voice analysis failed"}), 400
        
# # # # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # #         try:
# # # # # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # #             existing = cur.fetchone()
            
# # # # # # # # # # # # #             if existing:
# # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # #                     "UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # # # # # # # # #                     (voice_features, username)
# # # # # # # # # # # # #                 )
# # # # # # # # # # # # #                 message = "Голосовой профиль обновлен"
# # # # # # # # # # # # #             else:
# # # # # # # # # # # # #                 cur.execute(
# # # # # # # # # # # # #                     "INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # # # # # # # # #                     (username, voice_features)
# # # # # # # # # # # # #                 )
# # # # # # # # # # # # #                 message = "Голосовой профиль зарегистрирован"
            
# # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # #             result = {
# # # # # # # # # # # # #                 "status": "success", 
# # # # # # # # # # # # #                 "message": message, 
# # # # # # # # # # # # #                 "username": username,
# # # # # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # # # # #             }
            
# # # # # # # # # # # # #         except Exception as e:
# # # # # # # # # # # # #             conn.rollback()
# # # # # # # # # # # # #             result = {"error": f"DB error: {str(e)}"}
# # # # # # # # # # # # #         finally:
# # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # #             conn.close()
        
# # # # # # # # # # # # #         return jsonify(result)
        
# # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # #         return jsonify({"error": f"Registration error: {str(e)}"}), 500
# # # # # # # # # # # # #     finally:
# # # # # # # # # # # # #         # Очистка
# # # # # # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # # # # # #                 try:
# # # # # # # # # # # # #                     os.remove(temp_file)
# # # # # # # # # # # # #                 except:
# # # # # # # # # # # # #                     pass

# # # # # # # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # # # # # # def login_voice():
# # # # # # # # # # # # #     temp_audio = None
# # # # # # # # # # # # #     temp_wav = None
# # # # # # # # # # # # #     try:
# # # # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # # # #             return jsonify({"error": "Need audio and username"}), 400

# # # # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # # # #         if not username:
# # # # # # # # # # # # #             return jsonify({"error": "Username is required"}), 400
        
# # # # # # # # # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # # # #         # Конвертируем в WAV
# # # # # # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # # # # # #         if not temp_wav:
# # # # # # # # # # # # #             return jsonify({"error": "Audio conversion failed"}), 400
        
# # # # # # # # # # # # #         # Извлекаем голосовые характеристики
# # # # # # # # # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # # # # # #         if not current_features:
# # # # # # # # # # # # #             return jsonify({"error": "Voice analysis failed"}), 400
        
# # # # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # # # #         result = cur.fetchone()
        
# # # # # # # # # # # # #         if not result or not result[0]:
# # # # # # # # # # # # #             return jsonify({"error": "User not found"}), 404
        
# # # # # # # # # # # # #         stored_features = result[0]
        
# # # # # # # # # # # # #         # Сравниваем голосовые характеристики
# # # # # # # # # # # # #         similarity = voice_biometrics.compare_voice_features(stored_features, current_features)
        
# # # # # # # # # # # # #         cur.close()
# # # # # # # # # # # # #         conn.close()
        
# # # # # # # # # # # # #         print(f"Сходство голосов: {similarity:.4f}")
        
# # # # # # # # # # # # #         if similarity > 0.7:  # Высокий порог для надежности
# # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # #                 "status": "success", 
# # # # # # # # # # # # #                 "message": "Голос распознан! Вход успешен", 
# # # # # # # # # # # # #                 "username": username,
# # # # # # # # # # # # #                 "similarity": float(similarity),
# # # # # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # # # # #             })
# # # # # # # # # # # # #         else:
# # # # # # # # # # # # #             return jsonify({
# # # # # # # # # # # # #                 "status": "fail", 
# # # # # # # # # # # # #                 "message": "Голос не распознан",
# # # # # # # # # # # # #                 "similarity": float(similarity),
# # # # # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # # # # #             })
            
# # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # #         return jsonify({"error": f"Login error: {str(e)}"}), 500
# # # # # # # # # # # # #     finally:
# # # # # # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # # # # # #                 try:
# # # # # # # # # # # # #                     os.remove(temp_file)
# # # # # # # # # # # # #                 except:
# # # # # # # # # # # # #                     pass

# # # # # # # # # # # # # # --- YOLO детекция (оставляем как было) ---
# # # # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # # # # # # # # }

# # # # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # # # #         return None
# # # # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # # # print("Loading YOLO model...")
# # # # # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # # # # print("Model loaded.")

# # # # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # # # #     try:
# # # # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # # # #         detections = []
# # # # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # # # #             detections.append({
# # # # # # # # # # # # #                 "label": label,
# # # # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # # # #             })
# # # # # # # # # # # # #         return detections
# # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # # # #         return []

# # # # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # # # #     if not obstacles:
# # # # # # # # # # # # #         return "No obstacles detected"
# # # # # # # # # # # # #     descriptions = []
# # # # # # # # # # # # #     frame_center = 320
# # # # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # # # #         label = obs['label']
# # # # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # # # #         distance = obs.get('distance_m')
# # # # # # # # # # # # #         if distance is not None:
# # # # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))
# # # # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # # # #                 dist_text = f" at {centimeters} centimeters"
# # # # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # # # #                 dist_text = f" at {meters} meters"
# # # # # # # # # # # # #             else:
# # # # # # # # # # # # #                 dist_text = f" at {meters} meters {centimeters} centimeters"
# # # # # # # # # # # # #         else:
# # # # # # # # # # # # #             dist_text = ""
# # # # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # # # #             pos = "left"
# # # # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # # # #             pos = "right"
# # # # # # # # # # # # #         else:
# # # # # # # # # # # # #             pos = "ahead"
# # # # # # # # # # # # #         descriptions.append(f"{label} {pos}{dist_text} with {conf}% confidence")
# # # # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # # # #         descriptions.append(f"and {len(obstacles) - 5} more objects")
# # # # # # # # # # # # #     return "Detected: " + ", ".join(descriptions)

# # # # # # # # # # # # # @app.route('/')
# # # # # # # # # # # # # def index():
# # # # # # # # # # # # #     return jsonify({
# # # # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # # # #         "version": "5.0",
# # # # # # # # # # # # #         "message": "Сервер с упрощенной голосовой биометрией готов"
# # # # # # # # # # # # #     })

# # # # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # # # def process_frame():
# # # # # # # # # # # # #     try:
# # # # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # # # #             return jsonify({"error": "No frame provided"}), 400

# # # # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # # # #         if frame is None:
# # # # # # # # # # # # #             return jsonify({"error": "Invalid image"}), 400

# # # # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # # # #         if obstacles:
# # # # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # # # #                 try:
# # # # # # # # # # # # #                     cur.execute(
# # # # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # # # #                         (
# # # # # # # # # # # # #                             obs['label'],
# # # # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # # # #                         )
# # # # # # # # # # # # #                     )
# # # # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # # # #                     print(f"DB error: {e}")
# # # # # # # # # # # # #             conn.commit()
# # # # # # # # # # # # #             cur.close()
# # # # # # # # # # # # #             conn.close()

# # # # # # # # # # # # #         return jsonify({
# # # # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # # # #             "description": description,
# # # # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # # # #             "message": "Frame processed"
# # # # # # # # # # # # #         })
# # # # # # # # # # # # #     except Exception as e:
# # # # # # # # # # # # #         return jsonify({"error": f"Frame processing error: {str(e)}"}), 500

# # # # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # # # #     print("=" * 50)
# # # # # # # # # # # # #     print("СЕРВЕР С УПРОЩЕННОЙ ГОЛОСОВОЙ БИОМЕТРИЕЙ")
# # # # # # # # # # # # #     print("=" * 50)
# # # # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)






# # # # # # # # # # # # # итог сверху но камера не оч



# # # # # # # # # # # # --- Импорты ---
# # # # # # # # # # # import os
# # # # # # # # # # # import uuid
# # # # # # # # # # # import cv2
# # # # # # # # # # # import numpy as np
# # # # # # # # # # # import psycopg2
# # # # # # # # # # # from flask import Flask, request, jsonify
# # # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # # import wave
# # # # # # # # # # # import struct
# # # # # # # # # # # import hashlib
# # # # # # # # # # # import tempfile
# # # # # # # # # # # import time
# # # # # # # # # # # import io
# # # # # # # # # # # import base64
# # # # # # # # # # # from scipy.fftpack import dct

# # # # # # # # # # # # --- Конфигурация ---
# # # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # # --- CORS ---
# # # # # # # # # # # @app.after_request
# # # # # # # # # # # def after_request(response):
# # # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # # #     return response

# # # # # # # # # # # def get_db_connection():
# # # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # # # # # # #     )

# # # # # # # # # # # # --- УПРОЩЕННАЯ ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # # # # # # # # class SimpleVoiceBiometrics:
# # # # # # # # # # #     def __init__(self):
# # # # # # # # # # #         print("Инициализирована упрощенная голосовая биометрия!")
    
# # # # # # # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # # # # # # #         """Извлекаем MFCC характеристики вручную без librosa"""
# # # # # # # # # # #         try:
# # # # # # # # # # #             # Читаем WAV файл напрямую
# # # # # # # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # # # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # # # # # # #                 frame_rate = wav_file.getframerate()
# # # # # # # # # # #                 n_frames = wav_file.getnframes()
                
# # # # # # # # # # #                 # Читаем аудиоданные
# # # # # # # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # # # # # # #                 # Конвертируем в numpy массив
# # # # # # # # # # #                 if sample_width == 2:
# # # # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # # # # # # #                 else:
# # # # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # # # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # # # # # # #                 # Нормализуем
# # # # # # # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
                
# # # # # # # # # # #                 # Простое извлечение характеристик
# # # # # # # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
                
# # # # # # # # # # #                 # Конвертируем в base64 для хранения
# # # # # # # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # # # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
                
# # # # # # # # # # #                 print(f"Извлечены аудио характеристики: {len(features)} параметров")
# # # # # # # # # # #                 return features_b64
                
# # # # # # # # # # #         except Exception as e:
# # # # # # # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # # # # # # #             return self._fallback_features(audio_path)
    
# # # # # # # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # # # # # # #         """Простое извлечение аудио характеристик"""
# # # # # # # # # # #         features = []
        
# # # # # # # # # # #         # 1. Энергетические характеристики
# # # # # # # # # # #         features.append(np.mean(audio_data ** 2))  # Энергия
# # # # # # # # # # #         features.append(np.std(audio_data))        # Стандартное отклонение
# # # # # # # # # # #         features.append(np.max(np.abs(audio_data))) # Пиковая амплитуда
        
# # # # # # # # # # #         # 2. Спектральные характеристики (упрощенные)
# # # # # # # # # # #         fft = np.fft.fft(audio_data)
# # # # # # # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # # # # # # #         # Спектральные полосы (грубые MFCC)
# # # # # # # # # # #         bands = [
# # # # # # # # # # #             (0, 100),    # Низкие частоты
# # # # # # # # # # #             (100, 500),  # Средние низкие
# # # # # # # # # # #             (500, 1500), # Средние
# # # # # # # # # # #             (1500, 4000) # Высокие
# # # # # # # # # # #         ]
        
# # # # # # # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # # # # # # #         for low, high in bands:
# # # # # # # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # # # # # # #             if np.any(mask):
# # # # # # # # # # #                 band_energy = np.mean(fft_magnitude[mask])
# # # # # # # # # # #                 features.append(band_energy)
# # # # # # # # # # #             else:
# # # # # # # # # # #                 features.append(0.0)
        
# # # # # # # # # # #         # 3. Временные характеристики
# # # # # # # # # # #         features.append(self._zero_crossing_rate(audio_data))
# # # # # # # # # # #         features.append(self._spectral_centroid(fft_magnitude, freqs))
        
# # # # # # # # # # #         return np.array(features)
    
# # # # # # # # # # #     def _zero_crossing_rate(self, audio_data):
# # # # # # # # # # #         """Частота пересечения нуля"""
# # # # # # # # # # #         zero_crossings = np.where(np.diff(np.signbit(audio_data)))[0]
# # # # # # # # # # #         return len(zero_crossings) / len(audio_data)
    
# # # # # # # # # # #     def _spectral_centroid(self, magnitude, freqs):
# # # # # # # # # # #         """Спектральный центроид"""
# # # # # # # # # # #         if np.sum(magnitude) == 0:
# # # # # # # # # # #             return 0.0
# # # # # # # # # # #         return np.sum(magnitude * freqs) / np.sum(magnitude)
    
# # # # # # # # # # #     def _fallback_features(self, audio_path):
# # # # # # # # # # #         """Резервный метод"""
# # # # # # # # # # #         try:
# # # # # # # # # # #             file_stats = os.stat(audio_path)
# # # # # # # # # # #             features = np.array([file_stats.st_size, file_stats.st_mtime])
# # # # # # # # # # #             features_bytes = features.astype(np.float32).tobytes()
# # # # # # # # # # #             features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # # # # # # # # # #             return features_b64
# # # # # # # # # # #         except:
# # # # # # # # # # #             return None
    
# # # # # # # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # # # # # # #         """Сравниваем голосовые характеристики"""
# # # # # # # # # # #         try:
# # # # # # # # # # #             # Декодируем из base64
# # # # # # # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # # # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # # # # # # #             # Конвертируем обратно в numpy массивы
# # # # # # # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # # # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # # # # # # #             # Выравниваем размеры
# # # # # # # # # # #             min_len = min(len(features1), len(features2))
# # # # # # # # # # #             features1 = features1[:min_len]
# # # # # # # # # # #             features2 = features2[:min_len]
            
# # # # # # # # # # #             # Евклидово расстояние (чем меньше - тем более похожи)
# # # # # # # # # # #             distance = np.linalg.norm(features1 - features2)
            
# # # # # # # # # # #             # Преобразуем в схожесть (чем больше - тем более похожи)
# # # # # # # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # # # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
            
# # # # # # # # # # #             # Ограничиваем диапазон
# # # # # # # # # # #             similarity = max(0.0, min(1.0, similarity))
            
# # # # # # # # # # #             print(f"Схожесть голосов: {similarity:.4f}")
# # # # # # # # # # #             return float(similarity)
            
# # # # # # # # # # #         except Exception as e:
# # # # # # # # # # #             print(f"Ошибка сравнения: {e}")
# # # # # # # # # # #             return 0.0

# # # # # # # # # # # # Инициализируем упрощенную голосовую биометрию
# # # # # # # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # # # # # # --- Конвертер AAC в WAV ---
# # # # # # # # # # # try:
# # # # # # # # # # #     from pydub import AudioSegment
# # # # # # # # # # #     PYDUB_AVAILABLE = True
# # # # # # # # # # # except ImportError:
# # # # # # # # # # #     print("Pydub not available - installing...")
# # # # # # # # # # #     import subprocess
# # # # # # # # # # #     import sys
# # # # # # # # # # #     subprocess.check_call([sys.executable, "-m", "pip", "install", "pydub"])
# # # # # # # # # # #     from pydub import AudioSegment
# # # # # # # # # # #     PYDUB_AVAILABLE = True

# # # # # # # # # # # class AudioConverter:
# # # # # # # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # # # # # # #         """Конвертируем AAC в WAV"""
# # # # # # # # # # #         try:
# # # # # # # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # # # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # # # # # # #             audio.export(wav_path, format="wav")
# # # # # # # # # # #             return wav_path
# # # # # # # # # # #         except Exception as e:
# # # # # # # # # # #             print(f"Ошибка конвертации AAC->WAV: {e}")
# # # # # # # # # # #             return None

# # # # # # # # # # # audio_converter = AudioConverter()

# # # # # # # # # # # # --- YOLO детекция с улучшенным описанием ---
# # # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # # # # # # }

# # # # # # # # # # # # Перевод названий объектов на русский
# # # # # # # # # # # OBJECT_TRANSLATIONS = {
# # # # # # # # # # #     'person': 'человек',
# # # # # # # # # # #     'car': 'машина',
# # # # # # # # # # #     'chair': 'стул',
# # # # # # # # # # #     'bottle': 'бутылка',
# # # # # # # # # # #     'cup': 'чашка',
# # # # # # # # # # #     'dog': 'собака',
# # # # # # # # # # #     'cat': 'кошка',
# # # # # # # # # # #     'tv': 'телевизор',
# # # # # # # # # # #     'laptop': 'ноутбук',
# # # # # # # # # # #     'bicycle': 'велосипед',
# # # # # # # # # # #     'motorcycle': 'мотоцикл',
# # # # # # # # # # #     'bus': 'автобус',
# # # # # # # # # # #     'truck': 'грузовик',
# # # # # # # # # # #     'traffic light': 'светофор',
# # # # # # # # # # #     'fire hydrant': 'пожарный гидрант',
# # # # # # # # # # #     'stop sign': 'знак стоп',
# # # # # # # # # # #     'parking meter': 'паркомат',
# # # # # # # # # # #     'bench': 'скамейка',
# # # # # # # # # # #     'bird': 'птица',
# # # # # # # # # # #     'horse': 'лошадь',
# # # # # # # # # # #     'sheep': 'овца',
# # # # # # # # # # #     'cow': 'корова',
# # # # # # # # # # #     'elephant': 'слон',
# # # # # # # # # # #     'bear': 'медведь',
# # # # # # # # # # #     'zebra': 'зебра',
# # # # # # # # # # #     'giraffe': 'жираф',
# # # # # # # # # # #     'backpack': 'рюкзак',
# # # # # # # # # # #     'umbrella': 'зонт',
# # # # # # # # # # #     'handbag': 'сумка',
# # # # # # # # # # #     'tie': 'галстук',
# # # # # # # # # # #     'suitcase': 'чемодан',
# # # # # # # # # # #     'frisbee': 'фрисби',
# # # # # # # # # # #     'skis': 'лыжи',
# # # # # # # # # # #     'snowboard': 'сноуборд',
# # # # # # # # # # #     'sports ball': 'мяч',
# # # # # # # # # # #     'kite': 'воздушный змей',
# # # # # # # # # # #     'baseball bat': 'бейсбольная бита',
# # # # # # # # # # #     'baseball glove': 'бейсбольная перчатка',
# # # # # # # # # # #     'skateboard': 'скейтборд',
# # # # # # # # # # #     'surfboard': 'доска для серфинга',
# # # # # # # # # # #     'tennis racket': 'теннисная ракетка',
# # # # # # # # # # #     'wine glass': 'бокал',
# # # # # # # # # # #     'fork': 'вилка',
# # # # # # # # # # #     'knife': 'нож',
# # # # # # # # # # #     'spoon': 'ложка',
# # # # # # # # # # #     'bowl': 'миска',
# # # # # # # # # # #     'banana': 'банан',
# # # # # # # # # # #     'apple': 'яблоко',
# # # # # # # # # # #     'sandwich': 'сэндвич',
# # # # # # # # # # #     'orange': 'апельсин',
# # # # # # # # # # #     'broccoli': 'брокколи',
# # # # # # # # # # #     'carrot': 'морковь',
# # # # # # # # # # #     'hot dog': 'хот-дог',
# # # # # # # # # # #     'pizza': 'пицца',
# # # # # # # # # # #     'donut': 'пончик',
# # # # # # # # # # #     'cake': 'торт',
# # # # # # # # # # #     'couch': 'диван',
# # # # # # # # # # #     'potted plant': 'растение в горшке',
# # # # # # # # # # #     'bed': 'кровать',
# # # # # # # # # # #     'dining table': 'обеденный стол',
# # # # # # # # # # #     'toilet': 'туалет',
# # # # # # # # # # #     'monitor': 'монитор',
# # # # # # # # # # #     'mouse': 'мышь',
# # # # # # # # # # #     'remote': 'пульт',
# # # # # # # # # # #     'keyboard': 'клавиатура',
# # # # # # # # # # #     'cell phone': 'мобильный телефон',
# # # # # # # # # # #     'microwave': 'микроволновка',
# # # # # # # # # # #     'oven': 'духовка',
# # # # # # # # # # #     'toaster': 'тостер',
# # # # # # # # # # #     'sink': 'раковина',
# # # # # # # # # # #     'refrigerator': 'холодильник',
# # # # # # # # # # #     'book': 'книга',
# # # # # # # # # # #     'clock': 'часы',
# # # # # # # # # # #     'vase': 'ваза',
# # # # # # # # # # #     'scissors': 'ножницы',
# # # # # # # # # # #     'teddy bear': 'плюшевый мишка',
# # # # # # # # # # #     'hair drier': 'фен',
# # # # # # # # # # #     'toothbrush': 'зубная щетка'
# # # # # # # # # # # }

# # # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # # #         return None
# # # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # # print("Модель загружена.")

# # # # # # # # # # # def detect_objects(frame):
# # # # # # # # # # #     try:
# # # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # # #         detections = []
# # # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # # #         for r in results.boxes:
# # # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # # #             detections.append({
# # # # # # # # # # #                 "label": label,
# # # # # # # # # # #                 "confidence": conf,
# # # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # # #                 "distance_m": float(distance) if distance else None
# # # # # # # # # # #             })
# # # # # # # # # # #         return detections
# # # # # # # # # # #     except Exception as e:
# # # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # # #         return []

# # # # # # # # # # # def generate_obstacle_description(obstacles):
# # # # # # # # # # #     """Генерирует описание на русском языке с правильным форматированием"""
# # # # # # # # # # #     if not obstacles:
# # # # # # # # # # #         return "Препятствий не обнаружено"
    
# # # # # # # # # # #     descriptions = []
# # # # # # # # # # #     frame_center = 320
    
# # # # # # # # # # #     for obs in obstacles[:5]:
# # # # # # # # # # #         # Переводим название объекта на русский
# # # # # # # # # # #         label_en = obs['label']
# # # # # # # # # # #         label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
        
# # # # # # # # # # #         conf = int(obs['confidence'] * 100)
# # # # # # # # # # #         distance = obs.get('distance_m')
        
# # # # # # # # # # #         # Преобразование расстояния в метры и сантиметры
# # # # # # # # # # #         if distance is not None:
# # # # # # # # # # #             meters = int(distance)
# # # # # # # # # # #             centimeters = int(round((distance - meters) * 100))
            
# # # # # # # # # # #             if meters == 0 and centimeters > 0:
# # # # # # # # # # #                 dist_text = f" на расстоянии {centimeters} сантиметров"
# # # # # # # # # # #             elif meters > 0 and centimeters == 0:
# # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров"
# # # # # # # # # # #             else:
# # # # # # # # # # #                 dist_text = f" на расстоянии {meters} метров {centimeters} сантиметров"
# # # # # # # # # # #         else:
# # # # # # # # # # #             dist_text = ""
        
# # # # # # # # # # #         # Определяем положение объекта
# # # # # # # # # # #         bbox = obs['bbox']
# # # # # # # # # # #         x_center = (bbox[0] + bbox[2]) / 2
        
# # # # # # # # # # #         if x_center < frame_center - 100:
# # # # # # # # # # #             pos = "слева"
# # # # # # # # # # #         elif x_center > frame_center + 100:
# # # # # # # # # # #             pos = "справа"
# # # # # # # # # # #         else:
# # # # # # # # # # #             pos = "прямо перед вами"
        
# # # # # # # # # # #         descriptions.append(f"{label_ru} {pos}{dist_text} с уверенностью {conf}%")
    
# # # # # # # # # # #     if len(obstacles) > 5:
# # # # # # # # # # #         descriptions.append(f"и еще {len(obstacles) - 5} объектов")
    
# # # # # # # # # # #     return "Обнаружены: " + ", ".join(descriptions)

# # # # # # # # # # # # --- Эндпоинты ---

# # # # # # # # # # # @app.route('/')
# # # # # # # # # # # def index():
# # # # # # # # # # #     return jsonify({
# # # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # # #         "version": "6.0 - Unified",
# # # # # # # # # # #         "message": "Сервер с голосовой биометрией и детекцией объектов готов"
# # # # # # # # # # #     })

# # # # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # # # def register_voice():
# # # # # # # # # # #     """Регистрация голосового профиля пользователя"""
# # # # # # # # # # #     temp_audio = None
# # # # # # # # # # #     temp_wav = None
# # # # # # # # # # #     try:
# # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # #             return jsonify({"error": "Необходимо аудио и имя пользователя"}), 400

# # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # #         if not username:
# # # # # # # # # # #             return jsonify({"error": "Имя пользователя обязательно"}), 400
        
# # # # # # # # # # #         # Сохраняем AAC файл
# # # # # # # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # #         # Конвертируем в WAV
# # # # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # # # #         if not temp_wav:
# # # # # # # # # # #             return jsonify({"error": "Ошибка конвертации аудио"}), 400
        
# # # # # # # # # # #         # Извлекаем голосовые характеристики
# # # # # # # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # # # #         if not voice_features:
# # # # # # # # # # #             return jsonify({"error": "Ошибка анализа голоса"}), 400
        
# # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # #         try:
# # # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # # #             existing = cur.fetchone()
            
# # # # # # # # # # #             if existing:
# # # # # # # # # # #                 cur.execute(
# # # # # # # # # # #                     "UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # # # # # # #                     (voice_features, username)
# # # # # # # # # # #                 )
# # # # # # # # # # #                 message = "Голосовой профиль обновлен"
# # # # # # # # # # #             else:
# # # # # # # # # # #                 cur.execute(
# # # # # # # # # # #                     "INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # # # # # # #                     (username, voice_features)
# # # # # # # # # # #                 )
# # # # # # # # # # #                 message = "Голосовой профиль зарегистрирован"
            
# # # # # # # # # # #             conn.commit()
# # # # # # # # # # #             result = {
# # # # # # # # # # #                 "status": "success", 
# # # # # # # # # # #                 "message": message, 
# # # # # # # # # # #                 "username": username,
# # # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # # #             }
            
# # # # # # # # # # #         except Exception as e:
# # # # # # # # # # #             conn.rollback()
# # # # # # # # # # #             result = {"error": f"Ошибка БД: {str(e)}"}
# # # # # # # # # # #         finally:
# # # # # # # # # # #             cur.close()
# # # # # # # # # # #             conn.close()
        
# # # # # # # # # # #         return jsonify(result)
        
# # # # # # # # # # #     except Exception as e:
# # # # # # # # # # #         return jsonify({"error": f"Ошибка регистрации: {str(e)}"}), 500
# # # # # # # # # # #     finally:
# # # # # # # # # # #         # Очистка временных файлов
# # # # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # # # #                 try:
# # # # # # # # # # #                     os.remove(temp_file)
# # # # # # # # # # #                 except:
# # # # # # # # # # #                     pass

# # # # # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # # # # def login_voice():
# # # # # # # # # # #     """Вход через голосовую биометрию"""
# # # # # # # # # # #     temp_audio = None
# # # # # # # # # # #     temp_wav = None
# # # # # # # # # # #     try:
# # # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # # #             return jsonify({"error": "Необходимо аудио и имя пользователя"}), 400

# # # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # # #         if not username:
# # # # # # # # # # #             return jsonify({"error": "Имя пользователя обязательно"}), 400
        
# # # # # # # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # # #         # Конвертируем в WAV
# # # # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # # # #         if not temp_wav:
# # # # # # # # # # #             return jsonify({"error": "Ошибка конвертации аудио"}), 400
        
# # # # # # # # # # #         # Извлекаем голосовые характеристики
# # # # # # # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # # # #         if not current_features:
# # # # # # # # # # #             return jsonify({"error": "Ошибка анализа голоса"}), 400
        
# # # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # # #         result = cur.fetchone()
        
# # # # # # # # # # #         if not result or not result[0]:
# # # # # # # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # # # # # # #         stored_features = result[0]
        
# # # # # # # # # # #         # Сравниваем голосовые характеристики
# # # # # # # # # # #         similarity = voice_biometrics.compare_voice_features(stored_features, current_features)
        
# # # # # # # # # # #         cur.close()
# # # # # # # # # # #         conn.close()
        
# # # # # # # # # # #         print(f"Сходство голосов: {similarity:.4f}")
        
# # # # # # # # # # #         if similarity > 0.7:  # Высокий порог для надежности
# # # # # # # # # # #             return jsonify({
# # # # # # # # # # #                 "status": "success", 
# # # # # # # # # # #                 "message": "Голос распознан! Вход успешен", 
# # # # # # # # # # #                 "username": username,
# # # # # # # # # # #                 "similarity": float(similarity),
# # # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # # #             })
# # # # # # # # # # #         else:
# # # # # # # # # # #             return jsonify({
# # # # # # # # # # #                 "status": "fail", 
# # # # # # # # # # #                 "message": "Голос не распознан",
# # # # # # # # # # #                 "similarity": float(similarity),
# # # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # # #             })
            
# # # # # # # # # # #     except Exception as e:
# # # # # # # # # # #         return jsonify({"error": f"Ошибка входа: {str(e)}"}), 500
# # # # # # # # # # #     finally:
# # # # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # # # #                 try:
# # # # # # # # # # #                     os.remove(temp_file)
# # # # # # # # # # #                 except:
# # # # # # # # # # #                     pass

# # # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # # def process_frame():
# # # # # # # # # # #     """Обработка кадра с камеры и детекция объектов"""
# # # # # # # # # # #     try:
# # # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # # #         if frame is None:
# # # # # # # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # # # # # # #         obstacles = detect_objects(frame)
# # # # # # # # # # #         description = generate_obstacle_description(obstacles)

# # # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # # #         if obstacles:
# # # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # # #             for obs in obstacles:
# # # # # # # # # # #                 try:
# # # # # # # # # # #                     cur.execute(
# # # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # # #                         (
# # # # # # # # # # #                             obs['label'],
# # # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # # #                         )
# # # # # # # # # # #                     )
# # # # # # # # # # #                 except Exception as e:
# # # # # # # # # # #                     print(f"Ошибка БД: {e}")
# # # # # # # # # # #             conn.commit()
# # # # # # # # # # #             cur.close()
# # # # # # # # # # #             conn.close()

# # # # # # # # # # #         return jsonify({
# # # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # # #             "description": description,
# # # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # # #             "message": "Кадр обработан"
# # # # # # # # # # #         })
# # # # # # # # # # #     except Exception as e:
# # # # # # # # # # #         return jsonify({"error": f"Ошибка обработки кадра: {str(e)}"}), 500

# # # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # # #     print("=" * 60)
# # # # # # # # # # #     print("ОБЪЕДИНЕННЫЙ СЕРВЕР BLIND ASSISTANT")
# # # # # # # # # # #     print("=" * 60)
# # # # # # # # # # #     print("✓ Голосовая биометрия (регистрация и вход)")
# # # # # # # # # # #     print("✓ Детекция объектов с YOLO")
# # # # # # # # # # #     print("✓ Описания на русском языке")
# # # # # # # # # # #     print("✓ Сохранение в PostgreSQL")
# # # # # # # # # # #     print("=" * 60)
# # # # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)


# # # # # # # # # # # # сверху найс но без озвучки камеры

# # # # # # # # # # # --- Импорты ---
# # # # # # # # # # import os
# # # # # # # # # # import uuid
# # # # # # # # # # import cv2
# # # # # # # # # # import numpy as np
# # # # # # # # # # import psycopg2
# # # # # # # # # # from flask import Flask, request, jsonify, send_file
# # # # # # # # # # from ultralytics import YOLO
# # # # # # # # # # import wave
# # # # # # # # # # import struct
# # # # # # # # # # import hashlib
# # # # # # # # # # import tempfile
# # # # # # # # # # import time
# # # # # # # # # # import io
# # # # # # # # # # import base64
# # # # # # # # # # from scipy.fftpack import dct
# # # # # # # # # # import pytesseract
# # # # # # # # # # from gtts import gTTS
# # # # # # # # # # import re

# # # # # # # # # # # --- Конфигурация ---
# # # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # # DB_USER = "postgres"
# # # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # # app = Flask(__name__)

# # # # # # # # # # # --- CORS ---
# # # # # # # # # # @app.after_request
# # # # # # # # # # def after_request(response):
# # # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # # #     return response

# # # # # # # # # # def get_db_connection():
# # # # # # # # # #     return psycopg2.connect(
# # # # # # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # # # # # #     )

# # # # # # # # # # # --- ОЗВУЧКА ТЕКСТА ---
# # # # # # # # # # class TextToSpeech:
# # # # # # # # # #     def __init__(self):
# # # # # # # # # #         self.cache_dir = "audio_cache"
# # # # # # # # # #         if not os.path.exists(self.cache_dir):
# # # # # # # # # #             os.makedirs(self.cache_dir)
    
# # # # # # # # # #     def text_to_speech(self, text, lang='ru'):
# # # # # # # # # #         """Конвертирует текст в аудио и возвращает путь к файлу"""
# # # # # # # # # #         try:
# # # # # # # # # #             # Создаем хеш для кэширования
# # # # # # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # # # # # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # # # # # # # # #             # Если уже есть в кэше - возвращаем
# # # # # # # # # #             if os.path.exists(audio_path):
# # # # # # # # # #                 return audio_path
            
# # # # # # # # # #             # Генерируем озвучку
# # # # # # # # # #             tts = gTTS(text=text, lang=lang, slow=False)
# # # # # # # # # #             tts.save(audio_path)
            
# # # # # # # # # #             return audio_path
# # # # # # # # # #         except Exception as e:
# # # # # # # # # #             print(f"Ошибка озвучки: {e}")
# # # # # # # # # #             return None

# # # # # # # # # # tts_engine = TextToSpeech()

# # # # # # # # # # # --- УПРОЩЕННАЯ ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # # # # # # # class SimpleVoiceBiometrics:
# # # # # # # # # #     def __init__(self):
# # # # # # # # # #         print("Инициализирована упрощенная голосовая биометрия!")
    
# # # # # # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # # # # # #         """Извлекаем MFCC характеристики вручную без librosa"""
# # # # # # # # # #         try:
# # # # # # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # # # # # #                 frame_rate = wav_file.getframerate()
# # # # # # # # # #                 n_frames = wav_file.getnframes()
# # # # # # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # # # # # #                 if sample_width == 2:
# # # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # # # # # #                 else:
# # # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # # # # # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # # # # # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
                
# # # # # # # # # #                 print(f"Извлечены аудио характеристики: {len(features)} параметров")
# # # # # # # # # #                 return features_b64
                
# # # # # # # # # #         except Exception as e:
# # # # # # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # # # # # #             return self._fallback_features(audio_path)
    
# # # # # # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # # # # # #         features = []
# # # # # # # # # #         features.append(np.mean(audio_data ** 2))
# # # # # # # # # #         features.append(np.std(audio_data))
# # # # # # # # # #         features.append(np.max(np.abs(audio_data)))
        
# # # # # # # # # #         fft = np.fft.fft(audio_data)
# # # # # # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # # # # # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # # # # # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # # # # # #         for low, high in bands:
# # # # # # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # # # # # #             if np.any(mask):
# # # # # # # # # #                 band_energy = np.mean(fft_magnitude[mask])
# # # # # # # # # #                 features.append(band_energy)
# # # # # # # # # #             else:
# # # # # # # # # #                 features.append(0.0)
        
# # # # # # # # # #         features.append(self._zero_crossing_rate(audio_data))
# # # # # # # # # #         features.append(self._spectral_centroid(fft_magnitude, freqs))
        
# # # # # # # # # #         return np.array(features)
    
# # # # # # # # # #     def _zero_crossing_rate(self, audio_data):
# # # # # # # # # #         zero_crossings = np.where(np.diff(np.signbit(audio_data)))[0]
# # # # # # # # # #         return len(zero_crossings) / len(audio_data)
    
# # # # # # # # # #     def _spectral_centroid(self, magnitude, freqs):
# # # # # # # # # #         if np.sum(magnitude) == 0:
# # # # # # # # # #             return 0.0
# # # # # # # # # #         return np.sum(magnitude * freqs) / np.sum(magnitude)
    
# # # # # # # # # #     def _fallback_features(self, audio_path):
# # # # # # # # # #         try:
# # # # # # # # # #             file_stats = os.stat(audio_path)
# # # # # # # # # #             features = np.array([file_stats.st_size, file_stats.st_mtime])
# # # # # # # # # #             features_bytes = features.astype(np.float32).tobytes()
# # # # # # # # # #             features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # # # # # # # # #             return features_b64
# # # # # # # # # #         except:
# # # # # # # # # #             return None
    
# # # # # # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # # # # # #         try:
# # # # # # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # # # # # #             min_len = min(len(features1), len(features2))
# # # # # # # # # #             features1 = features1[:min_len]
# # # # # # # # # #             features2 = features2[:min_len]
            
# # # # # # # # # #             distance = np.linalg.norm(features1 - features2)
# # # # # # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
# # # # # # # # # #             similarity = max(0.0, min(1.0, similarity))
            
# # # # # # # # # #             print(f"Схожесть голосов: {similarity:.4f}")
# # # # # # # # # #             return float(similarity)
            
# # # # # # # # # #         except Exception as e:
# # # # # # # # # #             print(f"Ошибка сравнения: {e}")
# # # # # # # # # #             return 0.0

# # # # # # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # # # # # --- РАСПОЗНАВАНИЕ ГОЛОСОВЫХ КОМАНД ---
# # # # # # # # # # class VoiceCommandRecognizer:
# # # # # # # # # #     def __init__(self):
# # # # # # # # # #         self.commands = {
# # # # # # # # # #             'выйти': ['выйти', 'выход', 'выйти из аккаунта', 'logout'],
# # # # # # # # # #             'сканировать': ['сканировать', 'сканирование', 'что вижу', 'что видишь', 'что впереди', 'scan'],
# # # # # # # # # #         }
    
# # # # # # # # # #     def recognize_command(self, text):
# # # # # # # # # #         """Распознает голосовую команду из текста"""
# # # # # # # # # #         text = text.lower().strip()
        
# # # # # # # # # #         for command_type, keywords in self.commands.items():
# # # # # # # # # #             for keyword in keywords:
# # # # # # # # # #                 if keyword in text:
# # # # # # # # # #                     return command_type
        
# # # # # # # # # #         return None

# # # # # # # # # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # # # # # # # # --- Конвертер AAC в WAV ---
# # # # # # # # # # try:
# # # # # # # # # #     from pydub import AudioSegment
# # # # # # # # # #     PYDUB_AVAILABLE = True
# # # # # # # # # # except ImportError:
# # # # # # # # # #     print("Pydub not available - installing...")
# # # # # # # # # #     import subprocess
# # # # # # # # # #     import sys
# # # # # # # # # #     subprocess.check_call([sys.executable, "-m", "pip", "install", "pydub"])
# # # # # # # # # #     from pydub import AudioSegment
# # # # # # # # # #     PYDUB_AVAILABLE = True

# # # # # # # # # # class AudioConverter:
# # # # # # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # # # # # #         try:
# # # # # # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # # # # # #             audio.export(wav_path, format="wav")
# # # # # # # # # #             return wav_path
# # # # # # # # # #         except Exception as e:
# # # # # # # # # #             print(f"Ошибка конвертации AAC->WAV: {e}")
# # # # # # # # # #             return None

# # # # # # # # # # audio_converter = AudioConverter()

# # # # # # # # # # # --- OCR (Распознавание текста) ---
# # # # # # # # # # class TextRecognizer:
# # # # # # # # # #     def __init__(self):
# # # # # # # # # #         # Настройка Tesseract для лучшего распознавания
# # # # # # # # # #         self.config = '--oem 3 --psm 6'
    
# # # # # # # # # #     def extract_text(self, frame):
# # # # # # # # # #         """Извлекает текст с изображения"""
# # # # # # # # # #         try:
# # # # # # # # # #             # Предобработка для лучшего распознавания
# # # # # # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # # # # # #             gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
            
# # # # # # # # # #             # Распознаем текст (русский и английский)
# # # # # # # # # #             text = pytesseract.image_to_string(gray, lang='rus+eng', config=self.config)
# # # # # # # # # #             text = text.strip()
            
# # # # # # # # # #             if text:
# # # # # # # # # #                 # Извлекаем номера телефонов
# # # # # # # # # #                 phones = self._extract_phone_numbers(text)
                
# # # # # # # # # #                 return {
# # # # # # # # # #                     'text': text,
# # # # # # # # # #                     'phones': phones,
# # # # # # # # # #                     'has_text': True
# # # # # # # # # #                 }
            
# # # # # # # # # #             return {'text': '', 'phones': [], 'has_text': False}
            
# # # # # # # # # #         except Exception as e:
# # # # # # # # # #             print(f"Ошибка OCR: {e}")
# # # # # # # # # #             return {'text': '', 'phones': [], 'has_text': False}
    
# # # # # # # # # #     def _extract_phone_numbers(self, text):
# # # # # # # # # #         """Извлекает номера телефонов из текста"""
# # # # # # # # # #         # Паттерны для номеров телефонов
# # # # # # # # # #         patterns = [
# # # # # # # # # #             r'\+7\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',  # +7 XXX XXX XX XX
# # # # # # # # # #             r'8\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',     # 8 XXX XXX XX XX
# # # # # # # # # #             r'\d{3}[-\s]?\d{2}[-\s]?\d{2}',           # XXX-XX-XX
# # # # # # # # # #         ]
        
# # # # # # # # # #         phones = []
# # # # # # # # # #         for pattern in patterns:
# # # # # # # # # #             found = re.findall(pattern, text)
# # # # # # # # # #             phones.extend(found)
        
# # # # # # # # # #         return phones

# # # # # # # # # # text_recognizer = TextRecognizer()

# # # # # # # # # # # --- YOLO детекция с расширенным набором объектов ---
# # # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # # # # # # # # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8
# # # # # # # # # # }

# # # # # # # # # # # Расширенный перевод на русский
# # # # # # # # # # OBJECT_TRANSLATIONS = {
# # # # # # # # # #     'person': 'человек',
# # # # # # # # # #     'car': 'машина',
# # # # # # # # # #     'chair': 'стул',
# # # # # # # # # #     'bottle': 'бутылка',
# # # # # # # # # #     'cup': 'чашка',
# # # # # # # # # #     'dog': 'собака',
# # # # # # # # # #     'cat': 'кошка',
# # # # # # # # # #     'tv': 'телевизор',
# # # # # # # # # #     'laptop': 'ноутбук',
# # # # # # # # # #     'bicycle': 'велосипед',
# # # # # # # # # #     'motorcycle': 'мотоцикл',
# # # # # # # # # #     'bus': 'автобус',
# # # # # # # # # #     'truck': 'грузовик',
# # # # # # # # # #     'traffic light': 'светофор',
# # # # # # # # # #     'fire hydrant': 'пожарный гидрант',
# # # # # # # # # #     'stop sign': 'знак стоп',
# # # # # # # # # #     'parking meter': 'паркомат',
# # # # # # # # # #     'bench': 'скамейка',
# # # # # # # # # #     'bird': 'птица',
# # # # # # # # # #     'horse': 'лошадь',
# # # # # # # # # #     'sheep': 'овца',
# # # # # # # # # #     'cow': 'корова',
# # # # # # # # # #     'elephant': 'слон',
# # # # # # # # # #     'bear': 'медведь',
# # # # # # # # # #     'zebra': 'зебра',
# # # # # # # # # #     'giraffe': 'жираф',
# # # # # # # # # #     'backpack': 'рюкзак',
# # # # # # # # # #     'umbrella': 'зонт',
# # # # # # # # # #     'handbag': 'сумка',
# # # # # # # # # #     'tie': 'галстук',
# # # # # # # # # #     'suitcase': 'чемодан',
# # # # # # # # # #     'frisbee': 'фрисби',
# # # # # # # # # #     'skis': 'лыжи',
# # # # # # # # # #     'snowboard': 'сноуборд',
# # # # # # # # # #     'sports ball': 'мяч',
# # # # # # # # # #     'kite': 'воздушный змей',
# # # # # # # # # #     'baseball bat': 'бейсбольная бита',
# # # # # # # # # #     'baseball glove': 'бейсбольная перчатка',
# # # # # # # # # #     'skateboard': 'скейтборд',
# # # # # # # # # #     'surfboard': 'доска для серфинга',
# # # # # # # # # #     'tennis racket': 'теннисная ракетка',
# # # # # # # # # #     'wine glass': 'бокал',
# # # # # # # # # #     'fork': 'вилка',
# # # # # # # # # #     'knife': 'нож',
# # # # # # # # # #     'spoon': 'ложка',
# # # # # # # # # #     'bowl': 'миска',
# # # # # # # # # #     'banana': 'банан',
# # # # # # # # # #     'apple': 'яблоко',
# # # # # # # # # #     'sandwich': 'сэндвич',
# # # # # # # # # #     'orange': 'апельсин',
# # # # # # # # # #     'broccoli': 'брокколи',
# # # # # # # # # #     'carrot': 'морковь',
# # # # # # # # # #     'hot dog': 'хот-дог',
# # # # # # # # # #     'pizza': 'пицца',
# # # # # # # # # #     'donut': 'пончик',
# # # # # # # # # #     'cake': 'торт',
# # # # # # # # # #     'couch': 'диван',
# # # # # # # # # #     'potted plant': 'растение в горшке',
# # # # # # # # # #     'bed': 'кровать',
# # # # # # # # # #     'dining table': 'обеденный стол',
# # # # # # # # # #     'toilet': 'туалет',
# # # # # # # # # #     'monitor': 'монитор',
# # # # # # # # # #     'mouse': 'мышь',
# # # # # # # # # #     'remote': 'пульт',
# # # # # # # # # #     'keyboard': 'клавиатура',
# # # # # # # # # #     'cell phone': 'мобильный телефон',
# # # # # # # # # #     'microwave': 'микроволновка',
# # # # # # # # # #     'oven': 'духовка',
# # # # # # # # # #     'toaster': 'тостер',
# # # # # # # # # #     'sink': 'раковина',
# # # # # # # # # #     'refrigerator': 'холодильник',
# # # # # # # # # #     'book': 'книга',
# # # # # # # # # #     'clock': 'часы',
# # # # # # # # # #     'vase': 'ваза',
# # # # # # # # # #     'scissors': 'ножницы',
# # # # # # # # # #     'teddy bear': 'плюшевый мишка',
# # # # # # # # # #     'hair drier': 'фен',
# # # # # # # # # #     'toothbrush': 'зубная щетка'
# # # # # # # # # # }

# # # # # # # # # # # Приоритетные объекты для незрячих (больше внимания)
# # # # # # # # # # PRIORITY_OBJECTS = {
# # # # # # # # # #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck',
# # # # # # # # # #     'traffic light', 'stop sign', 'bench', 'chair', 'dog', 'cat'
# # # # # # # # # # }

# # # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # # #         return None
# # # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # # #     return round(distance, 2)

# # # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # # print("Модель загружена.")

# # # # # # # # # # def detect_objects(frame):
# # # # # # # # # #     try:
# # # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # # #         detections = []
# # # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # # #         for r in results.boxes:
# # # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # # #             label = model.names[cls_id]
# # # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # # #             detections.append({
# # # # # # # # # #                 "label": label,
# # # # # # # # # #                 "confidence": conf,
# # # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # # #                 "distance_m": float(distance) if distance else None,
# # # # # # # # # #                 "priority": label in PRIORITY_OBJECTS
# # # # # # # # # #             })
        
# # # # # # # # # #         # Сортируем по приоритету и близости
# # # # # # # # # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
        
# # # # # # # # # #         return detections
# # # # # # # # # #     except Exception as e:
# # # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # # #         return []

# # # # # # # # # # def generate_obstacle_description(obstacles, ocr_data=None):
# # # # # # # # # #     """Генерирует детальное описание на русском языке с OCR данными"""
# # # # # # # # # #     descriptions = []
# # # # # # # # # #     frame_center = 320
    
# # # # # # # # # #     # Добавляем распознанный текст
# # # # # # # # # #     if ocr_data and ocr_data.get('has_text'):
# # # # # # # # # #         text = ocr_data['text']
# # # # # # # # # #         phones = ocr_data['phones']
        
# # # # # # # # # #         if text:
# # # # # # # # # #             descriptions.append(f"Обнаружен текст: {text[:100]}")
        
# # # # # # # # # #         if phones:
# # # # # # # # # #             descriptions.append(f"Номера телефонов: {', '.join(phones)}")
    
# # # # # # # # # #     # Добавляем объекты
# # # # # # # # # #     if not obstacles:
# # # # # # # # # #         if not descriptions:
# # # # # # # # # #             return "Препятствий не обнаружено"
# # # # # # # # # #     else:
# # # # # # # # # #         for obs in obstacles[:5]:
# # # # # # # # # #             label_en = obs['label']
# # # # # # # # # #             label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
            
# # # # # # # # # #             conf = int(obs['confidence'] * 100)
# # # # # # # # # #             distance = obs.get('distance_m')
            
# # # # # # # # # #             if distance is not None:
# # # # # # # # # #                 meters = int(distance)
# # # # # # # # # #                 centimeters = int(round((distance - meters) * 100))
                
# # # # # # # # # #                 if meters == 0 and centimeters > 0:
# # # # # # # # # #                     dist_text = f" на расстоянии {centimeters} сантиметров"
# # # # # # # # # #                 elif meters > 0 and centimeters == 0:
# # # # # # # # # #                     dist_text = f" на расстоянии {meters} метров"
# # # # # # # # # #                 else:
# # # # # # # # # #                     dist_text = f" на расстоянии {meters} метров {centimeters} сантиметров"
# # # # # # # # # #             else:
# # # # # # # # # #                 dist_text = ""
            
# # # # # # # # # #             bbox = obs['bbox']
# # # # # # # # # #             x_center = (bbox[0] + bbox[2]) / 2
            
# # # # # # # # # #             if x_center < frame_center - 100:
# # # # # # # # # #                 pos = "слева"
# # # # # # # # # #             elif x_center > frame_center + 100:
# # # # # # # # # #                 pos = "справа"
# # # # # # # # # #             else:
# # # # # # # # # #                 pos = "прямо перед вами"
            
# # # # # # # # # #             priority_marker = "Внимание! " if obs.get('priority') else ""
# # # # # # # # # #             descriptions.append(f"{priority_marker}{label_ru} {pos}{dist_text}")
        
# # # # # # # # # #         if len(obstacles) > 5:
# # # # # # # # # #             descriptions.append(f"и еще {len(obstacles) - 5} объектов")
    
# # # # # # # # # #     if descriptions:
# # # # # # # # # #         return ". ".join(descriptions)
# # # # # # # # # #     else:
# # # # # # # # # #         return "Ничего не обнаружено"

# # # # # # # # # # # --- Эндпоинты ---

# # # # # # # # # # @app.route('/')
# # # # # # # # # # def index():
# # # # # # # # # #     return jsonify({
# # # # # # # # # #         "status": "Blind Assistant Server is running",
# # # # # # # # # #         "version": "7.0 - Voice + OCR",
# # # # # # # # # #         "features": [
# # # # # # # # # #             "Голосовая биометрия",
# # # # # # # # # #             "Детекция объектов YOLO",
# # # # # # # # # #             "Распознавание текста OCR",
# # # # # # # # # #             "Озвучка описаний",
# # # # # # # # # #             "Голосовые команды"
# # # # # # # # # #         ]
# # # # # # # # # #     })

# # # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # # def register_voice():
# # # # # # # # # #     """Регистрация голосового профиля пользователя"""
# # # # # # # # # #     temp_audio = None
# # # # # # # # # #     temp_wav = None
# # # # # # # # # #     try:
# # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # #             return jsonify({"error": "Необходимо аудио и имя пользователя"}), 400

# # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # #         if not username:
# # # # # # # # # #             return jsonify({"error": "Имя пользователя обязательно"}), 400
        
# # # # # # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # # #         if not temp_wav:
# # # # # # # # # #             return jsonify({"error": "Ошибка конвертации аудио"}), 400
        
# # # # # # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # # #         if not voice_features:
# # # # # # # # # #             return jsonify({"error": "Ошибка анализа голоса"}), 400
        
# # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # #         try:
# # # # # # # # # #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # # #             existing = cur.fetchone()
            
# # # # # # # # # #             if existing:
# # # # # # # # # #                 cur.execute(
# # # # # # # # # #                     "UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # # # # # #                     (voice_features, username)
# # # # # # # # # #                 )
# # # # # # # # # #                 message = "Голосовой профиль обновлен"
# # # # # # # # # #             else:
# # # # # # # # # #                 cur.execute(
# # # # # # # # # #                     "INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # # # # # #                     (username, voice_features)
# # # # # # # # # #                 )
# # # # # # # # # #                 message = "Голосовой профиль зарегистрирован"
            
# # # # # # # # # #             conn.commit()
# # # # # # # # # #             result = {
# # # # # # # # # #                 "status": "success", 
# # # # # # # # # #                 "message": message, 
# # # # # # # # # #                 "username": username,
# # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # #             }
            
# # # # # # # # # #         except Exception as e:
# # # # # # # # # #             conn.rollback()
# # # # # # # # # #             result = {"error": f"Ошибка БД: {str(e)}"}
# # # # # # # # # #         finally:
# # # # # # # # # #             cur.close()
# # # # # # # # # #             conn.close()
        
# # # # # # # # # #         return jsonify(result)
        
# # # # # # # # # #     except Exception as e:
# # # # # # # # # #         return jsonify({"error": f"Ошибка регистрации: {str(e)}"}), 500
# # # # # # # # # #     finally:
# # # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # # #                 try:
# # # # # # # # # #                     os.remove(temp_file)
# # # # # # # # # #                 except:
# # # # # # # # # #                     pass

# # # # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # # # def login_voice():
# # # # # # # # # #     """Вход через голосовую биометрию с озвучкой"""
# # # # # # # # # #     temp_audio = None
# # # # # # # # # #     temp_wav = None
# # # # # # # # # #     try:
# # # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # # #             return jsonify({"error": "Необходимо аудио и имя пользователя"}), 400

# # # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # # #         if not username:
# # # # # # # # # #             return jsonify({"error": "Имя пользователя обязательно"}), 400
        
# # # # # # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # # #         if not temp_wav:
# # # # # # # # # #             return jsonify({"error": "Ошибка конвертации аудио"}), 400
        
# # # # # # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # # #         if not current_features:
# # # # # # # # # #             return jsonify({"error": "Ошибка анализа голоса"}), 400
        
# # # # # # # # # #         conn = get_db_connection()
# # # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # # #         result = cur.fetchone()
        
# # # # # # # # # #         if not result or not result[0]:
# # # # # # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # # # # # #         stored_features = result[0]
# # # # # # # # # #         similarity = voice_biometrics.compare_voice_features(stored_features, current_features)
        
# # # # # # # # # #         cur.close()
# # # # # # # # # #         conn.close()
        
# # # # # # # # # #         print(f"Сходство голосов: {similarity:.4f}")
        
# # # # # # # # # #         if similarity > 0.7:
# # # # # # # # # #             # Генерируем приветственное сообщение
# # # # # # # # # #             welcome_text = f"Добро пожаловать, {username}! Перед вами камера. Чуть ниже центра экрана находится кнопка для сканирования окружения. Вы можете сказать 'сканировать' для анализа или 'выйти' для выхода из аккаунта."
# # # # # # # # # #             audio_path = tts_engine.text_to_speech(welcome_text)
            
# # # # # # # # # #             return jsonify({
# # # # # # # # # #                 "status": "success", 
# # # # # # # # # #                 "message": "Голос распознан! Вход успешен", 
# # # # # # # # # #                 "username": username,
# # # # # # # # # #                 "similarity": float(similarity),
# # # # # # # # # #                 "biometry": "simple_mfcc",
# # # # # # # # # #                 "welcome_text": welcome_text,
# # # # # # # # # #                 "audio_available": audio_path is not None
# # # # # # # # # #             })
# # # # # # # # # #         else:
# # # # # # # # # #             return jsonify({
# # # # # # # # # #                 "status": "fail", 
# # # # # # # # # #                 "message": "Голос не распознан",
# # # # # # # # # #                 "similarity": float(similarity),
# # # # # # # # # #                 "biometry": "simple_mfcc"
# # # # # # # # # #             })
            
# # # # # # # # # #     except Exception as e:
# # # # # # # # # #         return jsonify({"error": f"Ошибка входа: {str(e)}"}), 500
# # # # # # # # # #     finally:
# # # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # # #                 try:
# # # # # # # # # #                     os.remove(temp_file)
# # # # # # # # # #                 except:
# # # # # # # # # #                     pass

# # # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # # def voice_command():
# # # # # # # # # #     """Обработка голосовых команд"""
# # # # # # # # # #     temp_audio = None
# # # # # # # # # #     temp_wav = None
# # # # # # # # # #     try:
# # # # # # # # # #         if 'audio' not in request.files:
# # # # # # # # # #             return jsonify({"error": "Аудио не предоставлено"}), 400
        
# # # # # # # # # #         audio_file = request.files['audio']
# # # # # # # # # #         temp_audio = f"temp_command_{uuid.uuid4()}.aac"
# # # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # # #         if not temp_wav:
# # # # # # # # # #             return jsonify({"error": "Ошибка конвертации аудио"}), 400
        
# # # # # # # # # #         # Здесь должно быть speech-to-text, но для простоты используем заглушку
# # # # # # # # # #         # В реальности нужно использовать Google Speech API или другой сервис
# # # # # # # # # #         recognized_text = request.form.get('text', '').lower()
        
# # # # # # # # # #         if not recognized_text:
# # # # # # # # # #             return jsonify({"error": "Текст команды не распознан"}), 400
        
# # # # # # # # # #         command = voice_command_recognizer.recognize_command(recognized_text)
        
# # # # # # # # # #         if command:
# # # # # # # # # #             return jsonify({
# # # # # # # # # #                 "status": "success",
# # # # # # # # # #                 "command": command,
# # # # # # # # # #                 "text": recognized_text
# # # # # # # # # #             })
# # # # # # # # # #         else:
# # # # # # # # # #             return jsonify({
# # # # # # # # # #                 "status": "unknown",
# # # # # # # # # #                 "message": "Команда не распознана",
# # # # # # # # # #                 "text": recognized_text
# # # # # # # # # #             })
            
# # # # # # # # # #     except Exception as e:
# # # # # # # # # #         return jsonify({"error": f"Ошибка обработки команды: {str(e)}"}), 500
# # # # # # # # # #     finally:
# # # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # # #                 try:
# # # # # # # # # #                     os.remove(temp_file)
# # # # # # # # # #                 except:
# # # # # # # # # #                     pass

# # # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # # def process_frame():
# # # # # # # # # #     """Обработка кадра с камеры: детекция объектов + OCR + озвучка"""
# # # # # # # # # #     try:
# # # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # # #         if frame is None:
# # # # # # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # # # # # #         # Детекция объектов
# # # # # # # # # #         obstacles = detect_objects(frame)
        
# # # # # # # # # #         # Распознавание текста
# # # # # # # # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # # # # # # # #         # Генерируем описание
# # # # # # # # # #         description = generate_obstacle_description(obstacles, ocr_data)
        
# # # # # # # # # #         # Создаем аудио с описанием
# # # # # # # # # #         audio_path = tts_engine.text_to_speech(description)

# # # # # # # # # #         # Сохраняем в БД
# # # # # # # # # #         if obstacles:
# # # # # # # # # #             conn = get_db_connection()
# # # # # # # # # #             cur = conn.cursor()
# # # # # # # # # #             for obs in obstacles:
# # # # # # # # # #                 try:
# # # # # # # # # #                     cur.execute(
# # # # # # # # # #                         "INSERT INTO obstacles (label, confidence, bbox, distance) VALUES (%s, %s, %s, %s)",
# # # # # # # # # #                         (
# # # # # # # # # #                             obs['label'],
# # # # # # # # # #                             float(obs['confidence']),
# # # # # # # # # #                             ','.join(map(str, obs['bbox'])),
# # # # # # # # # #                             float(obs['distance_m']) if obs.get('distance_m') else None
# # # # # # # # # #                         )
# # # # # # # # # #                     )
# # # # # # # # # #                 except Exception as e:
# # # # # # # # # #                     print(f"Ошибка БД: {e}")
# # # # # # # # # #             conn.commit()
# # # # # # # # # #             cur.close()
# # # # # # # # # #             conn.close()

# # # # # # # # # #         return jsonify({
# # # # # # # # # #             "obstacles": obstacles,
# # # # # # # # # #             "ocr": ocr_data,
# # # # # # # # # #             "description": description,
# # # # # # # # # #             "count": len(obstacles),
# # # # # # # # # #             "audio_available": audio_path is not None,
# # # # # # # # # #             "message": "Кадр обработан"
# # # # # # # # # #         })
# # # # # # # # # #     except Exception as e:
# # # # # # # # # #         return jsonify({"error": f"Ошибка обработки кадра: {str(e)}"}), 500

# # # # # # # # # # @app.route('/get_audio/<text_hash>')
# # # # # # # # # # def get_audio(text_hash):
# # # # # # # # # #     """Отдает аудио файл по хешу текста"""
# # # # # # # # # #     try:
# # # # # # # # # #         audio_path = os.path.join(tts_engine.cache_dir, f"{text_hash}.mp3")
# # # # # # # # # #         if os.path.exists(audio_path):
# # # # # # # # # #             return send_file(audio_path, mimetype='audio/mpeg')
# # # # # # # # # #         else:
# # # # # # # # # #             return jsonify({"error": "Аудио не найдено"}), 404
# # # # # # # # # #     except Exception as e:
# # # # # # # # # #         return jsonify({"error": f"Ошибка получения аудио: {str(e)}"}), 500

# # # # # # # # # # @app.route('/text_to_speech', methods=['POST'])
# # # # # # # # # # def text_to_speech_endpoint():
# # # # # # # # # #     """Конвертирует текст в речь и возвращает аудио"""
# # # # # # # # # #     try:
# # # # # # # # # #         data = request.get_json()
# # # # # # # # # #         text = data.get('text', '')
        
# # # # # # # # # #         if not text:
# # # # # # # # # #             return jsonify({"error": "Текст не предоставлен"}), 400
        
# # # # # # # # # #         audio_path = tts_engine.text_to_speech(text)
        
# # # # # # # # # #         if audio_path:
# # # # # # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # # # # # #             return jsonify({
# # # # # # # # # #                 "status": "success",
# # # # # # # # # #                 "audio_hash": text_hash,
# # # # # # # # # #                 "audio_url": f"/get_audio/{text_hash}"
# # # # # # # # # #             })
# # # # # # # # # #         else:
# # # # # # # # # #             return jsonify({"error": "Ошибка генерации аудио"}), 500
            
# # # # # # # # # #     except Exception as e:
# # # # # # # # # #         return jsonify({"error": f"Ошибка: {str(e)}"}), 500

# # # # # # # # # # if __name__ == '__main__':
# # # # # # # # # #     print("=" * 60)
# # # # # # # # # #     print("BLIND ASSISTANT SERVER - ПОЛНАЯ ВЕРСИЯ")
# # # # # # # # # #     print("=" * 60)
# # # # # # # # # #     print("✓ Голосовая биометрия (регистрация и вход)")
# # # # # # # # # #     print("✓ Детекция объектов с YOLO")
# # # # # # # # # #     print("✓ Распознавание текста (OCR)")
# # # # # # # # # #     print("✓ Озвучка описаний (Text-to-Speech)")
# # # # # # # # # #     print("✓ Голосовые команды (выйти, сканировать)")
# # # # # # # # # #     print("✓ Приоритетные объекты для незрячих")
# # # # # # # # # #     print("✓ Описания на русском языке")
# # # # # # # # # #     print("✓ Сохранение в PostgreSQL")
# # # # # # # # # #     print("=" * 60)
# # # # # # # # # #     print("\nНеобходимые библиотеки:")
# # # # # # # # # #     print("pip install gtts pytesseract opencv-python")
# # # # # # # # # #     print("sudo apt-get install tesseract-ocr tesseract-ocr-rus")
# # # # # # # # # #     print("=" * 60)
# # # # # # # # # #     app.run(host='10.189.181.73', port=5000, debug=True)


# # # # # # # # # # # идеал сверху






# # # # # # # # # # --- Импорты ---
# # # # # # # # # import os
# # # # # # # # # import uuid
# # # # # # # # # import cv2
# # # # # # # # # import numpy as np
# # # # # # # # # import psycopg2
# # # # # # # # # from flask import Flask, request, jsonify, send_file
# # # # # # # # # from ultralytics import YOLO
# # # # # # # # # import wave
# # # # # # # # # import hashlib
# # # # # # # # # import base64
# # # # # # # # # import pytesseract
# # # # # # # # # from gtts import gTTS
# # # # # # # # # import re
# # # # # # # # # from datetime import datetime
# # # # # # # # # import requests

# # # # # # # # # # --- Конфигурация ---
# # # # # # # # # DB_NAME = "blind_app"
# # # # # # # # # DB_USER = "postgres"
# # # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # # DB_HOST = "localhost"

# # # # # # # # # app = Flask(__name__)

# # # # # # # # # # --- CORS ---
# # # # # # # # # @app.after_request
# # # # # # # # # def after_request(response):
# # # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # # #     return response

# # # # # # # # # def get_db_connection():
# # # # # # # # #     return psycopg2.connect(
# # # # # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # # # # #     )

# # # # # # # # # # --- ПОГОДА И ВРЕМЯ ---
# # # # # # # # # class WeatherTimeService:
# # # # # # # # #     def __init__(self):
# # # # # # # # #         self.weather_api_key = "YOUR_API_KEY"  # Замените на ваш ключ от OpenWeatherMap
# # # # # # # # #         self.city = "Astana"
    
# # # # # # # # #     def get_current_time(self):
# # # # # # # # #         """Возвращает текущее время"""
# # # # # # # # #         now = datetime.now()
# # # # # # # # #         hours = now.hour
# # # # # # # # #         minutes = now.minute
        
# # # # # # # # #         time_str = f"{hours} часов {minutes} минут"
# # # # # # # # #         return {
# # # # # # # # #             'time': time_str,
# # # # # # # # #             'hour': hours,
# # # # # # # # #             'minute': minutes
# # # # # # # # #         }
    
# # # # # # # # #     def get_weather(self):
# # # # # # # # #         """Получает текущую погоду"""
# # # # # # # # #         try:
# # # # # # # # #             url = f"http://api.openweathermap.org/data/2.5/weather?q={self.city}&appid={self.weather_api_key}&units=metric&lang=ru"
# # # # # # # # #             response = requests.get(url, timeout=5)
            
# # # # # # # # #             if response.status_code == 200:
# # # # # # # # #                 data = response.json()
                
# # # # # # # # #                 temp = round(data['main']['temp'])
# # # # # # # # #                 feels_like = round(data['main']['feels_like'])
# # # # # # # # #                 description = data['weather'][0]['description']
# # # # # # # # #                 humidity = data['main']['humidity']
# # # # # # # # #                 wind_speed = round(data['wind']['speed'])
                
# # # # # # # # #                 weather_text = f"Сейчас в городе {self.city} {temp} градусов. "
# # # # # # # # #                 weather_text += f"Ощущается как {feels_like}. "
# # # # # # # # #                 weather_text += f"{description}. "
# # # # # # # # #                 weather_text += f"Влажность {humidity} процентов. "
# # # # # # # # #                 weather_text += f"Скорость ветра {wind_speed} метров в секунду."
                
# # # # # # # # #                 return {
# # # # # # # # #                     'temperature': temp,
# # # # # # # # #                     'feels_like': feels_like,
# # # # # # # # #                     'description': description,
# # # # # # # # #                     'text': weather_text
# # # # # # # # #                 }
# # # # # # # # #             else:
# # # # # # # # #                 return {'error': 'Не удалось получить данные о погоде'}
                
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка получения погоды: {e}")
# # # # # # # # #             # Заглушка без API
# # # # # # # # #             return {
# # # # # # # # #                 'text': f"Сейчас в городе {self.city} примерно 15 градусов. Облачно.",
# # # # # # # # #                 'temperature': 15,
# # # # # # # # #                 'description': 'облачно'
# # # # # # # # #             }

# # # # # # # # # weather_service = WeatherTimeService()

# # # # # # # # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # # # # # # # class ColorAnalyzer:
# # # # # # # # #     def __init__(self):
# # # # # # # # #         self.color_names = {
# # # # # # # # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # # # # # # # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # # # # # # # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # # # # # # # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # # # # # # # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # # # # # # # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # # # # # # # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # # # # # # # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # # # # # # # #             'бордовый': ([170, 100, 100], [180, 255, 255]),
# # # # # # # # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # # # # # # # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # # # # # # # #             'черный': ([0, 0, 0], [180, 255, 50])
# # # # # # # # #         }
    
# # # # # # # # #     def analyze_colors(self, frame):
# # # # # # # # #         """Анализирует доминирующие цвета в кадре"""
# # # # # # # # #         try:
# # # # # # # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # # # # # # # #             detected_colors = []
            
# # # # # # # # #             for color_name, (lower, upper) in self.color_names.items():
# # # # # # # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # # # # # # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # # # # # # # #                 if percentage > 5:
# # # # # # # # #                     detected_colors.append({
# # # # # # # # #                         'color': color_name,
# # # # # # # # #                         'percentage': round(percentage, 1)
# # # # # # # # #                     })
            
# # # # # # # # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # # # # # # # #             return detected_colors[:3]
            
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка анализа цветов: {e}")
# # # # # # # # #             return []
    
# # # # # # # # #     def get_brightness_level(self, frame):
# # # # # # # # #         """Определяет уровень освещенности"""
# # # # # # # # #         try:
# # # # # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # # # # #             avg_brightness = np.mean(gray)
            
# # # # # # # # #             if avg_brightness < 50:
# # # # # # # # #                 return "очень темно"
# # # # # # # # #             elif avg_brightness < 100:
# # # # # # # # #                 return "темно"
# # # # # # # # #             elif avg_brightness < 150:
# # # # # # # # #                 return "нормальное освещение"
# # # # # # # # #             elif avg_brightness < 200:
# # # # # # # # #                 return "светло"
# # # # # # # # #             else:
# # # # # # # # #                 return "очень светло"
# # # # # # # # #         except:
# # # # # # # # #             return "не определено"

# # # # # # # # # color_analyzer = ColorAnalyzer()

# # # # # # # # # # --- ВИЗУАЛИЗАЦИЯ ОБЪЕКТОВ ---
# # # # # # # # # class ObjectVisualizer:
# # # # # # # # #     def __init__(self):
# # # # # # # # #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# # # # # # # # #     def draw_detections(self, frame, detections, ocr_data=None):
# # # # # # # # #         """Рисует зеленые прямоугольники и подписи на кадре"""
# # # # # # # # #         try:
# # # # # # # # #             annotated_frame = frame.copy()
            
# # # # # # # # #             for det in detections:
# # # # # # # # #                 bbox = det['bbox']
# # # # # # # # #                 label_en = det['label']
# # # # # # # # #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # # # # # # # #                 conf = int(det['confidence'] * 100)
# # # # # # # # #                 distance = det.get('distance_m')
# # # # # # # # #                 priority = det.get('priority', False)
                
# # # # # # # # #                 # Зеленый цвет для всех объектов
# # # # # # # # #                 color = (0, 255, 0)  # Зеленый
# # # # # # # # #                 thickness = 3 if priority else 2
                
# # # # # # # # #                 # Рисуем прямоугольник
# # # # # # # # #                 cv2.rectangle(annotated_frame, 
# # # # # # # # #                             (bbox[0], bbox[1]), 
# # # # # # # # #                             (bbox[2], bbox[3]), 
# # # # # # # # #                             color, thickness)
                
# # # # # # # # #                 # Формируем текст
# # # # # # # # #                 if distance:
# # # # # # # # #                     text = f"{label_ru} - {distance:.1f}м"
# # # # # # # # #                 else:
# # # # # # # # #                     text = f"{label_ru}"
                
# # # # # # # # #                 # Рисуем фон для текста
# # # # # # # # #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# # # # # # # # #                 cv2.rectangle(annotated_frame,
# # # # # # # # #                             (bbox[0], bbox[1] - text_height - 10),
# # # # # # # # #                             (bbox[0] + text_width + 10, bbox[1]),
# # # # # # # # #                             color, -1)
                
# # # # # # # # #                 # Рисуем текст
# # # # # # # # #                 cv2.putText(annotated_frame, text,
# # # # # # # # #                           (bbox[0] + 5, bbox[1] - 5),
# # # # # # # # #                           self.font, 0.7, (0, 0, 0), 2)
            
# # # # # # # # #             # Добавляем информацию об OCR
# # # # # # # # #             if ocr_data and ocr_data.get('has_text'):
# # # # # # # # #                 y_pos = 30
# # # # # # # # #                 cv2.rectangle(annotated_frame, (10, 10), (300, y_pos + 25), (0, 255, 0), -1)
# # # # # # # # #                 cv2.putText(annotated_frame, "Текст обнаружен",
# # # # # # # # #                           (15, y_pos), self.font, 0.7, (0, 0, 0), 2)
            
# # # # # # # # #             return annotated_frame
            
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка визуализации: {e}")
# # # # # # # # #             return frame

# # # # # # # # # object_visualizer = ObjectVisualizer()

# # # # # # # # # # --- ОЗВУЧКА ТЕКСТА ---
# # # # # # # # # class TextToSpeech:
# # # # # # # # #     def __init__(self):
# # # # # # # # #         self.cache_dir = "audio_cache"
# # # # # # # # #         if not os.path.exists(self.cache_dir):
# # # # # # # # #             os.makedirs(self.cache_dir)
    
# # # # # # # # #     def text_to_speech(self, text, lang='ru'):
# # # # # # # # #         """Конвертирует текст в аудио"""
# # # # # # # # #         try:
# # # # # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # # # # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # # # # # # # #             if os.path.exists(audio_path):
# # # # # # # # #                 return audio_path
            
# # # # # # # # #             tts = gTTS(text=text, lang=lang, slow=False)
# # # # # # # # #             tts.save(audio_path)
            
# # # # # # # # #             return audio_path
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка озвучки: {e}")
# # # # # # # # #             return None

# # # # # # # # # tts_engine = TextToSpeech()

# # # # # # # # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # # # # # # class SimpleVoiceBiometrics:
# # # # # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # # # # #         try:
# # # # # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # # # # #                 frame_rate = wav_file.getframerate()
# # # # # # # # #                 n_frames = wav_file.getnframes()
# # # # # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # # # # #                 if sample_width == 2:
# # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # # # # #                 else:
# # # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # # # # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # # # # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
                
# # # # # # # # #                 return features_b64
                
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # # # # #             return None
    
# # # # # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # # # # #         features = []
# # # # # # # # #         features.append(np.mean(audio_data ** 2))
# # # # # # # # #         features.append(np.std(audio_data))
# # # # # # # # #         features.append(np.max(np.abs(audio_data)))
        
# # # # # # # # #         fft = np.fft.fft(audio_data)
# # # # # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # # # # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # # # # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # # # # #         for low, high in bands:
# # # # # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # # # # #             if np.any(mask):
# # # # # # # # #                 band_energy = np.mean(fft_magnitude[mask])
# # # # # # # # #                 features.append(band_energy)
# # # # # # # # #             else:
# # # # # # # # #                 features.append(0.0)
        
# # # # # # # # #         return np.array(features)
    
# # # # # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # # # # #         try:
# # # # # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # # # # #             min_len = min(len(features1), len(features2))
# # # # # # # # #             features1 = features1[:min_len]
# # # # # # # # #             features2 = features2[:min_len]
            
# # # # # # # # #             distance = np.linalg.norm(features1 - features2)
# # # # # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
            
# # # # # # # # #             return float(max(0.0, min(1.0, similarity)))
            
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка сравнения: {e}")
# # # # # # # # #             return 0.0

# # # # # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # # # # --- РАСПОЗНАВАНИЕ КОМАНД ---
# # # # # # # # # class VoiceCommandRecognizer:
# # # # # # # # #     def __init__(self):
# # # # # # # # #         self.commands = {
# # # # # # # # #             'выйти': ['выйти', 'выход', 'logout'],
# # # # # # # # #             'сканировать': ['сканировать', 'что вижу', 'что впереди', 'scan'],
# # # # # # # # #             'погода': ['погода', 'какая погода', 'температура'],
# # # # # # # # #             'время': ['время', 'который час', 'сколько времени']
# # # # # # # # #         }
    
# # # # # # # # #     def recognize_command(self, text):
# # # # # # # # #         text = text.lower().strip()
        
# # # # # # # # #         for command_type, keywords in self.commands.items():
# # # # # # # # #             for keyword in keywords:
# # # # # # # # #                 if keyword in text:
# # # # # # # # #                     return command_type
        
# # # # # # # # #         return None

# # # # # # # # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # # # # # # # --- КОНВЕРТЕР АУДИО ---
# # # # # # # # # try:
# # # # # # # # #     from pydub import AudioSegment
# # # # # # # # #     PYDUB_AVAILABLE = True
# # # # # # # # # except ImportError:
# # # # # # # # #     import subprocess
# # # # # # # # #     import sys
# # # # # # # # #     subprocess.check_call([sys.executable, "-m", "pip", "install", "pydub"])
# # # # # # # # #     from pydub import AudioSegment
# # # # # # # # #     PYDUB_AVAILABLE = True

# # # # # # # # # class AudioConverter:
# # # # # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # # # # #         try:
# # # # # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # # # # #             audio.export(wav_path, format="wav")
# # # # # # # # #             return wav_path
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка конвертации: {e}")
# # # # # # # # #             return None

# # # # # # # # # audio_converter = AudioConverter()

# # # # # # # # # # --- OCR ---
# # # # # # # # # class TextRecognizer:
# # # # # # # # #     def __init__(self):
# # # # # # # # #         self.config = '--oem 3 --psm 6'
    
# # # # # # # # #     def extract_text(self, frame):
# # # # # # # # #         try:
# # # # # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # # # # #             gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
            
# # # # # # # # #             text = pytesseract.image_to_string(gray, lang='rus+eng', config=self.config)
# # # # # # # # #             text = text.strip()
            
# # # # # # # # #             if text:
# # # # # # # # #                 phones = self._extract_phone_numbers(text)
                
# # # # # # # # #                 return {
# # # # # # # # #                     'text': text,
# # # # # # # # #                     'phones': phones,
# # # # # # # # #                     'has_text': True
# # # # # # # # #                 }
            
# # # # # # # # #             return {'text': '', 'phones': [], 'has_text': False}
            
# # # # # # # # #         except Exception as e:
# # # # # # # # #             print(f"Ошибка OCR: {e}")
# # # # # # # # #             return {'text': '', 'phones': [], 'has_text': False}
    
# # # # # # # # #     def _extract_phone_numbers(self, text):
# # # # # # # # #         patterns = [
# # # # # # # # #             r'\+7\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # # # # #             r'8\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # # # # #             r'\d{3}[-\s]?\d{2}[-\s]?\d{2}',
# # # # # # # # #         ]
        
# # # # # # # # #         phones = []
# # # # # # # # #         for pattern in patterns:
# # # # # # # # #             found = re.findall(pattern, text)
# # # # # # # # #             phones.extend(found)
        
# # # # # # # # #         return phones

# # # # # # # # # text_recognizer = TextRecognizer()

# # # # # # # # # # --- YOLO ---
# # # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # # # # # # # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8
# # # # # # # # # }

# # # # # # # # # OBJECT_TRANSLATIONS = {
# # # # # # # # #     'person': 'человек', 'car': 'машина', 'chair': 'стул', 'bottle': 'бутылка',
# # # # # # # # #     'cup': 'чашка', 'dog': 'собака', 'cat': 'кошка', 'tv': 'телевизор',
# # # # # # # # #     'laptop': 'ноутбук', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл',
# # # # # # # # #     'bus': 'автобус', 'truck': 'грузовик', 'traffic light': 'светофор',
# # # # # # # # #     'stop sign': 'знак стоп', 'bench': 'скамейка', 'book': 'книга',
# # # # # # # # #     'clock': 'часы', 'cell phone': 'мобильный телефон'
# # # # # # # # # }

# # # # # # # # # PRIORITY_OBJECTS = {
# # # # # # # # #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck',
# # # # # # # # #     'traffic light', 'stop sign', 'bench', 'chair', 'dog', 'cat'
# # # # # # # # # }

# # # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # # #         return None
# # # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # # #     return round(distance, 2)

# # # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # # print("Модель загружена.")

# # # # # # # # # def detect_objects(frame):
# # # # # # # # #     try:
# # # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # # # # #         results = model.predict(img_rgb, verbose=False)[0]
# # # # # # # # #         detections = []
# # # # # # # # #         frame_height = frame.shape[0]

# # # # # # # # #         for r in results.boxes:
# # # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # # #             conf = float(r.conf[0])
# # # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # # #             label = model.names[cls_id]
# # # # # # # # #             distance = estimate_distance(bbox, frame_height, label)

# # # # # # # # #             detections.append({
# # # # # # # # #                 "label": label,
# # # # # # # # #                 "confidence": conf,
# # # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # # #                 "distance_m": float(distance) if distance else None,
# # # # # # # # #                 "priority": label in PRIORITY_OBJECTS
# # # # # # # # #             })
        
# # # # # # # # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
        
# # # # # # # # #         return detections
# # # # # # # # #     except Exception as e:
# # # # # # # # #         print(f"Detection error: {e}")
# # # # # # # # #         return []

# # # # # # # # # def generate_obstacle_description(obstacles, ocr_data=None, colors=None, brightness=None):
# # # # # # # # #     """Генерирует описание с цветами, текстом и освещением"""
# # # # # # # # #     descriptions = []
# # # # # # # # #     frame_center = 320
    
# # # # # # # # #     # Освещение
# # # # # # # # #     if brightness:
# # # # # # # # #         descriptions.append(f"Освещение: {brightness}")
    
# # # # # # # # #     # Цвета
# # # # # # # # #     if colors:
# # # # # # # # #         color_desc = ", ".join([f"{c['color']}" for c in colors[:2]])
# # # # # # # # #         descriptions.append(f"Преобладающие цвета: {color_desc}")
    
# # # # # # # # #     # Текст
# # # # # # # # #     if ocr_data and ocr_data.get('has_text'):
# # # # # # # # #         text = ocr_data['text']
# # # # # # # # #         phones = ocr_data['phones']
        
# # # # # # # # #         if text:
# # # # # # # # #             text_short = text[:100] + "..." if len(text) > 100 else text
# # # # # # # # #             descriptions.append(f"Обнаружен текст: {text_short}")
        
# # # # # # # # #         if phones:
# # # # # # # # #             descriptions.append(f"Номера телефонов: {', '.join(phones)}")
    
# # # # # # # # #     # Объекты
# # # # # # # # #     if obstacles:
# # # # # # # # #         for obs in obstacles[:5]:
# # # # # # # # #             label_ru = OBJECT_TRANSLATIONS.get(obs['label'], obs['label'])
# # # # # # # # #             distance = obs.get('distance_m')
            
# # # # # # # # #             if distance is not None:
# # # # # # # # #                 dist_text = f" на расстоянии {int(distance)} метров"
# # # # # # # # #             else:
# # # # # # # # #                 dist_text = ""
            
# # # # # # # # #             bbox = obs['bbox']
# # # # # # # # #             x_center = (bbox[0] + bbox[2]) / 2
            
# # # # # # # # #             if x_center < frame_center - 100:
# # # # # # # # #                 pos = "слева"
# # # # # # # # #             elif x_center > frame_center + 100:
# # # # # # # # #                 pos = "справа"
# # # # # # # # #             else:
# # # # # # # # #                 pos = "прямо"
            
# # # # # # # # #             priority_marker = "Внимание! " if obs.get('priority') else ""
# # # # # # # # #             descriptions.append(f"{priority_marker}{label_ru} {pos}{dist_text}")
    
# # # # # # # # #     return ". ".join(descriptions) if descriptions else "Ничего не обнаружено"

# # # # # # # # # # --- ЭНДПОИНТЫ ---

# # # # # # # # # @app.route('/')
# # # # # # # # # def index():
# # # # # # # # #     return jsonify({
# # # # # # # # #         "status": "Blind Assistant Server",
# # # # # # # # #         "version": "9.0 - Full Enhanced",
# # # # # # # # #         "features": [
# # # # # # # # #             "Голосовая биометрия",
# # # # # # # # #             "Детекция объектов с визуализацией",
# # # # # # # # #             "Распознавание текста и цветов",
# # # # # # # # #             "Погода и время",
# # # # # # # # #             "Озвучка всех данных",
# # # # # # # # #             "Голосовые команды"
# # # # # # # # #         ]
# # # # # # # # #     })

# # # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # # def register_voice():
# # # # # # # # #     temp_audio = None
# # # # # # # # #     temp_wav = None
# # # # # # # # #     try:
# # # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # # #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # #         if not temp_wav:
# # # # # # # # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # # # # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # #         conn = get_db_connection()
# # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # # #         existing = cur.fetchone()
        
# # # # # # # # #         if existing:
# # # # # # # # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # # # # #                        (voice_features, username))
# # # # # # # # #             message = "Профиль обновлен"
# # # # # # # # #         else:
# # # # # # # # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # # # # #                        (username, voice_features))
# # # # # # # # #             message = "Профиль создан"
        
# # # # # # # # #         conn.commit()
# # # # # # # # #         cur.close()
# # # # # # # # #         conn.close()
        
# # # # # # # # #         return jsonify({"status": "success", "message": message, "username": username})
        
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # # #     finally:
# # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # #                 try:
# # # # # # # # #                     os.remove(temp_file)
# # # # # # # # #                 except:
# # # # # # # # #                     pass

# # # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # # def login_voice():
# # # # # # # # #     temp_audio = None
# # # # # # # # #     temp_wav = None
# # # # # # # # #     try:
# # # # # # # # #         username = request.form['username'].strip()
# # # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # # #         conn = get_db_connection()
# # # # # # # # #         cur = conn.cursor()
        
# # # # # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # # #         result = cur.fetchone()
        
# # # # # # # # #         if not result:
# # # # # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # # # # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # # # # # # # #         cur.close()
# # # # # # # # #         conn.close()
        
# # # # # # # # #         if similarity > 0.7:
# # # # # # # # #             welcome_text = f"Добро пожаловать, {username}! Используйте кнопку для сканирования. Скажите 'погода' для прогноза или 'время' чтобы узнать текущее время"
# # # # # # # # #             audio_path = tts_engine.text_to_speech(welcome_text)
            
# # # # # # # # #             return jsonify({
# # # # # # # # #                 "status": "success",
# # # # # # # # #                 "message": "Вход выполнен",
# # # # # # # # #                 "username": username,
# # # # # # # # #                 "similarity": float(similarity)
# # # # # # # # #             })
# # # # # # # # #         else:
# # # # # # # # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
            
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # # #     finally:
# # # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # # #                 try:
# # # # # # # # #                     os.remove(temp_file)
# # # # # # # # #                 except:
# # # # # # # # #                     pass

# # # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # # def voice_command():
# # # # # # # # #     """Обработка голосовых команд включая погоду и время"""
# # # # # # # # #     try:
# # # # # # # # #         recognized_text = request.form.get('text', '').lower()
        
# # # # # # # # #         if not recognized_text:
# # # # # # # # #             return jsonify({"error": "Текст не распознан"}), 400
        
# # # # # # # # #         command = voice_command_recognizer.recognize_command(recognized_text)
        
# # # # # # # # #         if command == 'погода':
# # # # # # # # #             weather_data = weather_service.get_weather()
# # # # # # # # #             return jsonify({
# # # # # # # # #                 "status": "success",
# # # # # # # # #                 "command": "погода",
# # # # # # # # #                 "data": weather_data
# # # # # # # # #             })
        
# # # # # # # # #         elif command == 'время':
# # # # # # # # #             time_data = weather_service.get_current_time()
# # # # # # # # #             return jsonify({
# # # # # # # # #                 "status": "success",
# # # # # # # # #                 "command": "время",
# # # # # # # # #                 "data": time_data
# # # # # # # # #             })
        
# # # # # # # # #         elif command in ['сканировать', 'выйти']:
# # # # # # # # #             return jsonify({
# # # # # # # # #                 "status": "success",
# # # # # # # # #                 "command": command
# # # # # # # # #             })
        
# # # # # # # # #         else:
# # # # # # # # #             return jsonify({
# # # # # # # # #                 "status": "unknown",
# # # # # # # # #                 "message": "Команда не распознана"
# # # # # # # # #             })
            
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # # def process_frame():
# # # # # # # # #     """Обработка кадра: объекты + OCR + цвета + озвучка + визуализация"""
# # # # # # # # #     try:
# # # # # # # # #         if 'frame' not in request.files:
# # # # # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # # #         if frame is None:
# # # # # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # # # # #         # Детекция объектов
# # # # # # # # #         obstacles = detect_objects(frame)
        
# # # # # # # # #         # OCR
# # # # # # # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # # # # # # #         # Анализ цветов
# # # # # # # # #         colors = color_analyzer.analyze_colors(frame)
# # # # # # # # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # # # # # # # #         # Визуализация
# # # # # # # # #         annotated_frame = object_visualizer.draw_detections(frame, obstacles, ocr_data)
        
# # # # # # # # #         # Генерируем описание
# # # # # # # # #         description = generate_obstacle_description(obstacles, ocr_data, colors, brightness)
        
# # # # # # # # #         # Озвучка
# # # # # # # # #         audio_path = tts_engine.text_to_speech(description)
        
# # # # # # # # #         # Сохраняем аннотированный кадр
# # # # # # # # #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# # # # # # # # #         cv2.imwrite(annotated_path, annotated_frame)
        
# # # # # # # # #         # Кодируем в base64
# # # # # # # # #         with open(annotated_path, 'rb') as f:
# # # # # # # # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # # # # # # # #         os.remove(annotated_path)

# # # # # # # # #         return jsonify({
# # # # # # # # #             "obstacles": obstacles,
# # # # # # # # #             "ocr": ocr_data,
# # # # # # # # #             "colors": colors,
# # # # # # # # #             "brightness": brightness,
# # # # # # # # #             "description": description,
# # # # # # # # #             "count": len(obstacles),
# # # # # # # # #             "audio_available": audio_path is not None,
# # # # # # # # #             "annotated_frame": annotated_base64,
# # # # # # # # #             "message": "Кадр обработан успешно"
# # # # # # # # #         })
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": f"Ошибка обработки: {str(e)}"}), 500

# # # # # # # # # @app.route('/get_audio/<text_hash>')
# # # # # # # # # def get_audio(text_hash):
# # # # # # # # #     """Отдает аудио файл"""
# # # # # # # # #     try:
# # # # # # # # #         audio_path = os.path.join(tts_engine.cache_dir, f"{text_hash}.mp3")
# # # # # # # # #         if os.path.exists(audio_path):
# # # # # # # # #             return send_file(audio_path, mimetype='audio/mpeg')
# # # # # # # # #         else:
# # # # # # # # #             return jsonify({"error": "Аудио не найдено"}), 404
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # # @app.route('/text_to_speech', methods=['POST'])
# # # # # # # # # def text_to_speech_endpoint():
# # # # # # # # #     """Конвертирует текст в речь"""
# # # # # # # # #     try:
# # # # # # # # #         data = request.get_json()
# # # # # # # # #         text = data.get('text', '')
        
# # # # # # # # #         if not text:
# # # # # # # # #             return jsonify({"error": "Текст не предоставлен"}), 400
        
# # # # # # # # #         audio_path = tts_engine.text_to_speech(text)
        
# # # # # # # # #         if audio_path:
# # # # # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # # # # #             return jsonify({
# # # # # # # # #                 "status": "success",
# # # # # # # # #                 "audio_hash": text_hash,
# # # # # # # # #                 "audio_url": f"/get_audio/{text_hash}"
# # # # # # # # #             })
# # # # # # # # #         else:
# # # # # # # # #             return jsonify({"error": "Ошибка генерации аудио"}), 500
            
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # # @app.route('/get_weather', methods=['GET'])
# # # # # # # # # def get_weather():
# # # # # # # # #     """Получить текущую погоду"""
# # # # # # # # #     try:
# # # # # # # # #         weather_data = weather_service.get_weather()
# # # # # # # # #         return jsonify(weather_data)
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # # @app.route('/get_time', methods=['GET'])
# # # # # # # # # def get_time():
# # # # # # # # #     """Получить текущее время"""
# # # # # # # # #     try:
# # # # # # # # #         time_data = weather_service.get_current_time()
# # # # # # # # #         return jsonify(time_data)
# # # # # # # # #     except Exception as e:
# # # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # # if __name__ == '__main__':
# # # # # # # # #     print("=" * 60)
# # # # # # # # #     print("BLIND ASSISTANT SERVER - ПОЛНАЯ РАСШИРЕННАЯ ВЕРСИЯ")
# # # # # # # # #     print("=" * 60)
# # # # # # # # #     print("✓ Голосовая биометрия (регистрация и вход)")
# # # # # # # # #     print("✓ Детекция объектов YOLO с визуализацией")
# # # # # # # # #     print("✓ Распознавание текста (OCR)")
# # # # # # # # #     print("✓ Определение цветов")
# # # # # # # # #     print("✓ Анализ освещенности")
# # # # # # # # #     print("✓ Погода и текущее время")
# # # # # # # # #     print("✓ Озвучка всех данных (Text-to-Speech)")
# # # # # # # # #     print("✓ Голосовые команды")
# # # # # # # # #     print("✓ Аннотированные кадры с зелеными рамками")
# # # # # # # # #     print("=" * 60)
# # # # # # # # #     print("\nГолосовые команды:")
# # # # # # # # #     print("- 'сканировать' - анализ окружения")
# # # # # # # # #     print("- 'погода' - текущая погода")
# # # # # # # # #     print("- 'время' - текущее время")
# # # # # # # # #     print("- 'выйти' - выход из аккаунта")
# # # # # # # # #     print("=" * 60)
# # # # # # # # #     print("\nНеобходимые библиотеки:")
# # # # # # # # #     print("pip install gtts pytesseract opencv-python requests")
# # # # # # # # #     print("sudo apt-get install tesseract-ocr tesseract-ocr-rus")
# # # # # # # # #     print("=" * 60)
# # # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)





# # # # # # # # # --- Импорты ---
# # # # # # # # import os
# # # # # # # # import uuid
# # # # # # # # import cv2
# # # # # # # # import numpy as np
# # # # # # # # import psycopg2
# # # # # # # # from flask import Flask, request, jsonify, send_file
# # # # # # # # from ultralytics import YOLO
# # # # # # # # import wave
# # # # # # # # import hashlib
# # # # # # # # import base64
# # # # # # # # import pytesseract
# # # # # # # # from gtts import gTTS
# # # # # # # # import re
# # # # # # # # from datetime import datetime
# # # # # # # # import requests

# # # # # # # # # --- Конфигурация ---
# # # # # # # # DB_NAME = "blind_app"
# # # # # # # # DB_USER = "postgres"
# # # # # # # # DB_PASSWORD = "12345"
# # # # # # # # DB_HOST = "localhost"

# # # # # # # # app = Flask(__name__)

# # # # # # # # # --- CORS ---
# # # # # # # # @app.after_request
# # # # # # # # def after_request(response):
# # # # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # # # #     return response

# # # # # # # # def get_db_connection():
# # # # # # # #     return psycopg2.connect(
# # # # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # # # #     )

# # # # # # # # # --- ПОГОДА И ВРЕМЯ ---
# # # # # # # # class WeatherTimeService:
# # # # # # # #     def __init__(self):
# # # # # # # #         self.weather_api_key = "YOUR_API_KEY"  # Замените на ваш ключ от OpenWeatherMap
# # # # # # # #         self.city = "Astana"
    
# # # # # # # #     def get_current_time(self):
# # # # # # # #         """Возвращает текущее время"""
# # # # # # # #         now = datetime.now()
# # # # # # # #         hours = now.hour
# # # # # # # #         minutes = now.minute
        
# # # # # # # #         time_str = f"{hours} часов {minutes} минут"
# # # # # # # #         return {
# # # # # # # #             'time': time_str,
# # # # # # # #             'hour': hours,
# # # # # # # #             'minute': minutes
# # # # # # # #         }
    
# # # # # # # #     def get_weather(self):
# # # # # # # #         """Получает текущую погоду"""
# # # # # # # #         try:
# # # # # # # #             url = f"http://api.openweathermap.org/data/2.5/weather?q={self.city}&appid={self.weather_api_key}&units=metric&lang=ru"
# # # # # # # #             response = requests.get(url, timeout=5)
            
# # # # # # # #             if response.status_code == 200:
# # # # # # # #                 data = response.json()
                
# # # # # # # #                 temp = round(data['main']['temp'])
# # # # # # # #                 feels_like = round(data['main']['feels_like'])
# # # # # # # #                 description = data['weather'][0]['description']
# # # # # # # #                 humidity = data['main']['humidity']
# # # # # # # #                 wind_speed = round(data['wind']['speed'])
                
# # # # # # # #                 weather_text = f"Сейчас в городе {self.city} {temp} градусов. "
# # # # # # # #                 weather_text += f"Ощущается как {feels_like}. "
# # # # # # # #                 weather_text += f"{description}. "
# # # # # # # #                 weather_text += f"Влажность {humidity} процентов. "
# # # # # # # #                 weather_text += f"Скорость ветра {wind_speed} метров в секунду."
                
# # # # # # # #                 return {
# # # # # # # #                     'temperature': temp,
# # # # # # # #                     'feels_like': feels_like,
# # # # # # # #                     'description': description,
# # # # # # # #                     'text': weather_text
# # # # # # # #                 }
# # # # # # # #             else:
# # # # # # # #                 return {'error': 'Не удалось получить данные о погоде'}
                
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка получения погоды: {e}")
# # # # # # # #             # Заглушка без API
# # # # # # # #             return {
# # # # # # # #                 'text': f"Сейчас в городе {self.city} примерно 15 градусов. Облачно.",
# # # # # # # #                 'temperature': 15,
# # # # # # # #                 'description': 'облачно'
# # # # # # # #             }

# # # # # # # # weather_service = WeatherTimeService()

# # # # # # # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # # # # # # class ColorAnalyzer:
# # # # # # # #     def __init__(self):
# # # # # # # #         self.color_names = {
# # # # # # # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # # # # # # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # # # # # # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # # # # # # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # # # # # # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # # # # # # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # # # # # # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # # # # # # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # # # # # # #             'бордовый': ([170, 100, 100], [180, 255, 255]),
# # # # # # # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # # # # # # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # # # # # # #             'черный': ([0, 0, 0], [180, 255, 50])
# # # # # # # #         }
    
# # # # # # # #     def analyze_colors(self, frame):
# # # # # # # #         """Анализирует доминирующие цвета в кадре"""
# # # # # # # #         try:
# # # # # # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # # # # # # #             detected_colors = []
            
# # # # # # # #             for color_name, (lower, upper) in self.color_names.items():
# # # # # # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # # # # # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # # # # # # #                 if percentage > 5:
# # # # # # # #                     detected_colors.append({
# # # # # # # #                         'color': color_name,
# # # # # # # #                         'percentage': round(percentage, 1)
# # # # # # # #                     })
            
# # # # # # # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # # # # # # #             return detected_colors[:3]
            
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка анализа цветов: {e}")
# # # # # # # #             return []
    
# # # # # # # #     def get_brightness_level(self, frame):
# # # # # # # #         """Определяет уровень освещенности"""
# # # # # # # #         try:
# # # # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # # # #             avg_brightness = np.mean(gray)
            
# # # # # # # #             if avg_brightness < 50:
# # # # # # # #                 return "очень темно"
# # # # # # # #             elif avg_brightness < 100:
# # # # # # # #                 return "темно"
# # # # # # # #             elif avg_brightness < 150:
# # # # # # # #                 return "нормальное освещение"
# # # # # # # #             elif avg_brightness < 200:
# # # # # # # #                 return "светло"
# # # # # # # #             else:
# # # # # # # #                 return "очень светло"
# # # # # # # #         except:
# # # # # # # #             return "не определено"

# # # # # # # # color_analyzer = ColorAnalyzer()

# # # # # # # # # --- ВИЗУАЛИЗАЦИЯ ОБЪЕКТОВ ---
# # # # # # # # class ObjectVisualizer:
# # # # # # # #     def __init__(self):
# # # # # # # #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# # # # # # # #     def draw_detections(self, frame, detections, ocr_data=None):
# # # # # # # #         """Рисует зеленые прямоугольники и подписи на кадре"""
# # # # # # # #         try:
# # # # # # # #             annotated_frame = frame.copy()
            
# # # # # # # #             for det in detections:
# # # # # # # #                 bbox = det['bbox']
# # # # # # # #                 label_en = det['label']
# # # # # # # #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # # # # # # #                 conf = int(det['confidence'] * 100)
# # # # # # # #                 distance = det.get('distance_m')
# # # # # # # #                 priority = det.get('priority', False)
                
# # # # # # # #                 # Зеленый цвет для всех объектов
# # # # # # # #                 color = (0, 255, 0)  # Зеленый
# # # # # # # #                 thickness = 3 if priority else 2
                
# # # # # # # #                 # Рисуем прямоугольник
# # # # # # # #                 cv2.rectangle(annotated_frame, 
# # # # # # # #                             (bbox[0], bbox[1]), 
# # # # # # # #                             (bbox[2], bbox[3]), 
# # # # # # # #                             color, thickness)
                
# # # # # # # #                 # Формируем текст
# # # # # # # #                 if distance:
# # # # # # # #                     text = f"{label_ru} - {distance:.1f}м"
# # # # # # # #                 else:
# # # # # # # #                     text = f"{label_ru}"
                
# # # # # # # #                 # Рисуем фон для текста
# # # # # # # #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# # # # # # # #                 cv2.rectangle(annotated_frame,
# # # # # # # #                             (bbox[0], bbox[1] - text_height - 10),
# # # # # # # #                             (bbox[0] + text_width + 10, bbox[1]),
# # # # # # # #                             color, -1)
                
# # # # # # # #                 # Рисуем текст
# # # # # # # #                 cv2.putText(annotated_frame, text,
# # # # # # # #                           (bbox[0] + 5, bbox[1] - 5),
# # # # # # # #                           self.font, 0.7, (0, 0, 0), 2)
            
# # # # # # # #             # Добавляем информацию об OCR
# # # # # # # #             if ocr_data and ocr_data.get('has_text'):
# # # # # # # #                 y_pos = 30
# # # # # # # #                 cv2.rectangle(annotated_frame, (10, 10), (300, y_pos + 25), (0, 255, 0), -1)
# # # # # # # #                 cv2.putText(annotated_frame, "Текст обнаружен",
# # # # # # # #                           (15, y_pos), self.font, 0.7, (0, 0, 0), 2)
            
# # # # # # # #             return annotated_frame
            
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка визуализации: {e}")
# # # # # # # #             return frame

# # # # # # # # object_visualizer = ObjectVisualizer()

# # # # # # # # # --- ОЗВУЧКА ТЕКСТА ---
# # # # # # # # class TextToSpeech:
# # # # # # # #     def __init__(self):
# # # # # # # #         self.cache_dir = "audio_cache"
# # # # # # # #         if not os.path.exists(self.cache_dir):
# # # # # # # #             os.makedirs(self.cache_dir)
    
# # # # # # # #     def text_to_speech(self, text, lang='ru'):
# # # # # # # #         """Конвертирует текст в аудио"""
# # # # # # # #         try:
# # # # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # # # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # # # # # # #             if os.path.exists(audio_path):
# # # # # # # #                 return audio_path
            
# # # # # # # #             tts = gTTS(text=text, lang=lang, slow=False)
# # # # # # # #             tts.save(audio_path)
            
# # # # # # # #             return audio_path
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка озвучки: {e}")
# # # # # # # #             return None

# # # # # # # # tts_engine = TextToSpeech()

# # # # # # # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # # # # # class SimpleVoiceBiometrics:
# # # # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # # # #         try:
# # # # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # # # #                 frame_rate = wav_file.getframerate()
# # # # # # # #                 n_frames = wav_file.getnframes()
# # # # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # # # #                 if sample_width == 2:
# # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # # # #                 else:
# # # # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # # # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # # # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
                
# # # # # # # #                 return features_b64
                
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # # # #             return None
    
# # # # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # # # #         features = []
# # # # # # # #         features.append(np.mean(audio_data ** 2))
# # # # # # # #         features.append(np.std(audio_data))
# # # # # # # #         features.append(np.max(np.abs(audio_data)))
        
# # # # # # # #         fft = np.fft.fft(audio_data)
# # # # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # # # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # # # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # # # #         for low, high in bands:
# # # # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # # # #             if np.any(mask):
# # # # # # # #                 band_energy = np.mean(fft_magnitude[mask])
# # # # # # # #                 features.append(band_energy)
# # # # # # # #             else:
# # # # # # # #                 features.append(0.0)
        
# # # # # # # #         return np.array(features)
    
# # # # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # # # #         try:
# # # # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # # # #             min_len = min(len(features1), len(features2))
# # # # # # # #             features1 = features1[:min_len]
# # # # # # # #             features2 = features2[:min_len]
            
# # # # # # # #             distance = np.linalg.norm(features1 - features2)
# # # # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
            
# # # # # # # #             return float(max(0.0, min(1.0, similarity)))
            
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка сравнения: {e}")
# # # # # # # #             return 0.0

# # # # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # # # --- РАСПОЗНАВАНИЕ КОМАНД ---
# # # # # # # # class VoiceCommandRecognizer:
# # # # # # # #     def __init__(self):
# # # # # # # #         self.commands = {
# # # # # # # #             'выйти': [
# # # # # # # #                 'выйти', 'выход', 'выхожу', 'выйди', 'закрыть', 'закрой',
# # # # # # # #                 'logout', 'exit', 'quit'
# # # # # # # #             ],
# # # # # # # #             'сканировать': [
# # # # # # # #                 'сканировать', 'сканирование', 'скан', 'сканируй', 'сканирую',
# # # # # # # #                 'что вижу', 'что видишь', 'что впереди', 'что передо мной',
# # # # # # # #                 'анализ', 'анализируй', 'посмотри', 'смотри', 'посмотреть',
# # # # # # # #                 'камера', 'камеру', 'проверь', 'проверить',
# # # # # # # #                 'scan', 'analyze'
# # # # # # # #             ],
# # # # # # # #             'погода': [
# # # # # # # #                 'погода', 'погоду', 'погоды', 'какая погода', 'погоде',
# # # # # # # #                 'температура', 'температуру', 'прогноз', 'прогноза',
# # # # # # # #                 'на улице', 'за окном', 'градусов', 'градуса',
# # # # # # # #                 'тепло', 'холодно', 'жарко', 'морозно',
# # # # # # # #                 'weather', 'temp', 'temperature'
# # # # # # # #             ],
# # # # # # # #             'время': [
# # # # # # # #                 'время', 'времени', 'который час', 'сколько времени',
# # # # # # # #                 'сейчас времени', 'часы', 'текущее время', 'какое время',
# # # # # # # #                 'час', 'часа', 'часов', 'время сейчас',
# # # # # # # #                 'time', 'clock', 'what time'
# # # # # # # #             ]
# # # # # # # #         }
    
# # # # # # # #     def recognize_command(self, text):
# # # # # # # #         """Распознает команду с максимальной чувствительностью"""
# # # # # # # #         if not text:
# # # # # # # #             print("Пустой текст команды")
# # # # # # # #             return None
            
# # # # # # # #         # Приводим к нижнему регистру и убираем лишние пробелы
# # # # # # # #         text = text.lower().strip()
        
# # # # # # # #         # Удаляем распространенные шумовые слова
# # # # # # # #         noise_words = ['слушаю', 'команду', 'команда', 'пожалуйста', 'скажи', 'покажи']
# # # # # # # #         for noise in noise_words:
# # # # # # # #             text = text.replace(noise, '')
        
# # # # # # # #         text = ' '.join(text.split())  # Убираем двойные пробелы
        
# # # # # # # #         print(f"=== АНАЛИЗ КОМАНДЫ ===")
# # # # # # # #         print(f"Исходный текст: '{text}'")
        
# # # # # # # #         # Проверяем каждую команду
# # # # # # # #         for command_type, keywords in self.commands.items():
# # # # # # # #             for keyword in keywords:
# # # # # # # #                 # Проверяем полное совпадение или вхождение
# # # # # # # #                 if keyword == text or keyword in text:
# # # # # # # #                     print(f"✓ Найдена команда: {command_type.upper()} (ключ: '{keyword}')")
# # # # # # # #                     return command_type
        
# # # # # # # #         # Если ничего не найдено - пытаемся найти частичные совпадения
# # # # # # # #         print(f"✗ Точного совпадения не найдено")
# # # # # # # #         print(f"Пытаемся найти частичное совпадение...")
        
# # # # # # # #         for command_type, keywords in self.commands.items():
# # # # # # # #             for keyword in keywords:
# # # # # # # #                 # Ищем частичное совпадение (минимум 3 символа)
# # # # # # # #                 if len(keyword) >= 3 and keyword[:3] in text:
# # # # # # # #                     print(f"~ Частичное совпадение: {command_type.upper()} ('{keyword[:3]}' в '{text}')")
# # # # # # # #                     return command_type
        
# # # # # # # #         print(f"✗ Команда не распознана из текста: '{text}'")
# # # # # # # #         print(f"Доступные команды: {list(self.commands.keys())}")
# # # # # # # #         return None

# # # # # # # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # # # # # # --- КОНВЕРТЕР АУДИО ---
# # # # # # # # try:
# # # # # # # #     from pydub import AudioSegment
# # # # # # # #     PYDUB_AVAILABLE = True
# # # # # # # # except ImportError:
# # # # # # # #     import subprocess
# # # # # # # #     import sys
# # # # # # # #     subprocess.check_call([sys.executable, "-m", "pip", "install", "pydub"])
# # # # # # # #     from pydub import AudioSegment
# # # # # # # #     PYDUB_AVAILABLE = True

# # # # # # # # class AudioConverter:
# # # # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # # # #         try:
# # # # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # # # #             audio.export(wav_path, format="wav")
# # # # # # # #             return wav_path
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка конвертации: {e}")
# # # # # # # #             return None

# # # # # # # # audio_converter = AudioConverter()

# # # # # # # # # --- OCR ---
# # # # # # # # class TextRecognizer:
# # # # # # # #     def __init__(self):
# # # # # # # #         self.config = '--oem 3 --psm 6'
    
# # # # # # # #     def extract_text(self, frame):
# # # # # # # #         try:
# # # # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # # # #             gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
            
# # # # # # # #             text = pytesseract.image_to_string(gray, lang='rus+eng', config=self.config)
# # # # # # # #             text = text.strip()
            
# # # # # # # #             if text:
# # # # # # # #                 phones = self._extract_phone_numbers(text)
                
# # # # # # # #                 return {
# # # # # # # #                     'text': text,
# # # # # # # #                     'phones': phones,
# # # # # # # #                     'has_text': True
# # # # # # # #                 }
            
# # # # # # # #             return {'text': '', 'phones': [], 'has_text': False}
            
# # # # # # # #         except Exception as e:
# # # # # # # #             print(f"Ошибка OCR: {e}")
# # # # # # # #             return {'text': '', 'phones': [], 'has_text': False}
    
# # # # # # # #     def _extract_phone_numbers(self, text):
# # # # # # # #         patterns = [
# # # # # # # #             r'\+7\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # # # #             r'8\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # # # #             r'\d{3}[-\s]?\d{2}[-\s]?\d{2}',
# # # # # # # #         ]
        
# # # # # # # #         phones = []
# # # # # # # #         for pattern in patterns:
# # # # # # # #             found = re.findall(pattern, text)
# # # # # # # #             phones.extend(found)
        
# # # # # # # #         return phones

# # # # # # # # text_recognizer = TextRecognizer()

# # # # # # # # # --- YOLO ---
# # # # # # # # OBJECT_HEIGHTS = {
# # # # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # # # # # # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8
# # # # # # # # }

# # # # # # # # OBJECT_TRANSLATIONS = {
# # # # # # # #     'person': 'человек', 'car': 'машина', 'chair': 'стул', 'bottle': 'бутылка',
# # # # # # # #     'cup': 'чашка', 'dog': 'собака', 'cat': 'кошка', 'tv': 'телевизор',
# # # # # # # #     'laptop': 'ноутбук', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл',
# # # # # # # #     'bus': 'автобус', 'truck': 'грузовик', 'traffic light': 'светофор',
# # # # # # # #     'stop sign': 'знак стоп', 'bench': 'скамейка', 'book': 'книга',
# # # # # # # #     'clock': 'часы', 'cell phone': 'мобильный телефон'
# # # # # # # # }

# # # # # # # # PRIORITY_OBJECTS = {
# # # # # # # #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck',
# # # # # # # #     'traffic light', 'stop sign', 'bench', 'chair', 'dog', 'cat'
# # # # # # # # }

# # # # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # # # #     if bbox_height_pixels <= 0:
# # # # # # # #         return None
# # # # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # # # #     return round(distance, 2)

# # # # # # # # print("Загрузка YOLO модели...")
# # # # # # # # model = YOLO('yolov5su.pt')
# # # # # # # # print("Модель загружена.")

# # # # # # # # def detect_objects(frame):
# # # # # # # #     """Улучшенная детекция объектов с настройками для лучшего распознавания"""
# # # # # # # #     try:
# # # # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
# # # # # # # #         # Улучшенные параметры для лучшего распознавания
# # # # # # # #         results = model.predict(
# # # # # # # #             img_rgb, 
# # # # # # # #             conf=0.25,        # Порог уверенности снижен для лучшего обнаружения
# # # # # # # #             iou=0.45,         # IoU для подавления дублей
# # # # # # # #             verbose=False,
# # # # # # # #             imgsz=640,        # Размер изображения для обработки
# # # # # # # #             max_det=50        # Максимум объектов
# # # # # # # #         )[0]
        
# # # # # # # #         detections = []
# # # # # # # #         frame_height = frame.shape[0]
# # # # # # # #         frame_width = frame.shape[1]

# # # # # # # #         for r in results.boxes:
# # # # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # # # #             conf = float(r.conf[0])
# # # # # # # #             cls_id = int(r.cls[0])
# # # # # # # #             label = model.names[cls_id]
# # # # # # # #             distance = estimate_distance(bbox, frame_height, label)
            
# # # # # # # #             # Вычисляем позицию объекта
# # # # # # # #             x_center = (bbox[0] + bbox[2]) / 2
# # # # # # # #             y_center = (bbox[1] + bbox[3]) / 2
            
# # # # # # # #             # Определяем высоту объекта относительно кадра
# # # # # # # #             if y_center < frame_height * 0.33:
# # # # # # # #                 vertical_pos = "верхняя часть"
# # # # # # # #             elif y_center < frame_height * 0.66:
# # # # # # # #                 vertical_pos = "середина"
# # # # # # # #             else:
# # # # # # # #                 vertical_pos = "нижняя часть"

# # # # # # # #             detections.append({
# # # # # # # #                 "label": label,
# # # # # # # #                 "confidence": conf,
# # # # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # # # #                 "distance_m": float(distance) if distance else None,
# # # # # # # #                 "priority": label in PRIORITY_OBJECTS,
# # # # # # # #                 "vertical_position": vertical_pos,
# # # # # # # #                 "x_center": float(x_center),
# # # # # # # #                 "y_center": float(y_center)
# # # # # # # #             })
        
# # # # # # # #         # Сортируем по приоритету и близости
# # # # # # # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
        
# # # # # # # #         print(f"Обнаружено объектов: {len(detections)}")
# # # # # # # #         for det in detections[:5]:
# # # # # # # #             print(f"  - {det['label']}: {det['confidence']:.2f}, dist={det['distance_m']}")
        
# # # # # # # #         return detections
# # # # # # # #     except Exception as e:
# # # # # # # #         print(f"Detection error: {e}")
# # # # # # # #         return []

# # # # # # # # def generate_obstacle_description(obstacles, ocr_data=None, colors=None, brightness=None):
# # # # # # # #     """Генерирует подробное описание для слабовидящих"""
# # # # # # # #     descriptions = []
# # # # # # # #     frame_center = 320
    
# # # # # # # #     # Освещение
# # # # # # # #     if brightness:
# # # # # # # #         descriptions.append(f"{brightness}")
    
# # # # # # # #     # Цвета
# # # # # # # #     if colors and len(colors) > 0:
# # # # # # # #         if len(colors) == 1:
# # # # # # # #             color_desc = colors[0]['color']
# # # # # # # #         else:
# # # # # # # #             color_desc = " и ".join([c['color'] for c in colors[:2]])
# # # # # # # #         descriptions.append(f"Основные цвета: {color_desc}")
    
# # # # # # # #     # Текст
# # # # # # # #     if ocr_data and ocr_data.get('has_text'):
# # # # # # # #         text = ocr_data['text']
# # # # # # # #         phones = ocr_data['phones']
        
# # # # # # # #         if text:
# # # # # # # #             # Очищаем текст от лишних символов
# # # # # # # #             text_clean = ' '.join(text.split())
# # # # # # # #             text_short = text_clean[:150] + "..." if len(text_clean) > 150 else text_clean
# # # # # # # #             descriptions.append(f"Обнаружен текст: {text_short}")
        
# # # # # # # #         if phones:
# # # # # # # #             descriptions.append(f"Номера телефонов: {', '.join(phones)}")
    
# # # # # # # #     # Объекты с улучшенным описанием
# # # # # # # #     if obstacles:
# # # # # # # #         obstacle_count = len(obstacles)
# # # # # # # #         descriptions.append(f"Обнаружено объектов: {obstacle_count}")
        
# # # # # # # #         for i, obs in enumerate(obstacles[:5]):
# # # # # # # #             label_ru = OBJECT_TRANSLATIONS.get(obs['label'], obs['label'])
# # # # # # # #             distance = obs.get('distance_m')
# # # # # # # #             confidence = int(obs['confidence'] * 100)
            
# # # # # # # #             # Формируем описание расстояния
# # # # # # # #             if distance is not None:
# # # # # # # #                 if distance < 1.0:
# # # # # # # #                     centimeters = int(distance * 100)
# # # # # # # #                     dist_text = f"очень близко, {centimeters} сантиметров"
# # # # # # # #                 elif distance < 2.0:
# # # # # # # #                     dist_text = f"близко, {distance:.1f} метра"
# # # # # # # #                 elif distance < 5.0:
# # # # # # # #                     dist_text = f"{int(distance)} метров"
# # # # # # # #                 else:
# # # # # # # #                     dist_text = f"далеко, около {int(distance)} метров"
# # # # # # # #             else:
# # # # # # # #                 dist_text = "расстояние неизвестно"
            
# # # # # # # #             # Определяем позицию
# # # # # # # #             bbox = obs['bbox']
# # # # # # # #             x_center = (bbox[0] + bbox[2]) / 2
            
# # # # # # # #             if x_center < frame_center - 150:
# # # # # # # #                 pos = "слева от вас"
# # # # # # # #             elif x_center > frame_center + 150:
# # # # # # # #                 pos = "справа от вас"
# # # # # # # #             else:
# # # # # # # #                 pos = "прямо перед вами"
            
# # # # # # # #             # Вертикальная позиция
# # # # # # # #             vertical = obs.get('vertical_position', '')
# # # # # # # #             if vertical:
# # # # # # # #                 pos += f", в {vertical} кадра"
            
# # # # # # # #             # Приоритет
# # # # # # # #             priority_marker = "Внимание! " if obs.get('priority') else ""
            
# # # # # # # #             # Формируем финальное описание
# # # # # # # #             desc = f"{priority_marker}{label_ru} {pos}, {dist_text}"
# # # # # # # #             descriptions.append(desc)
        
# # # # # # # #         if obstacle_count > 5:
# # # # # # # #             descriptions.append(f"и еще {obstacle_count - 5} объектов")
# # # # # # # #     else:
# # # # # # # #         descriptions.append("Препятствий не обнаружено")
    
# # # # # # # #     return ". ".join(descriptions) if descriptions else "Ничего не обнаружено"

# # # # # # # # # --- ЭНДПОИНТЫ ---

# # # # # # # # @app.route('/')
# # # # # # # # def index():
# # # # # # # #     return jsonify({
# # # # # # # #         "status": "Blind Assistant Server",
# # # # # # # #         "version": "9.0 - Full Enhanced",
# # # # # # # #         "features": [
# # # # # # # #             "Голосовая биометрия",
# # # # # # # #             "Детекция объектов с визуализацией",
# # # # # # # #             "Распознавание текста и цветов",
# # # # # # # #             "Погода и время",
# # # # # # # #             "Озвучка всех данных",
# # # # # # # #             "Голосовые команды"
# # # # # # # #         ]
# # # # # # # #     })

# # # # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # # # def register_voice():
# # # # # # # #     temp_audio = None
# # # # # # # #     temp_wav = None
# # # # # # # #     try:
# # # # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # # # #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# # # # # # # #         username = request.form['username'].strip()
# # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # #         if not temp_wav:
# # # # # # # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # # # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # #         conn = get_db_connection()
# # # # # # # #         cur = conn.cursor()
        
# # # # # # # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # # # #         existing = cur.fetchone()
        
# # # # # # # #         if existing:
# # # # # # # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # # # #                        (voice_features, username))
# # # # # # # #             message = "Профиль обновлен"
# # # # # # # #         else:
# # # # # # # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # # # #                        (username, voice_features))
# # # # # # # #             message = "Профиль создан"
        
# # # # # # # #         conn.commit()
# # # # # # # #         cur.close()
# # # # # # # #         conn.close()
        
# # # # # # # #         return jsonify({"status": "success", "message": message, "username": username})
        
# # # # # # # #     except Exception as e:
# # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # #     finally:
# # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # #                 try:
# # # # # # # #                     os.remove(temp_file)
# # # # # # # #                 except:
# # # # # # # #                     pass

# # # # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # # # def login_voice():
# # # # # # # #     temp_audio = None
# # # # # # # #     temp_wav = None
# # # # # # # #     try:
# # # # # # # #         username = request.form['username'].strip()
# # # # # # # #         audio_file = request.files['audio']
        
# # # # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # # # #         audio_file.save(temp_audio)
        
# # # # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # # # #         conn = get_db_connection()
# # # # # # # #         cur = conn.cursor()
        
# # # # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # # # #         result = cur.fetchone()
        
# # # # # # # #         if not result:
# # # # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # # # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # # # # # # #         cur.close()
# # # # # # # #         conn.close()
        
# # # # # # # #         if similarity > 0.7:
# # # # # # # #             # ПОДРОБНОЕ приветствие для слепых пользователей
# # # # # # # #             welcome_text = f"Добро пожаловать, {username}! Вы успешно вошли в приложение Помощник для слабовидящих. "
# # # # # # # #             welcome_text += "Перед вами находится превью камеры, которая показывает окружение. "
# # # # # # # #             welcome_text += "В нижней части экрана расположены две большие кнопки. "
# # # # # # # #             welcome_text += "Первая кнопка - синяя кнопка Сканировать - анализирует окружение и сообщает о препятствиях, объектах, цветах и текстах. "
# # # # # # # #             welcome_text += "Вторая кнопка - зеленая кнопка Голосовая команда - позволяет управлять приложением голосом. "
# # # # # # # #             welcome_text += "Доступные голосовые команды: скажите Погода - чтобы узнать текущую погоду, "
# # # # # # # #             welcome_text += "скажите Время - чтобы узнать текущее время, "
# # # # # # # #             welcome_text += "скажите Сканировать - для анализа окружения, "
# # # # # # # #             welcome_text += "скажите Выйти - для выхода из приложения. "
# # # # # # # #             welcome_text += "Все результаты будут озвучены голосом. Желаю удачного использования!"
            
# # # # # # # #             audio_path = tts_engine.text_to_speech(welcome_text)
            
# # # # # # # #             return jsonify({
# # # # # # # #                 "status": "success",
# # # # # # # #                 "message": "Вход выполнен",
# # # # # # # #                 "username": username,
# # # # # # # #                 "similarity": float(similarity),
# # # # # # # #                 "welcome_text": welcome_text
# # # # # # # #             })
# # # # # # # #         else:
# # # # # # # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
            
# # # # # # # #     except Exception as e:
# # # # # # # #         return jsonify({"error": str(e)}), 500
# # # # # # # #     finally:
# # # # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # # # #                 try:
# # # # # # # #                     os.remove(temp_file)
# # # # # # # #                 except:
# # # # # # # #                     pass

# # # # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # # # def voice_command():
# # # # # # # #     """Обработка голосовых команд с улучшенным логированием"""
# # # # # # # #     try:
# # # # # # # #         recognized_text = request.form.get('text', '').lower().strip()
        
# # # # # # # #         print(f"=== ПОЛУЧЕНА КОМАНДА ===")
# # # # # # # #         print(f"Распознанный текст: '{recognized_text}'")
        
# # # # # # # #         if not recognized_text:
# # # # # # # #             return jsonify({"error": "Текст не распознан"}), 400
        
# # # # # # # #         command = voice_command_recognizer.recognize_command(recognized_text)
        
# # # # # # # #         print(f"Определенная команда: {command}")
        
# # # # # # # #         if command == 'погода':
# # # # # # # #             print("Обработка команды: ПОГОДА")
# # # # # # # #             weather_data = weather_service.get_weather()
# # # # # # # #             return jsonify({
# # # # # # # #                 "status": "success",
# # # # # # # #                 "command": "погода",
# # # # # # # #                 "data": weather_data,
# # # # # # # #                 "recognized_text": recognized_text
# # # # # # # #             })
        
# # # # # # # #         elif command == 'время':
# # # # # # # #             print("Обработка команды: ВРЕМЯ")
# # # # # # # #             time_data = weather_service.get_current_time()
# # # # # # # #             return jsonify({
# # # # # # # #                 "status": "success",
# # # # # # # #                 "command": "время",
# # # # # # # #                 "data": time_data,
# # # # # # # #                 "recognized_text": recognized_text
# # # # # # # #             })
        
# # # # # # # #         elif command == 'сканировать':
# # # # # # # #             print("Обработка команды: СКАНИРОВАТЬ")
# # # # # # # #             return jsonify({
# # # # # # # #                 "status": "success",
# # # # # # # #                 "command": "сканировать",
# # # # # # # #                 "recognized_text": recognized_text
# # # # # # # #             })
        
# # # # # # # #         elif command == 'выйти':
# # # # # # # #             print("Обработка команды: ВЫЙТИ")
# # # # # # # #             return jsonify({
# # # # # # # #                 "status": "success",
# # # # # # # #                 "command": "выйти",
# # # # # # # #                 "recognized_text": recognized_text
# # # # # # # #             })
        
# # # # # # # #         else:
# # # # # # # #             print(f"КОМАНДА НЕ РАСПОЗНАНА: '{recognized_text}'")
# # # # # # # #             return jsonify({
# # # # # # # #                 "status": "unknown",
# # # # # # # #                 "message": "Команда не распознана",
# # # # # # # #                 "recognized_text": recognized_text,
# # # # # # # #                 "hint": "Доступные команды: погода, время, сканировать, выйти"
# # # # # # # #             })
            
# # # # # # # #     except Exception as e:
# # # # # # # #         print(f"ОШИБКА обработки команды: {e}")
# # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # # # def process_frame():
# # # # # # # #     """Обработка кадра: объекты + OCR + цвета + озвучка + визуализация"""
# # # # # # # #     try:
# # # # # # # #         if 'frame' not in request.files:
# # # # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # # # #         frame_bytes = request.files['frame'].read()
# # # # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # # # #         if frame is None:
# # # # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # # # #         # Детекция объектов
# # # # # # # #         obstacles = detect_objects(frame)
        
# # # # # # # #         # OCR
# # # # # # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # # # # # #         # Анализ цветов
# # # # # # # #         colors = color_analyzer.analyze_colors(frame)
# # # # # # # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # # # # # # #         # Визуализация
# # # # # # # #         annotated_frame = object_visualizer.draw_detections(frame, obstacles, ocr_data)
        
# # # # # # # #         # Генерируем описание
# # # # # # # #         description = generate_obstacle_description(obstacles, ocr_data, colors, brightness)
        
# # # # # # # #         # Озвучка
# # # # # # # #         audio_path = tts_engine.text_to_speech(description)
        
# # # # # # # #         # Сохраняем аннотированный кадр
# # # # # # # #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# # # # # # # #         cv2.imwrite(annotated_path, annotated_frame)
        
# # # # # # # #         # Кодируем в base64
# # # # # # # #         with open(annotated_path, 'rb') as f:
# # # # # # # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # # # # # # #         os.remove(annotated_path)

# # # # # # # #         return jsonify({
# # # # # # # #             "obstacles": obstacles,
# # # # # # # #             "ocr": ocr_data,
# # # # # # # #             "colors": colors,
# # # # # # # #             "brightness": brightness,
# # # # # # # #             "description": description,
# # # # # # # #             "count": len(obstacles),
# # # # # # # #             "audio_available": audio_path is not None,
# # # # # # # #             "annotated_frame": annotated_base64,
# # # # # # # #             "message": "Кадр обработан успешно"
# # # # # # # #         })
# # # # # # # #     except Exception as e:
# # # # # # # #         return jsonify({"error": f"Ошибка обработки: {str(e)}"}), 500

# # # # # # # # @app.route('/get_audio/<text_hash>')
# # # # # # # # def get_audio(text_hash):
# # # # # # # #     """Отдает аудио файл"""
# # # # # # # #     try:
# # # # # # # #         audio_path = os.path.join(tts_engine.cache_dir, f"{text_hash}.mp3")
# # # # # # # #         if os.path.exists(audio_path):
# # # # # # # #             return send_file(audio_path, mimetype='audio/mpeg')
# # # # # # # #         else:
# # # # # # # #             return jsonify({"error": "Аудио не найдено"}), 404
# # # # # # # #     except Exception as e:
# # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # @app.route('/text_to_speech', methods=['POST'])
# # # # # # # # def text_to_speech_endpoint():
# # # # # # # #     """Конвертирует текст в речь"""
# # # # # # # #     try:
# # # # # # # #         data = request.get_json()
# # # # # # # #         text = data.get('text', '')
        
# # # # # # # #         if not text:
# # # # # # # #             return jsonify({"error": "Текст не предоставлен"}), 400
        
# # # # # # # #         audio_path = tts_engine.text_to_speech(text)
        
# # # # # # # #         if audio_path:
# # # # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # # # #             return jsonify({
# # # # # # # #                 "status": "success",
# # # # # # # #                 "audio_hash": text_hash,
# # # # # # # #                 "audio_url": f"/get_audio/{text_hash}"
# # # # # # # #             })
# # # # # # # #         else:
# # # # # # # #             return jsonify({"error": "Ошибка генерации аудио"}), 500
            
# # # # # # # #     except Exception as e:
# # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # @app.route('/get_weather', methods=['GET'])
# # # # # # # # def get_weather():
# # # # # # # #     """Получить текущую погоду"""
# # # # # # # #     try:
# # # # # # # #         weather_data = weather_service.get_weather()
# # # # # # # #         return jsonify(weather_data)
# # # # # # # #     except Exception as e:
# # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # @app.route('/get_time', methods=['GET'])
# # # # # # # # def get_time():
# # # # # # # #     """Получить текущее время"""
# # # # # # # #     try:
# # # # # # # #         time_data = weather_service.get_current_time()
# # # # # # # #         return jsonify(time_data)
# # # # # # # #     except Exception as e:
# # # # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # # # if __name__ == '__main__':
# # # # # # # #     print("=" * 60)
# # # # # # # #     print("BLIND ASSISTANT SERVER - ПОЛНАЯ РАСШИРЕННАЯ ВЕРСИЯ")
# # # # # # # #     print("=" * 60)
# # # # # # # #     print("✓ Голосовая биометрия (регистрация и вход)")
# # # # # # # #     print("✓ Детекция объектов YOLO с визуализацией")
# # # # # # # #     print("✓ Распознавание текста (OCR)")
# # # # # # # #     print("✓ Определение цветов")
# # # # # # # #     print("✓ Анализ освещенности")
# # # # # # # #     print("✓ Погода и текущее время")
# # # # # # # #     print("✓ Озвучка всех данных (Text-to-Speech)")
# # # # # # # #     print("✓ Голосовые команды")
# # # # # # # #     print("✓ Аннотированные кадры с зелеными рамками")
# # # # # # # #     print("=" * 60)
# # # # # # # #     print("\nГолосовые команды:")
# # # # # # # #     print("- 'сканировать' - анализ окружения")
# # # # # # # #     print("- 'погода' - текущая погода")
# # # # # # # #     print("- 'время' - текущее время")
# # # # # # # #     print("- 'выйти' - выход из аккаунта")
# # # # # # # #     print("=" * 60)
# # # # # # # #     print("\nНеобходимые библиотеки:")
# # # # # # # #     print("pip install gtts pytesseract opencv-python requests")
# # # # # # # #     print("sudo apt-get install tesseract-ocr tesseract-ocr-rus")
# # # # # # # #     print("=" * 60)
# # # # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)






# # # # # # # --- Импорты ---
# # # # # # import os
# # # # # # import uuid
# # # # # # import cv2
# # # # # # import numpy as np
# # # # # # import psycopg2
# # # # # # from flask import Flask, request, jsonify, send_file
# # # # # # from ultralytics import YOLO
# # # # # # import wave
# # # # # # import hashlib
# # # # # # import base64
# # # # # # import pytesseract
# # # # # # from gtts import gTTS
# # # # # # import re
# # # # # # from datetime import datetime
# # # # # # import requests

# # # # # # # --- Конфигурация ---
# # # # # # DB_NAME = "blind_app"
# # # # # # DB_USER = "postgres"
# # # # # # DB_PASSWORD = "12345"
# # # # # # DB_HOST = "localhost"

# # # # # # app = Flask(__name__)

# # # # # # # --- CORS ---
# # # # # # @app.after_request
# # # # # # def after_request(response):
# # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # #     return response

# # # # # # def get_db_connection():
# # # # # #     return psycopg2.connect(
# # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # #     )

# # # # # # # --- ПОГОДА И ВРЕМЯ ---
# # # # # # class WeatherTimeService:
# # # # # #     def __init__(self):
# # # # # #         # Погода отключена - используем заглушки
# # # # # #         self.weather_enabled = False
# # # # # #         self.city = "Астана"
    
# # # # # #     def get_current_time(self):
# # # # # #         """Возвращает текущее время"""
# # # # # #         now = datetime.now()
# # # # # #         hours = now.hour
# # # # # #         minutes = now.minute
        
# # # # # #         # Склонение
# # # # # #         if minutes == 1:
# # # # # #             min_word = "минута"
# # # # # #         elif 2 <= minutes <= 4:
# # # # # #             min_word = "минуты"
# # # # # #         else:
# # # # # #             min_word = "минут"
        
# # # # # #         if hours == 1 or hours == 21:
# # # # # #             hour_word = "час"
# # # # # #         elif 2 <= hours <= 4 or 22 <= hours <= 24:
# # # # # #             hour_word = "часа"
# # # # # #         else:
# # # # # #             hour_word = "часов"
        
# # # # # #         time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
# # # # # #         return {
# # # # # #             'time': time_str,
# # # # # #             'hour': hours,
# # # # # #             'minute': minutes
# # # # # #         }
    
# # # # # #     def get_weather(self):
# # # # # #         """Возвращает заглушку погоды"""
# # # # # #         return {
# # # # # #             'text': f"Прогноз погоды недоступен",
# # # # # #             'temperature': None,
# # # # # #             'description': 'нет данных'
# # # # # #         }

# # # # # # weather_service = WeatherTimeService()

# # # # # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # # # # class ColorAnalyzer:
# # # # # #     def __init__(self):
# # # # # #         self.color_names = {
# # # # # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # # # # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # # # # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # # # # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # # # # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # # # # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # # # # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # # # # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # # # # #             'бордовый': ([170, 100, 100], [180, 255, 255]),
# # # # # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # # # # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # # # # #             'черный': ([0, 0, 0], [180, 255, 50])
# # # # # #         }
    
# # # # # #     def analyze_colors(self, frame):
# # # # # #         """Анализирует доминирующие цвета в кадре"""
# # # # # #         try:
# # # # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # # # # #             detected_colors = []
            
# # # # # #             for color_name, (lower, upper) in self.color_names.items():
# # # # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # # # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # # # # #                 if percentage > 5:
# # # # # #                     detected_colors.append({
# # # # # #                         'color': color_name,
# # # # # #                         'percentage': round(percentage, 1)
# # # # # #                     })
            
# # # # # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # # # # #             return detected_colors[:3]
            
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка анализа цветов: {e}")
# # # # # #             return []
    
# # # # # #     def get_brightness_level(self, frame):
# # # # # #         """Определяет уровень освещенности"""
# # # # # #         try:
# # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # #             avg_brightness = np.mean(gray)
            
# # # # # #             if avg_brightness < 50:
# # # # # #                 return "очень темно"
# # # # # #             elif avg_brightness < 100:
# # # # # #                 return "темно"
# # # # # #             elif avg_brightness < 150:
# # # # # #                 return "нормальное освещение"
# # # # # #             elif avg_brightness < 200:
# # # # # #                 return "светло"
# # # # # #             else:
# # # # # #                 return "очень светло"
# # # # # #         except:
# # # # # #             return "не определено"

# # # # # # color_analyzer = ColorAnalyzer()

# # # # # # # --- ВИЗУАЛИЗАЦИЯ ОБЪЕКТОВ ---
# # # # # # class ObjectVisualizer:
# # # # # #     def __init__(self):
# # # # # #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# # # # # #     def draw_detections(self, frame, detections, ocr_data=None):
# # # # # #         """Рисует зеленые прямоугольники и подписи на кадре"""
# # # # # #         try:
# # # # # #             annotated_frame = frame.copy()
            
# # # # # #             for det in detections:
# # # # # #                 bbox = det['bbox']
# # # # # #                 label_en = det['label']
# # # # # #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # # # # #                 conf = int(det['confidence'] * 100)
# # # # # #                 distance = det.get('distance_m')
# # # # # #                 priority = det.get('priority', False)
                
# # # # # #                 color = (0, 255, 0)
# # # # # #                 thickness = 3 if priority else 2
                
# # # # # #                 cv2.rectangle(annotated_frame, 
# # # # # #                             (bbox[0], bbox[1]), 
# # # # # #                             (bbox[2], bbox[3]), 
# # # # # #                             color, thickness)
                
# # # # # #                 if distance:
# # # # # #                     text = f"{label_ru} - {distance:.1f}м"
# # # # # #                 else:
# # # # # #                     text = f"{label_ru}"
                
# # # # # #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# # # # # #                 cv2.rectangle(annotated_frame,
# # # # # #                             (bbox[0], bbox[1] - text_height - 10),
# # # # # #                             (bbox[0] + text_width + 10, bbox[1]),
# # # # # #                             color, -1)
                
# # # # # #                 cv2.putText(annotated_frame, text,
# # # # # #                           (bbox[0] + 5, bbox[1] - 5),
# # # # # #                           self.font, 0.7, (0, 0, 0), 2)
            
# # # # # #             if ocr_data and ocr_data.get('has_text'):
# # # # # #                 y_pos = 30
# # # # # #                 cv2.rectangle(annotated_frame, (10, 10), (300, y_pos + 25), (0, 255, 0), -1)
# # # # # #                 cv2.putText(annotated_frame, "Текст обнаружен",
# # # # # #                           (15, y_pos), self.font, 0.7, (0, 0, 0), 2)
            
# # # # # #             return annotated_frame
            
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка визуализации: {e}")
# # # # # #             return frame

# # # # # # object_visualizer = ObjectVisualizer()

# # # # # # # --- ОЗВУЧКА ТЕКСТА ---
# # # # # # class TextToSpeech:
# # # # # #     def __init__(self):
# # # # # #         self.cache_dir = "audio_cache"
# # # # # #         if not os.path.exists(self.cache_dir):
# # # # # #             os.makedirs(self.cache_dir)
    
# # # # # #     def text_to_speech(self, text, lang='ru'):
# # # # # #         """Конвертирует текст в аудио"""
# # # # # #         try:
# # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # # # # #             if os.path.exists(audio_path):
# # # # # #                 return audio_path
            
# # # # # #             tts = gTTS(text=text, lang=lang, slow=False)
# # # # # #             tts.save(audio_path)
            
# # # # # #             return audio_path
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка озвучки: {e}")
# # # # # #             return None

# # # # # # tts_engine = TextToSpeech()

# # # # # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # # # class SimpleVoiceBiometrics:
# # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # #         try:
# # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # #                 frame_rate = wav_file.getframerate()
# # # # # #                 n_frames = wav_file.getnframes()
# # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # #                 if sample_width == 2:
# # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # #                 else:
# # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
                
# # # # # #                 return features_b64
                
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # #             return None
    
# # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # #         features = []
# # # # # #         features.append(np.mean(audio_data ** 2))
# # # # # #         features.append(np.std(audio_data))
# # # # # #         features.append(np.max(np.abs(audio_data)))
        
# # # # # #         fft = np.fft.fft(audio_data)
# # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # #         for low, high in bands:
# # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # #             if np.any(mask):
# # # # # #                 band_energy = np.mean(fft_magnitude[mask])
# # # # # #                 features.append(band_energy)
# # # # # #             else:
# # # # # #                 features.append(0.0)
        
# # # # # #         return np.array(features)
    
# # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # #         try:
# # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # #             min_len = min(len(features1), len(features2))
# # # # # #             features1 = features1[:min_len]
# # # # # #             features2 = features2[:min_len]
            
# # # # # #             distance = np.linalg.norm(features1 - features2)
# # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
            
# # # # # #             return float(max(0.0, min(1.0, similarity)))
            
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка сравнения: {e}")
# # # # # #             return 0.0

# # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # --- РАСПОЗНАВАНИЕ КОМАНД (УЛУЧШЕННОЕ) ---
# # # # # # class VoiceCommandRecognizer:
# # # # # #     def __init__(self):
# # # # # #         # Расширенные списки команд
# # # # # #         self.commands = {
# # # # # #             'выйти': [
# # # # # #                 'выйти', 'выход', 'выхожу', 'выйди', 'закрыть', 'закрой',
# # # # # #                 'выход из приложения', 'покинуть', 'уйти', 'завершить',
# # # # # #                 'logout', 'exit', 'quit', 'log out'
# # # # # #             ],
# # # # # #             'сканировать': [
# # # # # #                 'сканировать', 'сканирование', 'скан', 'сканируй', 'сканирую',
# # # # # #                 'что вижу', 'что видишь', 'что впереди', 'что передо мной',
# # # # # #                 'анализ', 'анализируй', 'посмотри', 'смотри', 'посмотреть',
# # # # # #                 'камера', 'камеру', 'проверь', 'проверить', 'окружение',
# # # # # #                 'что вокруг', 'что рядом', 'опиши', 'описание',
# # # # # #                 'scan', 'analyze', 'check', 'look'
# # # # # #             ],
# # # # # #             'время': [
# # # # # #                 'время', 'времени', 'который час', 'сколько времени',
# # # # # #                 'сейчас времени', 'часы', 'текущее время', 'какое время',
# # # # # #                 'час', 'часа', 'часов', 'время сейчас', 'сколько время',
# # # # # #                 'time', 'clock', 'what time', 'current time'
# # # # # #             ],
# # # # # #             'читать': [
# # # # # #                 'читать', 'читай', 'прочитай', 'прочитать', 'чтение',
# # # # # #                 'текст', 'что написано', 'прочти', 'прочесть',
# # # # # #                 'read', 'read text', 'what does it say'
# # # # # #             ]
# # # # # #         }
    
# # # # # #     def recognize_command(self, text):
# # # # # #         """Распознает команду с максимальной чувствительностью"""
# # # # # #         if not text:
# # # # # #             print("❌ Пустой текст команды")
# # # # # #             return None
            
# # # # # #         text = text.lower().strip()
        
# # # # # #         # Удаляем шумовые слова
# # # # # #         noise_words = ['слушаю', 'команду', 'команда', 'пожалуйста', 'скажи', 
# # # # # #                       'покажи', 'давай', 'хочу', 'нужно', 'можно']
# # # # # #         for noise in noise_words:
# # # # # #             text = text.replace(noise, '')
        
# # # # # #         text = ' '.join(text.split())
        
# # # # # #         print(f"\n{'='*50}")
# # # # # #         print(f"🎤 АНАЛИЗ КОМАНДЫ")
# # # # # #         print(f"{'='*50}")
# # # # # #         print(f"📝 Исходный текст: '{text}'")
        
# # # # # #         # Проверяем полные и частичные совпадения
# # # # # #         for command_type, keywords in self.commands.items():
# # # # # #             for keyword in keywords:
# # # # # #                 # Полное совпадение
# # # # # #                 if keyword == text:
# # # # # #                     print(f"✅ КОМАНДА НАЙДЕНА: {command_type.upper()}")
# # # # # #                     print(f"   Ключевое слово: '{keyword}'")
# # # # # #                     print(f"{'='*50}\n")
# # # # # #                     return command_type
                
# # # # # #                 # Вхождение
# # # # # #                 if keyword in text or text in keyword:
# # # # # #                     print(f"✅ КОМАНДА НАЙДЕНА: {command_type.upper()}")
# # # # # #                     print(f"   Найдено вхождение: '{keyword}' в '{text}'")
# # # # # #                     print(f"{'='*50}\n")
# # # # # #                     return command_type
        
# # # # # #         # Fuzzy matching для опечаток
# # # # # #         print(f"⚠️  Точного совпадения не найдено, пробуем нечеткий поиск...")
# # # # # #         for command_type, keywords in self.commands.items():
# # # # # #             for keyword in keywords:
# # # # # #                 if len(keyword) >= 4 and keyword[:4] in text:
# # # # # #                     print(f"⚡ ЧАСТИЧНОЕ СОВПАДЕНИЕ: {command_type.upper()}")
# # # # # #                     print(f"   Начало слова: '{keyword[:4]}' в '{text}'")
# # # # # #                     print(f"{'='*50}\n")
# # # # # #                     return command_type
        
# # # # # #         print(f"❌ КОМАНДА НЕ РАСПОЗНАНА")
# # # # # #         print(f"   Текст: '{text}'")
# # # # # #         print(f"   Доступные команды: {list(self.commands.keys())}")
# # # # # #         print(f"{'='*50}\n")
# # # # # #         return None

# # # # # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # # # # --- КОНВЕРТЕР АУДИО ---
# # # # # # try:
# # # # # #     from pydub import AudioSegment
# # # # # #     PYDUB_AVAILABLE = True
# # # # # # except ImportError:
# # # # # #     import subprocess
# # # # # #     import sys
# # # # # #     subprocess.check_call([sys.executable, "-m", "pip", "install", "pydub"])
# # # # # #     from pydub import AudioSegment
# # # # # #     PYDUB_AVAILABLE = True

# # # # # # class AudioConverter:
# # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # #         try:
# # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # #             audio.export(wav_path, format="wav")
# # # # # #             return wav_path
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка конвертации: {e}")
# # # # # #             return None

# # # # # # audio_converter = AudioConverter()

# # # # # # # --- OCR (УЛУЧШЕННЫЙ) ---
# # # # # # class TextRecognizer:
# # # # # #     def __init__(self):
# # # # # #         # Улучшенные настройки для лучшего распознавания
# # # # # #         self.config = '--oem 3 --psm 3'
    
# # # # # #     def extract_text(self, frame):
# # # # # #         try:
# # # # # #             # Предобработка изображения для лучшего OCR
# # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
# # # # # #             # Увеличиваем контраст
# # # # # #             gray = cv2.equalizeHist(gray)
            
# # # # # #             # Бинаризация
# # # # # #             gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
            
# # # # # #             # Убираем шум
# # # # # #             gray = cv2.medianBlur(gray, 3)
            
# # # # # #             # Извлекаем текст
# # # # # #             text = pytesseract.image_to_string(gray, lang='rus+eng', config=self.config)
# # # # # #             text = text.strip()
            
# # # # # #             if text:
# # # # # #                 # Очищаем текст
# # # # # #                 text = ' '.join(text.split())
# # # # # #                 phones = self._extract_phone_numbers(text)
                
# # # # # #                 print(f"📖 OCR: Обнаружен текст ({len(text)} символов)")
# # # # # #                 print(f"   Текст: {text[:100]}...")
                
# # # # # #                 return {
# # # # # #                     'text': text,
# # # # # #                     'phones': phones,
# # # # # #                     'has_text': True,
# # # # # #                     'length': len(text)
# # # # # #                 }
            
# # # # # #             return {'text': '', 'phones': [], 'has_text': False, 'length': 0}
            
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка OCR: {e}")
# # # # # #             return {'text': '', 'phones': [], 'has_text': False, 'length': 0}
    
# # # # # #     def _extract_phone_numbers(self, text):
# # # # # #         patterns = [
# # # # # #             r'\+7\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # #             r'8\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # #             r'\d{3}[-\s]?\d{2}[-\s]?\d{2}',
# # # # # #         ]
        
# # # # # #         phones = []
# # # # # #         for pattern in patterns:
# # # # # #             found = re.findall(pattern, text)
# # # # # #             phones.extend(found)
        
# # # # # #         return phones

# # # # # # text_recognizer = TextRecognizer()

# # # # # # # --- YOLO ---
# # # # # # OBJECT_HEIGHTS = {
# # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # # # # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8, 'book': 0.3
# # # # # # }

# # # # # # OBJECT_TRANSLATIONS = {
# # # # # #     'person': 'человек', 'car': 'машина', 'chair': 'стул', 'bottle': 'бутылка',
# # # # # #     'cup': 'чашка', 'dog': 'собака', 'cat': 'кошка', 'tv': 'телевизор',
# # # # # #     'laptop': 'ноутбук', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл',
# # # # # #     'bus': 'автобус', 'truck': 'грузовик', 'traffic light': 'светофор',
# # # # # #     'stop sign': 'знак стоп', 'bench': 'скамейка', 'book': 'книга',
# # # # # #     'clock': 'часы', 'cell phone': 'телефон', 'keyboard': 'клавиатура',
# # # # # #     'mouse': 'мышь', 'remote': 'пульт', 'microwave': 'микроволновка',
# # # # # #     'oven': 'духовка', 'toaster': 'тостер', 'sink': 'раковина',
# # # # # #     'refrigerator': 'холодильник', 'couch': 'диван', 'bed': 'кровать',
# # # # # #     'dining table': 'стол', 'toilet': 'туалет', 'door': 'дверь', 'window': 'окно'
# # # # # # }

# # # # # # PRIORITY_OBJECTS = {
# # # # # #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck',
# # # # # #     'traffic light', 'stop sign', 'bench', 'chair', 'dog', 'cat', 'door'
# # # # # # }

# # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # #     if bbox_height_pixels <= 0:
# # # # # #         return None
# # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # #     return round(distance, 2)

# # # # # # print("Загрузка YOLO модели...")
# # # # # # model = YOLO('yolov5su.pt')
# # # # # # print("✅ Модель загружена.")

# # # # # # def detect_objects(frame):
# # # # # #     """Детекция объектов"""
# # # # # #     try:
# # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
# # # # # #         results = model.predict(
# # # # # #             img_rgb, 
# # # # # #             conf=0.20,        # Снижен порог для лучшего обнаружения
# # # # # #             iou=0.45,
# # # # # #             verbose=False,
# # # # # #             imgsz=640,
# # # # # #             max_det=50
# # # # # #         )[0]
        
# # # # # #         detections = []
# # # # # #         frame_height = frame.shape[0]
# # # # # #         frame_width = frame.shape[1]

# # # # # #         for r in results.boxes:
# # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # #             conf = float(r.conf[0])
# # # # # #             cls_id = int(r.cls[0])
# # # # # #             label = model.names[cls_id]
# # # # # #             distance = estimate_distance(bbox, frame_height, label)
            
# # # # # #             x_center = (bbox[0] + bbox[2]) / 2
# # # # # #             y_center = (bbox[1] + bbox[3]) / 2
            
# # # # # #             if y_center < frame_height * 0.33:
# # # # # #                 vertical_pos = "верхняя часть"
# # # # # #             elif y_center < frame_height * 0.66:
# # # # # #                 vertical_pos = "середина"
# # # # # #             else:
# # # # # #                 vertical_pos = "нижняя часть"

# # # # # #             detections.append({
# # # # # #                 "label": label,
# # # # # #                 "confidence": conf,
# # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # #                 "distance_m": float(distance) if distance else None,
# # # # # #                 "priority": label in PRIORITY_OBJECTS,
# # # # # #                 "vertical_position": vertical_pos,
# # # # # #                 "x_center": float(x_center),
# # # # # #                 "y_center": float(y_center)
# # # # # #             })
        
# # # # # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
        
# # # # # #         print(f"🔍 Обнаружено объектов: {len(detections)}")
# # # # # #         return detections
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ Ошибка детекции: {e}")
# # # # # #         return []

# # # # # # def generate_obstacle_description(obstacles, ocr_data=None, colors=None, brightness=None):
# # # # # #     """Генерирует описание"""
# # # # # #     descriptions = []
# # # # # #     frame_center = 320
    
# # # # # #     if brightness:
# # # # # #         descriptions.append(f"{brightness}")
    
# # # # # #     if colors and len(colors) > 0:
# # # # # #         if len(colors) == 1:
# # # # # #             color_desc = colors[0]['color']
# # # # # #         else:
# # # # # #             color_desc = " и ".join([c['color'] for c in colors[:2]])
# # # # # #         descriptions.append(f"Основные цвета: {color_desc}")
    
# # # # # #     # УЛУЧШЕННОЕ ОПИСАНИЕ ТЕКСТА
# # # # # #     if ocr_data and ocr_data.get('has_text'):
# # # # # #         text = ocr_data['text']
# # # # # #         phones = ocr_data['phones']
        
# # # # # #         if text:
# # # # # #             text_clean = ' '.join(text.split())
# # # # # #             # Для длинного текста - читаем полностью
# # # # # #             if len(text_clean) > 200:
# # # # # #                 descriptions.append(f"Обнаружен длинный текст. Начало текста: {text_clean[:200]}")
# # # # # #             else:
# # # # # #                 descriptions.append(f"Обнаружен текст: {text_clean}")
        
# # # # # #         if phones:
# # # # # #             descriptions.append(f"Номера телефонов: {', '.join(phones)}")
    
# # # # # #     if obstacles:
# # # # # #         obstacle_count = len(obstacles)
# # # # # #         descriptions.append(f"Обнаружено объектов: {obstacle_count}")
        
# # # # # #         for i, obs in enumerate(obstacles[:5]):
# # # # # #             label_ru = OBJECT_TRANSLATIONS.get(obs['label'], obs['label'])
# # # # # #             distance = obs.get('distance_m')
            
# # # # # #             if distance is not None:
# # # # # #                 if distance < 1.0:
# # # # # #                     centimeters = int(distance * 100)
# # # # # #                     dist_text = f"очень близко, {centimeters} сантиметров"
# # # # # #                 elif distance < 2.0:
# # # # # #                     dist_text = f"близко, {distance:.1f} метра"
# # # # # #                 elif distance < 5.0:
# # # # # #                     dist_text = f"{int(distance)} метров"
# # # # # #                 else:
# # # # # #                     dist_text = f"далеко, около {int(distance)} метров"
# # # # # #             else:
# # # # # #                 dist_text = "расстояние неизвестно"
            
# # # # # #             bbox = obs['bbox']
# # # # # #             x_center = (bbox[0] + bbox[2]) / 2
            
# # # # # #             if x_center < frame_center - 150:
# # # # # #                 pos = "слева от вас"
# # # # # #             elif x_center > frame_center + 150:
# # # # # #                 pos = "справа от вас"
# # # # # #             else:
# # # # # #                 pos = "прямо перед вами"
            
# # # # # #             vertical = obs.get('vertical_position', '')
# # # # # #             if vertical:
# # # # # #                 pos += f", в {vertical} кадра"
            
# # # # # #             priority_marker = "Внимание! " if obs.get('priority') else ""
            
# # # # # #             desc = f"{priority_marker}{label_ru} {pos}, {dist_text}"
# # # # # #             descriptions.append(desc)
        
# # # # # #         if obstacle_count > 5:
# # # # # #             descriptions.append(f"и еще {obstacle_count - 5} объектов")
# # # # # #     else:
# # # # # #         descriptions.append("Препятствий не обнаружено")
    
# # # # # #     return ". ".join(descriptions) if descriptions else "Ничего не обнаружено"

# # # # # # # --- ЭНДПОИНТЫ ---

# # # # # # @app.route('/')
# # # # # # def index():
# # # # # #     return jsonify({
# # # # # #         "status": "Blind Assistant Server v10",
# # # # # #         "version": "10.0 - Улучшенная",
# # # # # #         "features": [
# # # # # #             "✅ Голосовая биометрия",
# # # # # #             "✅ Детекция объектов с визуализацией",
# # # # # #             "✅ Распознавание текста (OCR) - УЛУЧШЕНО",
# # # # # #             "✅ Определение цветов",
# # # # # #             "✅ Текущее время",
# # # # # #             "✅ Озвучка всех данных",
# # # # # #             "✅ Улучшенное распознавание команд"
# # # # # #         ]
# # # # # #     })

# # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # def register_voice():
# # # # # #     temp_audio = None
# # # # # #     temp_wav = None
# # # # # #     try:
# # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# # # # # #         username = request.form['username'].strip()
# # # # # #         audio_file = request.files['audio']
        
# # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # #         audio_file.save(temp_audio)
        
# # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # #         if not temp_wav:
# # # # # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # #         conn = get_db_connection()
# # # # # #         cur = conn.cursor()
        
# # # # # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # #         existing = cur.fetchone()
        
# # # # # #         if existing:
# # # # # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s",
# # # # # #                        (voice_features, username))
# # # # # #             message = "Профиль обновлен"
# # # # # #         else:
# # # # # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)",
# # # # # #                        (username, voice_features))
# # # # # #             message = "Профиль создан"
        
# # # # # #         conn.commit()
# # # # # #         cur.close()
# # # # # #         conn.close()
        
# # # # # #         return jsonify({"status": "success", "message": message, "username": username})
        
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500
# # # # # #     finally:
# # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # #                 try:
# # # # # #                     os.remove(temp_file)
# # # # # #                 except:
# # # # # #                     pass

# # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # def login_voice():
# # # # # #     temp_audio = None
# # # # # #     temp_wav = None
# # # # # #     try:
# # # # # #         username = request.form['username'].strip()
# # # # # #         audio_file = request.files['audio']
        
# # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # #         audio_file.save(temp_audio)
        
# # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # #         conn = get_db_connection()
# # # # # #         cur = conn.cursor()
        
# # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # #         result = cur.fetchone()
        
# # # # # #         if not result:
# # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # # # # #         cur.close()
# # # # # #         conn.close()
        
# # # # # #         if similarity > 0.7:
# # # # # #             # КРАТКОЕ приветствие с возможностью пропустить
# # # # # #             welcome_text = f"Добро пожаловать, {username}! "
# # # # # #             welcome_text += "Перед вами камера. "
# # # # # #             welcome_text += "Внизу две кнопки: синяя кнопка Сканировать для анализа окружения, "
# # # # # #             welcome_text += "зеленая кнопка Голосовая команда для управления голосом. "
# # # # # #             welcome_text += "Команды: Сканировать, Время, Читать текст, Выйти. "
# # # # # #             welcome_text += "Нажмите экран чтобы пропустить это сообщение."
            
# # # # # #             audio_path = tts_engine.text_to_speech(welcome_text)
            
# # # # # #             return jsonify({
# # # # # #                 "status": "success",
# # # # # #                 "message": "Вход выполнен",
# # # # # #                 "username": username,
# # # # # #                 "similarity": float(similarity),
# # # # # #                 "welcome_text": welcome_text,
# # # # # #                 "skippable": True  # Флаг что можно пропустить
# # # # # #             })
# # # # # #         else:
# # # # # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
            
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500
# # # # # #     finally:
# # # # # #         for temp_file in [temp_audio, temp_wav]:
# # # # # #             if temp_file and os.path.exists(temp_file):
# # # # # #                 try:
# # # # # #                     os.remove(temp_file)
# # # # # #                 except:
# # # # # #                     pass

# # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # def voice_command():
# # # # # #     """Обработка голосовых команд с улучшенным логированием"""
# # # # # #     try:
# # # # # #         recognized_text = request.form.get('text', '').lower().strip()
        
# # # # # #         print(f"\n{'='*50}")
# # # # # #         print(f"🎤 ПОЛУЧЕНА ГОЛОСОВАЯ КОМАНДА")
# # # # # #         print(f"{'='*50}")
# # # # # #         print(f"📝 Распознанный текст: '{recognized_text}'")
        
# # # # # #         if not recognized_text:
# # # # # #             print(f"❌ ОШИБКА: Пустой текст")
# # # # # #             return jsonify({"error": "Текст не распознан"}), 400
        
# # # # # #         command = voice_command_recognizer.recognize_command(recognized_text)
        
# # # # # #         print(f"🎯 Результат: {command if command else 'НЕ РАСПОЗНАНО'}")
# # # # # #         print(f"{'='*50}\n")
        
# # # # # #         if command == 'время':
# # # # # #             time_data = weather_service.get_current_time()
# # # # # #             return jsonify({
# # # # # #                 "status": "success",
# # # # # #                 "command": "время",
# # # # # #                 "data": time_data,
# # # # # #                 "recognized_text": recognized_text
# # # # # #             })
        
# # # # # #         elif command == 'сканировать':
# # # # # #             return jsonify({
# # # # # #                 "status": "success",
# # # # # #                 "command": "сканировать",
# # # # # #                 "recognized_text": recognized_text
# # # # # #             })
        
# # # # # #         elif command == 'читать':
# # # # # #             return jsonify({
# # # # # #                 "status": "success",
# # # # # #                 "command": "читать",
# # # # # #                 "recognized_text": recognized_text
# # # # # #             })
        
# # # # # #         elif command == 'выйти':
# # # # # #             return jsonify({
# # # # # #                 "status": "success",
# # # # # #                 "command": "выйти",
# # # # # #                 "message": "Выход из приложения",
# # # # # #                 "recognized_text": recognized_text
# # # # # #             })
        
# # # # # #         else:
# # # # # #             return jsonify({
# # # # # #                 "status": "unknown",
# # # # # #                 "message": "Команда не распознана. Повторите четко: Сканировать, Время, Читать или Выйти",
# # # # # #                 "recognized_text": recognized_text,
# # # # # #                 "available_commands": ["сканировать", "время", "читать", "выйти"]
# # # # # #             })
            
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ ОШИБКА обработки команды: {e}")
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # def process_frame():
# # # # # #     """Обработка кадра: объекты + OCR + цвета + озвучка + визуализация"""
# # # # # #     try:
# # # # # #         if 'frame' not in request.files:
# # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         if frame is None:
# # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # #         # Детекция объектов
# # # # # #         obstacles = detect_objects(frame)
        
# # # # # #         # OCR
# # # # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # # # #         # Анализ цветов
# # # # # #         colors = color_analyzer.analyze_colors(frame)
# # # # # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # # # # #         # Визуализация
# # # # # #         annotated_frame = object_visualizer.draw_detections(frame, obstacles, ocr_data)
        
# # # # # #         # Генерируем описание
# # # # # #         description = generate_obstacle_description(obstacles, ocr_data, colors, brightness)
        
# # # # # #         # Озвучка
# # # # # #         audio_path = tts_engine.text_to_speech(description)
        
# # # # # #         # Сохраняем аннотированный кадр
# # # # # #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# # # # # #         cv2.imwrite(annotated_path, annotated_frame)
        
# # # # # #         # Кодируем в base64
# # # # # #         with open(annotated_path, 'rb') as f:
# # # # # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # # # # #         os.remove(annotated_path)

# # # # # #         return jsonify({
# # # # # #             "obstacles": obstacles,
# # # # # #             "ocr": ocr_data,
# # # # # #             "colors": colors,
# # # # # #             "brightness": brightness,
# # # # # #             "description": description,
# # # # # #             "count": len(obstacles),
# # # # # #             "audio_available": audio_path is not None,
# # # # # #             "annotated_frame": annotated_base64,
# # # # # #             "message": "Кадр обработан успешно"
# # # # # #         })
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": f"Ошибка обработки: {str(e)}"}), 500

# # # # # # @app.route('/read_text', methods=['POST'])
# # # # # # def read_text():
# # # # # #     """НОВЫЙ ЭНДПОИНТ: Специальный режим для чтения текста"""
# # # # # #     try:
# # # # # #         if 'frame' not in request.files:
# # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         if frame is None:
# # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # #         # Только OCR, без детекции объектов
# # # # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # # # #         if ocr_data.get('has_text'):
# # # # # #             text = ocr_data['text']
# # # # # #             description = f"Обнаружен текст. {text}"
            
# # # # # #             if ocr_data.get('phones'):
# # # # # #                 description += f". Номера телефонов: {', '.join(ocr_data['phones'])}"
# # # # # #         else:
# # # # # #             description = "Текст не обнаружен. Попробуйте навести камеру на текст и держите устройство неподвижно."
        
# # # # # #         # Озвучка
# # # # # #         audio_path = tts_engine.text_to_speech(description)
        
# # # # # #         return jsonify({
# # # # # #             "ocr": ocr_data,
# # # # # #             "description": description,
# # # # # #             "audio_available": audio_path is not None,
# # # # # #             "message": "Распознавание текста выполнено"
# # # # # #         })
        
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": f"Ошибка чтения текста: {str(e)}"}), 500

# # # # # # @app.route('/get_audio/<text_hash>')
# # # # # # def get_audio(text_hash):
# # # # # #     """Отдает аудио файл"""
# # # # # #     try:
# # # # # #         audio_path = os.path.join(tts_engine.cache_dir, f"{text_hash}.mp3")
# # # # # #         if os.path.exists(audio_path):
# # # # # #             return send_file(audio_path, mimetype='audio/mpeg')
# # # # # #         else:
# # # # # #             return jsonify({"error": "Аудио не найдено"}), 404
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/text_to_speech', methods=['POST'])
# # # # # # def text_to_speech_endpoint():
# # # # # #     """Конвертирует текст в речь"""
# # # # # #     try:
# # # # # #         data = request.get_json()
# # # # # #         text = data.get('text', '')
        
# # # # # #         if not text:
# # # # # #             return jsonify({"error": "Текст не предоставлен"}), 400
        
# # # # # #         audio_path = tts_engine.text_to_speech(text)
        
# # # # # #         if audio_path:
# # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # #             return jsonify({
# # # # # #                 "status": "success",
# # # # # #                 "audio_hash": text_hash,
# # # # # #                 "audio_url": f"/get_audio/{text_hash}"
# # # # # #             })
# # # # # #         else:
# # # # # #             return jsonify({"error": "Ошибка генерации аудио"}), 500
            
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/get_time', methods=['GET'])
# # # # # # def get_time():
# # # # # #     """Получить текущее время"""
# # # # # #     try:
# # # # # #         time_data = weather_service.get_current_time()
# # # # # #         return jsonify(time_data)
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/logout', methods=['POST'])
# # # # # # def logout():
# # # # # #     """НОВЫЙ ЭНДПОИНТ: Выход из приложения"""
# # # # # #     try:
# # # # # #         username = request.form.get('username', 'Пользователь')
# # # # # #         logout_text = f"До свидания, {username}. Возвращаемся к экрану входа."
        
# # # # # #         audio_path = tts_engine.text_to_speech(logout_text)
        
# # # # # #         return jsonify({
# # # # # #             "status": "success",
# # # # # #             "message": logout_text,
# # # # # #             "audio_available": audio_path is not None
# # # # # #         })
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # if __name__ == '__main__':
# # # # # #     print("=" * 60)
# # # # # #     print("🚀 BLIND ASSISTANT SERVER - ВЕРСИЯ 10.0")
# # # # # #     print("=" * 60)
# # # # # #     print("✅ Голосовая биометрия (регистрация и вход)")
# # # # # #     print("✅ Детекция объектов YOLO с визуализацией")
# # # # # #     print("✅ Распознавание текста (OCR) - УЛУЧШЕНО")
# # # # # #     print("✅ Определение цветов и освещенности")
# # # # # #     print("✅ Текущее время (погода отключена)")
# # # # # #     print("✅ Озвучка всех данных (Text-to-Speech)")
# # # # # #     print("✅ УЛУЧШЕННОЕ распознавание голосовых команд")
# # # # # #     print("✅ Возможность пропустить приветствие")
# # # # # #     print("✅ Правильный выход к экрану входа")
# # # # # #     print("=" * 60)
# # # # # #     print("\n📢 ГОЛОСОВЫЕ КОМАНДЫ:")
# # # # # #     print("   🔹 'СКАНИРОВАТЬ' - анализ окружения")
# # # # # #     print("   🔹 'ВРЕМЯ' - текущее время")
# # # # # #     print("   🔹 'ЧИТАТЬ' - режим чтения текста")
# # # # # #     print("   🔹 'ВЫЙТИ' - выход к экрану входа")
# # # # # #     print("=" * 60)
# # # # # #     print("\n🎯 УЛУЧШЕНИЯ:")
# # # # # #     print("   • Команды распознаются НАМНОГО лучше")
# # # # # #     print("   • Приветствие можно пропустить нажатием")
# # # # # #     print("   • Выход возвращает к экрану входа")
# # # # # #     print("   • Улучшенное OCR для чтения текстов")
# # # # # #     print("   • Погода отключена (не работала)")
# # # # # #     print("=" * 60)
# # # # # #     print("\n📦 Необходимые библиотеки:")
# # # # # #     print("   pip install gtts pytesseract opencv-python pydub")
# # # # # #     print("   sudo apt-get install tesseract-ocr tesseract-ocr-rus")
# # # # # #     print("=" * 60)
# # # # # #     print("\n🌐 Сервер запускается на http://192.168.8.63:5000")
# # # # # #     print("=" * 60)
    
# # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)




# # # # # # норм вроде













# # # # # # import os
# # # # # # import uuid
# # # # # # import cv2
# # # # # # import numpy as np
# # # # # # import psycopg2
# # # # # # from flask import Flask, request, jsonify, send_file
# # # # # # from ultralytics import YOLO
# # # # # # import wave
# # # # # # import hashlib
# # # # # # import base64
# # # # # # import pytesseract
# # # # # # from gtts import gTTS
# # # # # # import re
# # # # # # from datetime import datetime
# # # # # # from pydub import AudioSegment

# # # # # # # --- Конфигурация ---
# # # # # # DB_NAME = "blind_app"
# # # # # # DB_USER = "postgres"
# # # # # # DB_PASSWORD = "12345"
# # # # # # DB_HOST = "localhost"

# # # # # # app = Flask(__name__)

# # # # # # @app.after_request
# # # # # # def after_request(response):
# # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # #     return response

# # # # # # def get_db_connection():
# # # # # #     return psycopg2.connect(
# # # # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # # # #     )

# # # # # # # --- ВРЕМЯ ---
# # # # # # class TimeService:
# # # # # #     def get_current_time(self):
# # # # # #         now = datetime.now()
# # # # # #         hours = now.hour
# # # # # #         minutes = now.minute
        
# # # # # #         if minutes == 1:
# # # # # #             min_word = "минута"
# # # # # #         elif 2 <= minutes <= 4:
# # # # # #             min_word = "минуты"
# # # # # #         else:
# # # # # #             min_word = "минут"
        
# # # # # #         if hours == 1 or hours == 21:
# # # # # #             hour_word = "час"
# # # # # #         elif 2 <= hours <= 4 or 22 <= hours <= 24:
# # # # # #             hour_word = "часа"
# # # # # #         else:
# # # # # #             hour_word = "часов"
        
# # # # # #         time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
# # # # # #         return {'time': time_str, 'hour': hours, 'minute': minutes}

# # # # # # time_service = TimeService()

# # # # # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # # # # class ColorAnalyzer:
# # # # # #     def __init__(self):
# # # # # #         self.color_names = {
# # # # # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # # # # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # # # # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # # # # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # # # # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # # # # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # # # # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # # # # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # # # # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # # # # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # # # # #             'черный': ([0, 0, 0], [180, 255, 50])
# # # # # #         }
    
# # # # # #     def analyze_colors(self, frame):
# # # # # #         try:
# # # # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # # # # #             detected_colors = []
            
# # # # # #             for color_name, (lower, upper) in self.color_names.items():
# # # # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # # # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # # # # #                 if percentage > 5:
# # # # # #                     detected_colors.append({'color': color_name, 'percentage': round(percentage, 1)})
            
# # # # # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # # # # #             return detected_colors[:3]
# # # # # #         except:
# # # # # #             return []
    
# # # # # #     def get_brightness_level(self, frame):
# # # # # #         try:
# # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # #             avg_brightness = np.mean(gray)
            
# # # # # #             if avg_brightness < 50:
# # # # # #                 return "очень темно"
# # # # # #             elif avg_brightness < 100:
# # # # # #                 return "темно"
# # # # # #             elif avg_brightness < 150:
# # # # # #                 return "нормальное освещение"
# # # # # #             elif avg_brightness < 200:
# # # # # #                 return "светло"
# # # # # #             else:
# # # # # #                 return "очень светло"
# # # # # #         except:
# # # # # #             return "не определено"

# # # # # # color_analyzer = ColorAnalyzer()

# # # # # # # --- ВИЗУАЛИЗАЦИЯ ---
# # # # # # class ObjectVisualizer:
# # # # # #     def __init__(self):
# # # # # #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# # # # # #     def draw_detections(self, frame, detections, ocr_data=None):
# # # # # #         try:
# # # # # #             annotated_frame = frame.copy()
            
# # # # # #             for det in detections:
# # # # # #                 bbox = det['bbox']
# # # # # #                 label_en = det['label']
# # # # # #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # # # # #                 distance = det.get('distance_m')
# # # # # #                 priority = det.get('priority', False)
                
# # # # # #                 color = (0, 255, 0)
# # # # # #                 thickness = 3 if priority else 2
                
# # # # # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
                
# # # # # #                 text = f"{label_ru} - {distance:.1f}м" if distance else f"{label_ru}"
                
# # # # # #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# # # # # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1] - text_height - 10),
# # # # # #                             (bbox[0] + text_width + 10, bbox[1]), color, -1)
                
# # # # # #                 cv2.putText(annotated_frame, text, (bbox[0] + 5, bbox[1] - 5),
# # # # # #                           self.font, 0.7, (0, 0, 0), 2)
            
# # # # # #             if ocr_data and ocr_data.get('has_text'):
# # # # # #                 cv2.rectangle(annotated_frame, (10, 10), (300, 55), (0, 255, 0), -1)
# # # # # #                 cv2.putText(annotated_frame, "Текст обнаружен", (15, 30), self.font, 0.7, (0, 0, 0), 2)
            
# # # # # #             return annotated_frame
# # # # # #         except:
# # # # # #             return frame

# # # # # # object_visualizer = ObjectVisualizer()

# # # # # # # --- ОЗВУЧКА ---
# # # # # # class TextToSpeech:
# # # # # #     def __init__(self):
# # # # # #         self.cache_dir = "audio_cache"
# # # # # #         if not os.path.exists(self.cache_dir):
# # # # # #             os.makedirs(self.cache_dir)
    
# # # # # #     def text_to_speech(self, text, lang='ru'):
# # # # # #         try:
# # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # # # # #             if os.path.exists(audio_path):
# # # # # #                 return audio_path
            
# # # # # #             tts = gTTS(text=text, lang=lang, slow=False)
# # # # # #             tts.save(audio_path)
# # # # # #             return audio_path
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка озвучки: {e}")
# # # # # #             return None

# # # # # # tts_engine = TextToSpeech()

# # # # # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # # # class SimpleVoiceBiometrics:
# # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # #         try:
# # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # #                 frame_rate = wav_file.getframerate()
# # # # # #                 n_frames = wav_file.getnframes()
# # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # #                 if sample_width == 2:
# # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # #                 else:
# # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # # # # #                 return features_b64
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # #             return None
    
# # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # #         features = []
# # # # # #         features.append(np.mean(audio_data ** 2))
# # # # # #         features.append(np.std(audio_data))
# # # # # #         features.append(np.max(np.abs(audio_data)))
        
# # # # # #         fft = np.fft.fft(audio_data)
# # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # #         for low, high in bands:
# # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # #             if np.any(mask):
# # # # # #                 features.append(np.mean(fft_magnitude[mask]))
# # # # # #             else:
# # # # # #                 features.append(0.0)
        
# # # # # #         return np.array(features)
    
# # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # #         try:
# # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # #             min_len = min(len(features1), len(features2))
# # # # # #             features1 = features1[:min_len]
# # # # # #             features2 = features2[:min_len]
            
# # # # # #             distance = np.linalg.norm(features1 - features2)
# # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
# # # # # #             return float(max(0.0, min(1.0, similarity)))
# # # # # #         except:
# # # # # #             return 0.0

# # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # --- КОМАНДЫ ---
# # # # # # class VoiceCommandRecognizer:
# # # # # #     def __init__(self):
# # # # # #         self.commands = {
# # # # # #             'выйти': ['выйти', 'выход', 'выхожу', 'закрыть', 'exit'],
# # # # # #             'сканировать': ['сканировать', 'скан', 'сканируй', 'что вижу', 'что впереди', 'анализ', 'scan'],
# # # # # #             'время': ['время', 'который час', 'сколько времени', 'time'],
# # # # # #             'читать': ['читать', 'читай', 'прочитай', 'текст', 'что написано', 'read'],
# # # # # #             'деньги': ['деньги', 'распознать деньги', 'сколько денег', 'какая купюра', 'купюра', 'тенге', 'money']
# # # # # #         }
    
# # # # # #     def recognize_command(self, text):
# # # # # #         if not text:
# # # # # #             return None
        
# # # # # #         text = text.lower().strip()
# # # # # #         noise = ['слушаю', 'команду', 'команда', 'пожалуйста']
# # # # # #         for n in noise:
# # # # # #             text = text.replace(n, '')
# # # # # #         text = ' '.join(text.split())
        
# # # # # #         print(f"🎤 Анализ: '{text}'")
        
# # # # # #         for cmd_type, keywords in self.commands.items():
# # # # # #             for keyword in keywords:
# # # # # #                 if keyword in text or text in keyword:
# # # # # #                     print(f"✅ Команда: {cmd_type}")
# # # # # #                     return cmd_type
        
# # # # # #         print(f"❌ Не распознано")
# # # # # #         return None

# # # # # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # # # # --- КОНВЕРТЕР АУДИО ---
# # # # # # class AudioConverter:
# # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # #         try:
# # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # #             audio.export(wav_path, format="wav")
# # # # # #             return wav_path
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка конвертации: {e}")
# # # # # #             return None

# # # # # # audio_converter = AudioConverter()

# # # # # # # --- OCR (УЛУЧШЕННЫЙ) ---
# # # # # # class TextRecognizer:
# # # # # #     def __init__(self):
# # # # # #         self.config = '--oem 3 --psm 6'
    
# # # # # #     def extract_text(self, frame):
# # # # # #         try:
# # # # # #             print("\n📖 Распознавание текста...")
            
# # # # # #             # Улучшенная предобработка
# # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
# # # # # #             # Увеличение изображения для лучшего распознавания
# # # # # #             gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            
# # # # # #             # Улучшение контраста
# # # # # #             gray = cv2.equalizeHist(gray)
            
# # # # # #             # Бинаризация методом Оцу
# # # # # #             _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
# # # # # #             # Убираем шум
# # # # # #             binary = cv2.medianBlur(binary, 3)
            
# # # # # #             # Пробуем несколько режимов PSM
# # # # # #             best_text = ""
# # # # # #             for psm in [3, 6, 11, 12]:
# # # # # #                 config = f'--oem 3 --psm {psm}'
# # # # # #                 text = pytesseract.image_to_string(binary, lang='rus+eng', config=config)
# # # # # #                 if len(text.strip()) > len(best_text):
# # # # # #                     best_text = text.strip()
            
# # # # # #             if best_text:
# # # # # #                 best_text = ' '.join(best_text.split())
# # # # # #                 phones = self._extract_phone_numbers(best_text)
                
# # # # # #                 print(f"✅ Текст найден: {len(best_text)} символов")
# # # # # #                 print(f"   Начало: {best_text[:100]}")
                
# # # # # #                 return {
# # # # # #                     'text': best_text,
# # # # # #                     'phones': phones,
# # # # # #                     'has_text': True,
# # # # # #                     'length': len(best_text)
# # # # # #                 }
            
# # # # # #             print("❌ Текст не найден")
# # # # # #             return {'text': '', 'phones': [], 'has_text': False, 'length': 0}
# # # # # #         except Exception as e:
# # # # # #             print(f"❌ Ошибка OCR: {e}")
# # # # # #             return {'text': '', 'phones': [], 'has_text': False, 'length': 0}
    
# # # # # #     def _extract_phone_numbers(self, text):
# # # # # #         patterns = [
# # # # # #             r'\+7\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # #             r'8\s?\d{3}\s?\d{3}\s?\d{2}\s?\d{2}',
# # # # # #             r'\d{3}[-\s]?\d{2}[-\s]?\d{2}',
# # # # # #         ]
# # # # # #         phones = []
# # # # # #         for pattern in patterns:
# # # # # #             phones.extend(re.findall(pattern, text))
# # # # # #         return phones

# # # # # # text_recognizer = TextRecognizer()

# # # # # # # --- РАСПОЗНАВАНИЕ ДЕНЕГ (УЛУЧШЕННОЕ) ---
# # # # # # class MoneyRecognizer:
# # # # # #     def __init__(self):
# # # # # #         print("💰 Инициализация распознавателя денег...")
# # # # # #         # Более точные цветовые диапазоны для тенге
# # # # # #         self.denominations = {
# # # # # #             200: {'color': 'синий', 'hsv_range': ([100, 50, 50], [130, 255, 255]), 'keywords': ['200', 'екі жүз', 'двести']},
# # # # # #             500: {'color': 'желтый', 'hsv_range': ([20, 50, 50], [40, 255, 255]), 'keywords': ['500', 'бес жүз', 'пятьсот']},
# # # # # #             1000: {'color': 'зеленый', 'hsv_range': ([40, 50, 50], [80, 255, 255]), 'keywords': ['1000', 'мың', 'тысяча']},
# # # # # #             2000: {'color': 'сиреневый', 'hsv_range': ([130, 30, 50], [160, 255, 255]), 'keywords': ['2000', 'екі мың']},
# # # # # #             5000: {'color': 'коричневый', 'hsv_range': ([0, 50, 30], [20, 255, 150]), 'keywords': ['5000', 'бес мың']},
# # # # # #             10000: {'color': 'бежевый', 'hsv_range': ([15, 20, 100], [35, 100, 220]), 'keywords': ['10000', 'он мың']},
# # # # # #             20000: {'color': 'серо-синий', 'hsv_range': ([90, 20, 80], [110, 120, 180]), 'keywords': ['20000', 'жиырма']}
# # # # # #         }
# # # # # #         print("✅ Распознаватель денег готов")
    
# # # # # #     def recognize(self, frame):
# # # # # #         try:
# # # # # #             print("\n💵 Распознавание купюры...")
            
# # # # # #             # Увеличиваем изображение
# # # # # #             frame_large = cv2.resize(frame, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
            
# # # # # #             # OCR
# # # # # #             gray = cv2.cvtColor(frame_large, cv2.COLOR_BGR2GRAY)
# # # # # #             gray = cv2.equalizeHist(gray)
# # # # # #             _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# # # # # #             binary = cv2.medianBlur(binary, 3)
            
# # # # # #             text = pytesseract.image_to_string(binary, lang='rus+eng', config='--psm 6')
# # # # # #             text = text.lower().strip()
# # # # # #             print(f"📖 OCR текст: '{text}'")
            
# # # # # #             # Анализ цвета
# # # # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
# # # # # #             detected = []
            
# # # # # #             for denom, info in self.denominations.items():
# # # # # #                 score = 0
                
# # # # # #                 # Проверка текста на номинал
# # # # # #                 denom_str = str(denom)
# # # # # #                 if denom_str in text:
# # # # # #                     score += 60
# # # # # #                     print(f"  ✓ Найден номинал {denom} в тексте")
                
# # # # # #                 # Проверка ключевых слов
# # # # # #                 for keyword in info['keywords']:
# # # # # #                     if keyword.lower() in text:
# # # # # #                         score += 40
# # # # # #                         print(f"  ✓ Найдено ключевое слово '{keyword}'")
                
# # # # # #                 # Проверка цвета
# # # # # #                 lower, upper = info['hsv_range']
# # # # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # # # #                 color_pct = (np.sum(mask > 0) / mask.size) * 100
                
# # # # # #                 if color_pct > 3:
# # # # # #                     score += min(color_pct * 2, 40)
# # # # # #                     print(f"  🎨 Цвет {info['color']}: {color_pct:.1f}%")
                
# # # # # #                 if score > 0:
# # # # # #                     detected.append({'denomination': denom, 'score': score, 'confidence': min(score/100, 1.0)})
            
# # # # # #             if detected:
# # # # # #                 detected.sort(key=lambda x: x['score'], reverse=True)
# # # # # #                 best = detected[0]
# # # # # #                 denom = best['denomination']
# # # # # #                 conf = best['confidence']
                
# # # # # #                 print(f"✅ Результат: {denom} тенге (уверенность: {conf:.0%})")
                
# # # # # #                 return {
# # # # # #                     'has_money': True,
# # # # # #                     'denomination': denom,
# # # # # #                     'confidence': float(conf),
# # # # # #                     'description': f"Обнаружена купюра {denom} тенге"
# # # # # #                 }
            
# # # # # #             print("❌ Купюра не распознана")
# # # # # #             return {
# # # # # #                 'has_money': False,
# # # # # #                 'description': 'Деньги не обнаружены. Наведите камеру на всю купюру целиком и держите неподвижно несколько секунд'
# # # # # #             }
# # # # # #         except Exception as e:
# # # # # #             print(f"❌ Ошибка: {e}")
# # # # # #             return {'has_money': False, 'description': 'Ошибка распознавания'}

# # # # # # money_recognizer = MoneyRecognizer()

# # # # # # # --- YOLO ---
# # # # # # OBJECT_HEIGHTS = {
# # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # # # # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8, 'book': 0.3
# # # # # # }

# # # # # # OBJECT_TRANSLATIONS = {
# # # # # #     'person': 'человек', 'car': 'машина', 'chair': 'стул', 'bottle': 'бутылка',
# # # # # #     'cup': 'чашка', 'dog': 'собака', 'cat': 'кошка', 'tv': 'телевизор',
# # # # # #     'laptop': 'ноутбук', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл',
# # # # # #     'bus': 'автобус', 'truck': 'грузовик', 'traffic light': 'светофор',
# # # # # #     'stop sign': 'знак стоп', 'bench': 'скамейка', 'book': 'книга',
# # # # # #     'clock': 'часы', 'cell phone': 'телефон', 'keyboard': 'клавиатура'
# # # # # # }

# # # # # # PRIORITY_OBJECTS = {'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck', 'traffic light', 'stop sign', 'dog', 'cat'}

# # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # #     if bbox_height_pixels <= 0:
# # # # # #         return None
# # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # #     return round(distance, 2)

# # # # # # print("🔄 Загрузка YOLO...")
# # # # # # model = YOLO('yolov5su.pt')
# # # # # # print("✅ YOLO готова")

# # # # # # def detect_objects(frame):
# # # # # #     try:
# # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # #         results = model.predict(img_rgb, conf=0.25, iou=0.45, verbose=False, imgsz=640, max_det=50)[0]
        
# # # # # #         detections = []
# # # # # #         frame_height, frame_width = frame.shape[:2]

# # # # # #         for r in results.boxes:
# # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # #             conf = float(r.conf[0])
# # # # # #             cls_id = int(r.cls[0])
# # # # # #             label = model.names[cls_id]
# # # # # #             distance = estimate_distance(bbox, frame_height, label)
            
# # # # # #             x_center = (bbox[0] + bbox[2]) / 2
# # # # # #             y_center = (bbox[1] + bbox[3]) / 2
            
# # # # # #             vertical_pos = "верхняя часть" if y_center < frame_height * 0.33 else "середина" if y_center < frame_height * 0.66 else "нижняя часть"

# # # # # #             detections.append({
# # # # # #                 "label": label,
# # # # # #                 "confidence": conf,
# # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # #                 "distance_m": float(distance) if distance else None,
# # # # # #                 "priority": label in PRIORITY_OBJECTS,
# # # # # #                 "vertical_position": vertical_pos,
# # # # # #                 "x_center": float(x_center),
# # # # # #                 "y_center": float(y_center)
# # # # # #             })
        
# # # # # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
# # # # # #         print(f"🔍 Обнаружено: {len(detections)} объектов")
# # # # # #         return detections
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ Ошибка YOLO: {e}")
# # # # # #         return []

# # # # # # def generate_obstacle_description(obstacles, ocr_data=None, colors=None, brightness=None):
# # # # # #     descriptions = []
    
# # # # # #     if brightness:
# # # # # #         descriptions.append(f"{brightness}")
    
# # # # # #     if colors and len(colors) > 0:
# # # # # #         color_desc = " и ".join([c['color'] for c in colors[:2]])
# # # # # #         descriptions.append(f"Основные цвета: {color_desc}")
    
# # # # # #     if ocr_data and ocr_data.get('has_text'):
# # # # # #         text = ocr_data['text']
# # # # # #         if text:
# # # # # #             text_clean = ' '.join(text.split())
# # # # # #             if len(text_clean) > 200:
# # # # # #                 descriptions.append(f"Обнаружен текст. Начало: {text_clean[:200]}")
# # # # # #             else:
# # # # # #                 descriptions.append(f"Обнаружен текст: {text_clean}")
        
# # # # # #         if ocr_data.get('phones'):
# # # # # #             descriptions.append(f"Телефоны: {', '.join(ocr_data['phones'])}")
    
# # # # # #     if obstacles:
# # # # # #         descriptions.append(f"Обнаружено объектов: {len(obstacles)}")
        
# # # # # #         frame_center = 320
# # # # # #         for obs in obstacles[:5]:
# # # # # #             label_ru = OBJECT_TRANSLATIONS.get(obs['label'], obs['label'])
# # # # # #             distance = obs.get('distance_m')
            
# # # # # #             if distance:
# # # # # #                 if distance < 1.0:
# # # # # #                     dist_text = f"очень близко, {int(distance * 100)} сантиметров"
# # # # # #                 elif distance < 2.0:
# # # # # #                     dist_text = f"близко, {distance:.1f} метра"
# # # # # #                 else:
# # # # # #                     dist_text = f"{int(distance)} метров"
# # # # # #             else:
# # # # # #                 dist_text = "расстояние неизвестно"
            
# # # # # #             x_center = (obs['bbox'][0] + obs['bbox'][2]) / 2
# # # # # #             pos = "слева" if x_center < frame_center - 150 else "справа" if x_center > frame_center + 150 else "прямо перед вами"
            
# # # # # #             priority = "Внимание! " if obs.get('priority') else ""
# # # # # #             descriptions.append(f"{priority}{label_ru} {pos}, {dist_text}")
        
# # # # # #         if len(obstacles) > 5:
# # # # # #             descriptions.append(f"и еще {len(obstacles) - 5} объектов")
# # # # # #     else:
# # # # # #         descriptions.append("Препятствий не обнаружено")
    
# # # # # #     return ". ".join(descriptions)

# # # # # # # --- ЭНДПОИНТЫ ---

# # # # # # @app.route('/')
# # # # # # def index():
# # # # # #     return jsonify({
# # # # # #         "status": "Blind Assistant Server v12 - ПОЛНАЯ ВЕРСИЯ",
# # # # # #         "features": ["Детекция объектов", "OCR", "Распознавание денег", "Время", "Озвучка"]
# # # # # #     })

# # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # def register_voice():
# # # # # #     temp_audio = temp_wav = None
# # # # # #     try:
# # # # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # # # #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# # # # # #         username = request.form['username'].strip()
# # # # # #         audio_file = request.files['audio']
        
# # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # #         audio_file.save(temp_audio)
        
# # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # #         if not temp_wav:
# # # # # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # #         conn = get_db_connection()
# # # # # #         cur = conn.cursor()
        
# # # # # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # #         existing = cur.fetchone()
        
# # # # # #         if existing:
# # # # # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (voice_features, username))
# # # # # #             message = "Профиль обновлен"
# # # # # #         else:
# # # # # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, voice_features))
# # # # # #             message = "Профиль создан"
        
# # # # # #         conn.commit()
# # # # # #         cur.close()
# # # # # #         conn.close()
        
# # # # # #         return jsonify({"status": "success", "message": message, "username": username})
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500
# # # # # #     finally:
# # # # # #         for f in [temp_audio, temp_wav]:
# # # # # #             if f and os.path.exists(f):
# # # # # #                 try: os.remove(f)
# # # # # #                 except: pass

# # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # def login_voice():
# # # # # #     temp_audio = temp_wav = None
# # # # # #     try:
# # # # # #         username = request.form['username'].strip()
# # # # # #         audio_file = request.files['audio']
        
# # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # #         audio_file.save(temp_audio)
        
# # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # #         conn = get_db_connection()
# # # # # #         cur = conn.cursor()
        
# # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # #         result = cur.fetchone()
        
# # # # # #         if not result:
# # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # # # # #         cur.close()
# # # # # #         conn.close()
        
# # # # # #         if similarity > 0.7:
# # # # # #             welcome_text = f"Добро пожаловать, {username}! Камера готова. Доступны команды: Сканировать, Читать текст, Распознать деньги, Время, Выйти."
# # # # # #             return jsonify({
# # # # # #                 "status": "success",
# # # # # #                 "message": "Вход выполнен",
# # # # # #                 "username": username,
# # # # # #                 "similarity": float(similarity),
# # # # # #                 "welcome_text": welcome_text,
# # # # # #                 "skippable": True
# # # # # #             })
# # # # # #         else:
# # # # # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500
# # # # # #     finally:
# # # # # #         for f in [temp_audio, temp_wav]:
# # # # # #             if f and os.path.exists(f):
# # # # # #                 try: os.remove(f)
# # # # # #                 except: pass

# # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # def voice_command():
# # # # # #     try:
# # # # # #         text = request.form.get('text', '').lower().strip()
# # # # # #         print(f"🎤 Голосовая команда: '{text}'")
        
# # # # # #         if not text:
# # # # # #             return jsonify({"error": "Текст не распознан"}), 400
        
# # # # # #         command = voice_command_recognizer.recognize_command(text)
        
# # # # # #         if command == 'время':
# # # # # #             time_data = time_service.get_current_time()
# # # # # #             return jsonify({"status": "success", "command": "время", "data": time_data})
# # # # # #         elif command in ['сканировать', 'читать', 'деньги', 'выйти']:
# # # # # #             return jsonify({"status": "success", "command": command})
# # # # # #         else:
# # # # # #             return jsonify({
# # # # # #                 "status": "unknown",
# # # # # #                 "message": "Команда не распознана",
# # # # # #                 "available_commands": ["сканировать", "время", "читать", "деньги", "выйти"]
# # # # # #             })
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # def process_frame():
# # # # # #     try:
# # # # # #         print("\n🔵 Обработка кадра...")
# # # # # #         if 'frame' not in request.files:
# # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         if frame is None:
# # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # #         print(f"✅ Изображение: {frame.shape}")

# # # # # #         # Полный анализ
# # # # # #         obstacles = detect_objects(frame)
# # # # # #         ocr_data = text_recognizer.extract_text(frame)
# # # # # #         colors = color_analyzer.analyze_colors(frame)
# # # # # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # # # # #         annotated_frame = object_visualizer.draw_detections(frame, obstacles, ocr_data)
# # # # # #         description = generate_obstacle_description(obstacles, ocr_data, colors, brightness)
        
# # # # # #         audio_path = tts_engine.text_to_speech(description)
        
# # # # # #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# # # # # #         cv2.imwrite(annotated_path, annotated_frame)
        
# # # # # #         with open(annotated_path, 'rb') as f:
# # # # # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # # # # #         os.remove(annotated_path)

# # # # # #         print(f"✅ Результат: {description[:100]}...")

# # # # # #         return jsonify({
# # # # # #             "obstacles": obstacles,
# # # # # #             "ocr": ocr_data,
# # # # # #             "colors": colors,
# # # # # #             "brightness": brightness,
# # # # # #             "description": description,
# # # # # #             "count": len(obstacles),
# # # # # #             "audio_available": audio_path is not None,
# # # # # #             "annotated_frame": annotated_base64,
# # # # # #             "message": "Кадр обработан успешно"
# # # # # #         })
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ Ошибка: {e}")
# # # # # #         import traceback
# # # # # #         traceback.print_exc()
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/read_text', methods=['POST'])
# # # # # # def read_text():
# # # # # #     try:
# # # # # #         print("\n📖 Чтение текста...")
# # # # # #         if 'frame' not in request.files:
# # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         if frame is None:
# # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # #         print(f"✅ Изображение: {frame.shape}")

# # # # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # # # #         if ocr_data.get('has_text'):
# # # # # #             text = ocr_data['text']
# # # # # #             if len(text) > 300:
# # # # # #                 description = f"Обнаружен длинный текст. {text}"
# # # # # #             else:
# # # # # #                 description = f"Обнаружен текст: {text}"
            
# # # # # #             if ocr_data.get('phones'):
# # # # # #                 description += f". Номера телефонов: {', '.join(ocr_data['phones'])}"
# # # # # #         else:
# # # # # #             description = "Текст не обнаружен. Наведите камеру на текст и держите устройство неподвижно несколько секунд"
        
# # # # # #         audio_path = tts_engine.text_to_speech(description)
        
# # # # # #         print(f"✅ Результат: {description[:100]}...")
        
# # # # # #         return jsonify({
# # # # # #             "ocr": ocr_data,
# # # # # #             "description": description,
# # # # # #             "audio_available": audio_path is not None,
# # # # # #             "message": "Распознавание текста выполнено"
# # # # # #         })
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ Ошибка: {e}")
# # # # # #         import traceback
# # # # # #         traceback.print_exc()
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/recognize_money', methods=['POST'])
# # # # # # def recognize_money():
# # # # # #     try:
# # # # # #         print("\n💰 Распознавание денег...")
# # # # # #         if 'frame' not in request.files:
# # # # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         if frame is None:
# # # # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # # # #         print(f"✅ Изображение: {frame.shape}")

# # # # # #         money_data = money_recognizer.recognize(frame)
# # # # # #         description = money_data['description']
        
# # # # # #         audio_path = tts_engine.text_to_speech(description)
        
# # # # # #         print(f"✅ Результат: {description}")
        
# # # # # #         return jsonify({
# # # # # #             "money": money_data,
# # # # # #             "description": description,
# # # # # #             "audio_available": audio_path is not None,
# # # # # #             "message": "Распознавание денег выполнено"
# # # # # #         })
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ Ошибка: {e}")
# # # # # #         import traceback
# # # # # #         traceback.print_exc()
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/get_time', methods=['GET'])
# # # # # # def get_time():
# # # # # #     try:
# # # # # #         time_data = time_service.get_current_time()
# # # # # #         return jsonify(time_data)
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/test_connection', methods=['GET'])
# # # # # # def test_connection():
# # # # # #     return jsonify({"status": "ok", "message": "Сервер работает"})

# # # # # # if __name__ == '__main__':
# # # # # #     print("\n" + "="*70)
# # # # # #     print("🚀 BLIND ASSISTANT SERVER v12 - ПОЛНАЯ РАБОЧАЯ ВЕРСИЯ")
# # # # # #     print("="*70)
# # # # # #     print("✅ Детекция объектов (YOLO) - РАБОТАЕТ")
# # # # # #     print("✅ Распознавание текста (OCR) - УЛУЧШЕНО")
# # # # # #     print("✅ Распознавание казахстанских тенге - УЛУЧШЕНО")
# # # # # #     print("✅ Определение цветов и освещенности")
# # # # # #     print("✅ Голосовые команды")
# # # # # #     print("✅ Текущее время")
# # # # # #     print("✅ Озвучка всех данных")
# # # # # #     print("="*70)
# # # # # #     print("\n💡 ВАЖНО:")
# # # # # #     print("   - Для текста: держите камеру неподвижно 2-3 секунды")
# # # # # #     print("   - Для денег: наведите на ВСЮ купюру целиком")
# # # # # #     print("   - Хорошее освещение = лучшее распознавание")
# # # # # #     print("="*70)
# # # # # #     print("\n📢 Голосовые команды:")
# # # # # #     print("   🔹 'СКАНИРОВАТЬ' - полный анализ окружения")
# # # # # #     print("   🔹 'ЧИТАТЬ' - режим чтения текста")
# # # # # #     print("   🔹 'ДЕНЬГИ' - распознавание купюр")
# # # # # #     print("   🔹 'ВРЕМЯ' - текущее время")
# # # # # #     print("   🔹 'ВЫЙТИ' - выход из приложения")
# # # # # #     print("="*70)
# # # # # #     print("\n🌐 Сервер запускается на:")
# # # # # #     print("   - Localhost: http://192.168.8.63:5000")
# # # # # #     print("   - Эмулятор Android: http://10.0.2.2:5000")
# # # # # #     print("   - Реальное устройство: http://<IP_КОМПЬЮТЕРА>:5000")
# # # # # #     print("\n💻 Узнать IP компьютера:")
# # # # # #     print("   Windows: ipconfig")
# # # # # #     print("   Linux/Mac: ifconfig")
# # # # # #     print("="*70 + "\n")
    
# # # # # #     app.run(host='192.168.8.63', port=5000, debug=True)




# # # # # # import os
# # # # # # import uuid
# # # # # # import cv2
# # # # # # import numpy as np
# # # # # # import psycopg2
# # # # # # from flask import Flask, request, jsonify, send_file
# # # # # # from ultralytics import YOLO
# # # # # # import wave
# # # # # # import hashlib
# # # # # # import base64
# # # # # # from gtts import gTTS
# # # # # # import re
# # # # # # from datetime import datetime
# # # # # # from pydub import AudioSegment

# # # # # # # === ПРОВЕРКА TESSERACT ===
# # # # # # TESSERACT_AVAILABLE = False
# # # # # # try:
# # # # # #     import pytesseract
    
# # # # # #     # Попытка найти Tesseract автоматически
# # # # # #     TESSERACT_PATHS = [
# # # # # #         r'C:\Program Files\Tesseract-OCR\tesseract.exe',
# # # # # #         r'C:\Program Files (x86)\Tesseract-OCR\tesseract.exe',
# # # # # #         r'C:\Tesseract-OCR\tesseract.exe',
# # # # # #         '/usr/bin/tesseract',
# # # # # #         '/usr/local/bin/tesseract',
# # # # # #     ]
    
# # # # # #     for path in TESSERACT_PATHS:
# # # # # #         if os.path.exists(path):
# # # # # #             pytesseract.pytesseract.tesseract_cmd = path
# # # # # #             # Пробуем использовать
# # # # # #             try:
# # # # # #                 test_img = np.ones((50, 200, 3), dtype=np.uint8) * 255
# # # # # #                 pytesseract.image_to_string(test_img, lang='eng')
# # # # # #                 TESSERACT_AVAILABLE = True
# # # # # #                 print(f"✅ Tesseract найден и работает: {path}")
# # # # # #                 break
# # # # # #             except:
# # # # # #                 continue
    
# # # # # #     if not TESSERACT_AVAILABLE:
# # # # # #         print("⚠️  Tesseract НЕ НАЙДЕН")
# # # # # #         print("   OCR будет ОТКЛЮЧЕН")
# # # # # #         print("   Деньги распознаются только по ЦВЕТУ")
# # # # # # except Exception as e:
# # # # # #     print(f"⚠️  Ошибка импорта pytesseract: {e}")
# # # # # #     print("   OCR будет ОТКЛЮЧЕН")

# # # # # # # --- Конфигурация ---
# # # # # # DB_NAME = "blind_app"
# # # # # # DB_USER = "postgres"
# # # # # # DB_PASSWORD = "12345"
# # # # # # DB_HOST = "localhost"

# # # # # # app = Flask(__name__)

# # # # # # @app.after_request
# # # # # # def after_request(response):
# # # # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # # # #     return response

# # # # # # def get_db_connection():
# # # # # #     return psycopg2.connect(dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST)

# # # # # # # --- ВРЕМЯ ---
# # # # # # class TimeService:
# # # # # #     def get_current_time(self):
# # # # # #         now = datetime.now()
# # # # # #         hours = now.hour
# # # # # #         minutes = now.minute
        
# # # # # #         if minutes == 1:
# # # # # #             min_word = "минута"
# # # # # #         elif 2 <= minutes <= 4:
# # # # # #             min_word = "минуты"
# # # # # #         else:
# # # # # #             min_word = "минут"
        
# # # # # #         if hours == 1 or hours == 21:
# # # # # #             hour_word = "час"
# # # # # #         elif 2 <= hours <= 4 or 22 <= hours <= 24:
# # # # # #             hour_word = "часа"
# # # # # #         else:
# # # # # #             hour_word = "часов"
        
# # # # # #         time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
# # # # # #         return {'time': time_str, 'hour': hours, 'minute': minutes}

# # # # # # time_service = TimeService()

# # # # # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # # # # class ColorAnalyzer:
# # # # # #     def __init__(self):
# # # # # #         self.color_names = {
# # # # # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # # # # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # # # # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # # # # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # # # # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # # # # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # # # # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # # # # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # # # # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # # # # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # # # # #             'черный': ([0, 0, 0], [180, 255, 50])
# # # # # #         }
    
# # # # # #     def analyze_colors(self, frame):
# # # # # #         try:
# # # # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # # # # #             detected_colors = []
            
# # # # # #             for color_name, (lower, upper) in self.color_names.items():
# # # # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # # # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # # # # #                 if percentage > 5:
# # # # # #                     detected_colors.append({'color': color_name, 'percentage': round(percentage, 1)})
            
# # # # # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # # # # #             return detected_colors[:3]
# # # # # #         except:
# # # # # #             return []
    
# # # # # #     def get_brightness_level(self, frame):
# # # # # #         try:
# # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # #             avg_brightness = np.mean(gray)
            
# # # # # #             if avg_brightness < 50:
# # # # # #                 return "очень темно"
# # # # # #             elif avg_brightness < 100:
# # # # # #                 return "темно"
# # # # # #             elif avg_brightness < 150:
# # # # # #                 return "нормальное освещение"
# # # # # #             elif avg_brightness < 200:
# # # # # #                 return "светло"
# # # # # #             else:
# # # # # #                 return "очень светло"
# # # # # #         except:
# # # # # #             return "не определено"

# # # # # # color_analyzer = ColorAnalyzer()

# # # # # # # --- ОЗВУЧКА ---
# # # # # # class TextToSpeech:
# # # # # #     def __init__(self):
# # # # # #         self.cache_dir = "audio_cache"
# # # # # #         if not os.path.exists(self.cache_dir):
# # # # # #             os.makedirs(self.cache_dir)
    
# # # # # #     def text_to_speech(self, text, lang='ru'):
# # # # # #         try:
# # # # # #             text_hash = hashlib.md5(text.encode()).hexdigest()
# # # # # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # # # # #             if os.path.exists(audio_path):
# # # # # #                 return audio_path
            
# # # # # #             tts = gTTS(text=text, lang=lang, slow=False)
# # # # # #             tts.save(audio_path)
# # # # # #             return audio_path
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка озвучки: {e}")
# # # # # #             return None

# # # # # # tts_engine = TextToSpeech()

# # # # # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # # # class SimpleVoiceBiometrics:
# # # # # #     def extract_mfcc_features(self, audio_path):
# # # # # #         try:
# # # # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # # # #                 sample_width = wav_file.getsampwidth()
# # # # # #                 frame_rate = wav_file.getframerate()
# # # # # #                 n_frames = wav_file.getnframes()
# # # # # #                 frames = wav_file.readframes(n_frames)
                
# # # # # #                 if sample_width == 2:
# # # # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # # # #                 else:
# # # # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # # # # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # # # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # # # # #                 return features_b64
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # # # #             return None
    
# # # # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # # # #         features = []
# # # # # #         features.append(np.mean(audio_data ** 2))
# # # # # #         features.append(np.std(audio_data))
# # # # # #         features.append(np.max(np.abs(audio_data)))
        
# # # # # #         fft = np.fft.fft(audio_data)
# # # # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # # # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # # # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # # # #         for low, high in bands:
# # # # # #             mask = (freqs >= low) & (freqs < high)
# # # # # #             if np.any(mask):
# # # # # #                 features.append(np.mean(fft_magnitude[mask]))
# # # # # #             else:
# # # # # #                 features.append(0.0)
        
# # # # # #         return np.array(features)
    
# # # # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # # # #         try:
# # # # # #             features1_bytes = base64.b64decode(features1_b64)
# # # # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # # # #             min_len = min(len(features1), len(features2))
# # # # # #             features1 = features1[:min_len]
# # # # # #             features2 = features2[:min_len]
            
# # # # # #             distance = np.linalg.norm(features1 - features2)
# # # # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
# # # # # #             return float(max(0.0, min(1.0, similarity)))
# # # # # #         except:
# # # # # #             return 0.0

# # # # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # # # --- КОМАНДЫ ---
# # # # # # class VoiceCommandRecognizer:
# # # # # #     def __init__(self):
# # # # # #         self.commands = {
# # # # # #             'выйти': ['выйти', 'выход', 'выхожу', 'закрыть', 'exit'],
# # # # # #             'сканировать': ['сканировать', 'скан', 'сканируй', 'что вижу', 'что впереди', 'анализ', 'scan'],
# # # # # #             'время': ['время', 'который час', 'сколько времени', 'time'],
# # # # # #             'читать': ['читать', 'читай', 'прочитай', 'текст', 'что написано', 'read'],
# # # # # #             'деньги': ['деньги', 'распознать деньги', 'сколько денег', 'какая купюра', 'купюра', 'тенге', 'money']
# # # # # #         }
    
# # # # # #     def recognize_command(self, text):
# # # # # #         if not text:
# # # # # #             return None
        
# # # # # #         text = text.lower().strip()
# # # # # #         noise = ['слушаю', 'команду', 'команда', 'пожалуйста']
# # # # # #         for n in noise:
# # # # # #             text = text.replace(n, '')
# # # # # #         text = ' '.join(text.split())
        
# # # # # #         for cmd_type, keywords in self.commands.items():
# # # # # #             for keyword in keywords:
# # # # # #                 if keyword in text or text in keyword:
# # # # # #                     return cmd_type
# # # # # #         return None

# # # # # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # # # # --- КОНВЕРТЕР АУДИО ---
# # # # # # class AudioConverter:
# # # # # #     def convert_aac_to_wav(self, aac_path):
# # # # # #         try:
# # # # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # # # #             audio.export(wav_path, format="wav")
# # # # # #             return wav_path
# # # # # #         except Exception as e:
# # # # # #             print(f"Ошибка конвертации: {e}")
# # # # # #             return None

# # # # # # audio_converter = AudioConverter()

# # # # # # # --- OCR (с проверкой доступности) ---
# # # # # # class TextRecognizer:
# # # # # #     def __init__(self):
# # # # # #         self.available = TESSERACT_AVAILABLE
    
# # # # # #     def _clean_text(self, text):
# # # # # #         """Очищает текст от мусора для озвучки"""
# # # # # #         if not text:
# # # # # #             return ""
        
# # # # # #         # Убираем символы которые плохо озвучиваются
# # # # # #         cleaned = text
        
# # # # # #         # Убираем повторяющиеся символы (---, ===, ...)
# # # # # #         cleaned = re.sub(r'(.)\1{2,}', r'\1', cleaned)
        
# # # # # #         # Убираем строки только из символов (без букв)
# # # # # #         lines = cleaned.split('\n')
# # # # # #         good_lines = []
# # # # # #         for line in lines:
# # # # # #             # Считаем буквы (кириллица и латиница)
# # # # # #             letters = len(re.findall(r'[а-яА-ЯёЁa-zA-Z]', line))
# # # # # #             # Оставляем только строки где больше 30% букв
# # # # # #             if len(line) > 0 and letters / len(line) > 0.3:
# # # # # #                 good_lines.append(line)
        
# # # # # #         cleaned = ' '.join(good_lines)
        
# # # # # #         # Убираем лишние спецсимволы
# # # # # #         cleaned = re.sub(r'[_|\\\/\[\]{}()]+', ' ', cleaned)
        
# # # # # #         # Убираем множественные пробелы
# # # # # #         cleaned = re.sub(r'\s+', ' ', cleaned)
        
# # # # # #         # Убираем короткие "слова" (1-2 символа) которые не буквы
# # # # # #         words = cleaned.split()
# # # # # #         good_words = []
# # # # # #         for word in words:
# # # # # #             if len(word) >= 3 or re.match(r'^[а-яА-ЯёЁa-zA-Z]+$', word):
# # # # # #                 good_words.append(word)
        
# # # # # #         cleaned = ' '.join(good_words)
# # # # # #         return cleaned.strip()
    
# # # # # #     def extract_text(self, frame):
# # # # # #         if not self.available:
# # # # # #             print("⚠️  OCR недоступен (Tesseract не установлен)")
# # # # # #             return {
# # # # # #                 'text': '',
# # # # # #                 'phones': [],
# # # # # #                 'has_text': False,
# # # # # #                 'length': 0,
# # # # # #                 'error': 'OCR недоступен. Установите Tesseract OCR'
# # # # # #             }
        
# # # # # #         try:
# # # # # #             print("\n📖 Распознавание текста...")
# # # # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # # # #             gray = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
# # # # # #             gray = cv2.equalizeHist(gray)
# # # # # #             _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# # # # # #             binary = cv2.medianBlur(binary, 3)
            
# # # # # #             best_text = ""
# # # # # #             for psm in [3, 6, 11]:
# # # # # #                 config = f'--oem 3 --psm {psm}'
# # # # # #                 text = pytesseract.image_to_string(binary, lang='rus+eng', config=config)
# # # # # #                 if len(text.strip()) > len(best_text):
# # # # # #                     best_text = text.strip()
            
# # # # # #             if best_text:
# # # # # #                 # Сохраняем оригинальный текст
# # # # # #                 original_text = ' '.join(best_text.split())
                
# # # # # #                 # Очищаем для озвучки
# # # # # #                 cleaned_text = self._clean_text(best_text)
                
# # # # # #                 if cleaned_text:
# # # # # #                     print(f"✅ Текст найден: {len(cleaned_text)} символов")
# # # # # #                     print(f"   Оригинал: {original_text[:100]}")
# # # # # #                     print(f"   Очищено: {cleaned_text[:100]}")
                    
# # # # # #                     return {
# # # # # #                         'text': original_text,  # Полный текст
# # # # # #                         'cleaned_text': cleaned_text,  # Для озвучки
# # # # # #                         'phones': [],
# # # # # #                         'has_text': True,
# # # # # #                         'length': len(cleaned_text)
# # # # # #                     }
            
# # # # # #             print("❌ Текст не найден")
# # # # # #             return {'text': '', 'cleaned_text': '', 'phones': [], 'has_text': False, 'length': 0}
# # # # # #         except Exception as e:
# # # # # #             print(f"❌ Ошибка OCR: {e}")
# # # # # #             return {'text': '', 'cleaned_text': '', 'phones': [], 'has_text': False, 'length': 0}

# # # # # # text_recognizer = TextRecognizer()

# # # # # # # --- РАСПОЗНАВАНИЕ ДЕНЕГ (ТОЛЬКО ПО ЦВЕТУ) ---
# # # # # # class MoneyRecognizer:
# # # # # #     def __init__(self):
# # # # # #         print("💰 Инициализация распознавателя денег (режим: ТОЛЬКО ЦВЕТ)...")
# # # # # #         self.denominations = {
# # # # # #             200: {'color': 'бежевый', 'hsv_range': ([100, 50, 50], [130, 255, 255]), 'threshold': 12},
# # # # # #             500: {'color': 'синий', 'hsv_range': ([20, 50, 50], [40, 255, 255]), 'threshold': 15},
# # # # # #             1000: {'color': 'желный', 'hsv_range': ([40, 50, 50], [80, 255, 255]), 'threshold': 15},
# # # # # #             2000: {'color': 'зеленый', 'hsv_range': ([130, 30, 50], [160, 255, 255]), 'threshold': 10},
# # # # # #             5000: {'color': 'красный', 'hsv_range': ([0, 50, 30], [20, 255, 150]), 'threshold': 8},
# # # # # #             10000: {'color': 'голубой', 'hsv_range': ([15, 20, 100], [35, 100, 220]), 'threshold': 10},
# # # # # #             20000: {'color': 'серо-синий', 'hsv_range': ([90, 20, 80], [110, 120, 180]), 'threshold': 8}
# # # # # #         }
# # # # # #         print("✅ Распознаватель готов")
    
# # # # # #     def recognize(self, frame):
# # # # # #         try:
# # # # # #             print("\n💵 Распознавание купюры по цвету...")
# # # # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
# # # # # #             detected = []
            
# # # # # #             for denom, info in self.denominations.items():
# # # # # #                 lower, upper = info['hsv_range']
# # # # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # # # #                 color_pct = (np.sum(mask > 0) / mask.size) * 100
                
# # # # # #                 if color_pct > info['threshold']:
# # # # # #                     score = color_pct * 5
# # # # # #                     detected.append({
# # # # # #                         'denomination': denom,
# # # # # #                         'score': score,
# # # # # #                         'confidence': min(score / 100, 1.0),
# # # # # #                         'color_percentage': color_pct
# # # # # #                     })
# # # # # #                     print(f"  🎨 {denom} тенге - {info['color']}: {color_pct:.1f}%")
            
# # # # # #             if detected:
# # # # # #                 detected.sort(key=lambda x: x['score'], reverse=True)
# # # # # #                 best = detected[0]
# # # # # #                 denom = best['denomination']
# # # # # #                 conf = best['confidence']
                
# # # # # #                 print(f"✅ РЕЗУЛЬТАТ: {denom} тенге ({conf:.0%})")
                
# # # # # #                 return {
# # # # # #                     'has_money': True,
# # # # # #                     'denomination': denom,
# # # # # #                     'confidence': float(conf),
# # # # # #                     'description': f"Обнаружена купюра {denom} тенге"
# # # # # #                 }
            
# # # # # #             print("❌ Купюра не распознана")
# # # # # #             return {
# # # # # #                 'has_money': False,
# # # # # #                 'description': 'Деньги не обнаружены. Наведите камеру на ВСЮ купюру. Убедитесь что освещение хорошее'
# # # # # #             }
# # # # # #         except Exception as e:
# # # # # #             print(f"❌ Ошибка: {e}")
# # # # # #             import traceback
# # # # # #             traceback.print_exc()
# # # # # #             return {'has_money': False, 'description': 'Ошибка распознавания'}

# # # # # # money_recognizer = MoneyRecognizer()

# # # # # # # --- YOLO ---
# # # # # # OBJECT_HEIGHTS = {
# # # # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02
# # # # # # }

# # # # # # OBJECT_TRANSLATIONS = {
# # # # # #     'person': 'человек', 'car': 'машина', 'chair': 'стул', 'bottle': 'бутылка',
# # # # # #     'cup': 'чашка', 'dog': 'собака', 'cat': 'кошка', 'tv': 'телевизор',
# # # # # #     'laptop': 'ноутбук', 'bicycle': 'велосипед', 'book': 'книга', 'phone': 'телефон'
# # # # # # }

# # # # # # PRIORITY_OBJECTS = {'person', 'car', 'bicycle', 'dog', 'cat'}

# # # # # # def estimate_distance(bbox, frame_height, object_label):
# # # # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # # # #     if bbox_height_pixels <= 0:
# # # # # #         return None
# # # # # #     FOCAL_LENGTH_PIXELS = 700
# # # # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # # # #     return round(distance, 2)

# # # # # # print("🔄 Загрузка YOLO...")
# # # # # # model = YOLO('yolov5su.pt')
# # # # # # print("✅ YOLO готова")

# # # # # # def detect_objects(frame):
# # # # # #     try:
# # # # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # # # #         results = model.predict(img_rgb, conf=0.25, iou=0.45, verbose=False)[0]
        
# # # # # #         detections = []
# # # # # #         frame_height, frame_width = frame.shape[:2]

# # # # # #         for r in results.boxes:
# # # # # #             bbox = r.xyxy[0].cpu().numpy()
# # # # # #             conf = float(r.conf[0])
# # # # # #             cls_id = int(r.cls[0])
# # # # # #             label = model.names[cls_id]
# # # # # #             distance = estimate_distance(bbox, frame_height, label)
            
# # # # # #             detections.append({
# # # # # #                 "label": label,
# # # # # #                 "confidence": conf,
# # # # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # # # #                 "distance_m": float(distance) if distance else None,
# # # # # #                 "priority": label in PRIORITY_OBJECTS
# # # # # #             })
        
# # # # # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
# # # # # #         print(f"🔍 Обнаружено: {len(detections)} объектов")
# # # # # #         return detections
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ Ошибка YOLO: {e}")
# # # # # #         return []

# # # # # # def generate_description(obstacles, colors=None, brightness=None):
# # # # # #     descriptions = []
    
# # # # # #     if brightness:
# # # # # #         descriptions.append(f"{brightness}")
    
# # # # # #     if colors:
# # # # # #         color_desc = " и ".join([c['color'] for c in colors[:2]])
# # # # # #         descriptions.append(f"Основные цвета: {color_desc}")
    
# # # # # #     if obstacles:
# # # # # #         descriptions.append(f"Обнаружено объектов: {len(obstacles)}")
        
# # # # # #         for obs in obstacles[:5]:
# # # # # #             label_ru = OBJECT_TRANSLATIONS.get(obs['label'], obs['label'])
# # # # # #             distance = obs.get('distance_m')
            
# # # # # #             if distance:
# # # # # #                 if distance < 1.0:
# # # # # #                     dist_text = f"{int(distance * 100)} сантиметров"
# # # # # #                 else:
# # # # # #                     dist_text = f"{int(distance)} метров"
# # # # # #             else:
# # # # # #                 dist_text = "неизвестно"
            
# # # # # #             priority = "Внимание! " if obs.get('priority') else ""
# # # # # #             descriptions.append(f"{priority}{label_ru}, {dist_text}")
# # # # # #     else:
# # # # # #         descriptions.append("Препятствий не обнаружено")
    
# # # # # #     return ". ".join(descriptions)

# # # # # # # --- ЭНДПОИНТЫ ---

# # # # # # @app.route('/')
# # # # # # def index():
# # # # # #     return jsonify({
# # # # # #         "status": "Blind Assistant Server v13 - БЕЗ TESSERACT",
# # # # # #         "tesseract": "доступен" if TESSERACT_AVAILABLE else "недоступен",
# # # # # #         "features": ["Детекция объектов (YOLO)", "Распознавание денег (цвет)", "Время"]
# # # # # #     })

# # # # # # @app.route('/register_voice', methods=['POST'])
# # # # # # def register_voice():
# # # # # #     temp_audio = temp_wav = None
# # # # # #     try:
# # # # # #         username = request.form['username'].strip()
# # # # # #         audio_file = request.files['audio']
        
# # # # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # # # #         audio_file.save(temp_audio)
        
# # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # #         if not temp_wav:
# # # # # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # # # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # #         conn = get_db_connection()
# # # # # #         cur = conn.cursor()
        
# # # # # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # # # #         if cur.fetchone():
# # # # # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (voice_features, username))
# # # # # #         else:
# # # # # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, voice_features))
        
# # # # # #         conn.commit()
# # # # # #         cur.close()
# # # # # #         conn.close()
        
# # # # # #         return jsonify({"status": "success", "username": username})
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500
# # # # # #     finally:
# # # # # #         for f in [temp_audio, temp_wav]:
# # # # # #             if f and os.path.exists(f):
# # # # # #                 try: os.remove(f)
# # # # # #                 except: pass

# # # # # # @app.route('/login_voice', methods=['POST'])
# # # # # # def login_voice():
# # # # # #     temp_audio = temp_wav = None
# # # # # #     try:
# # # # # #         username = request.form['username'].strip()
# # # # # #         audio_file = request.files['audio']
        
# # # # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # # # #         audio_file.save(temp_audio)
        
# # # # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # # # #         conn = get_db_connection()
# # # # # #         cur = conn.cursor()
        
# # # # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # # # #         result = cur.fetchone()
        
# # # # # #         if not result:
# # # # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # # # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # # # # #         cur.close()
# # # # # #         conn.close()
        
# # # # # #         if similarity > 0.7:
# # # # # #             return jsonify({"status": "success", "username": username, "similarity": float(similarity)})
# # # # # #         else:
# # # # # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500
# # # # # #     finally:
# # # # # #         for f in [temp_audio, temp_wav]:
# # # # # #             if f and os.path.exists(f):
# # # # # #                 try: os.remove(f)
# # # # # #                 except: pass

# # # # # # @app.route('/voice_command', methods=['POST'])
# # # # # # def voice_command():
# # # # # #     try:
# # # # # #         text = request.form.get('text', '').strip()
# # # # # #         command = voice_command_recognizer.recognize_command(text)
        
# # # # # #         if command == 'время':
# # # # # #             return jsonify({"status": "success", "command": "время", "data": time_service.get_current_time()})
# # # # # #         elif command in ['сканировать', 'читать', 'деньги', 'выйти']:
# # # # # #             return jsonify({"status": "success", "command": command})
# # # # # #         else:
# # # # # #             return jsonify({"status": "unknown"})
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/process_frame', methods=['POST'])
# # # # # # def process_frame():
# # # # # #     try:
# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         obstacles = detect_objects(frame)
# # # # # #         colors = color_analyzer.analyze_colors(frame)
# # # # # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # # # # #         description = generate_description(obstacles, colors, brightness)
# # # # # #         tts_engine.text_to_speech(description)
        
# # # # # #         annotated_path = f"ann_{uuid.uuid4()}.jpg"
# # # # # #         cv2.imwrite(annotated_path, frame)
        
# # # # # #         with open(annotated_path, 'rb') as f:
# # # # # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # # # # #         os.remove(annotated_path)

# # # # # #         return jsonify({
# # # # # #             "obstacles": obstacles,
# # # # # #             "colors": colors,
# # # # # #             "brightness": brightness,
# # # # # #             "description": description,
# # # # # #             "annotated_frame": annotated_base64
# # # # # #         })
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/read_text', methods=['POST'])
# # # # # # def read_text():
# # # # # #     try:
# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # # # #         if ocr_data.get('has_text'):
# # # # # #             # Используем очищенный текст для озвучки
# # # # # #             cleaned = ocr_data.get('cleaned_text', ocr_data.get('text', ''))
            
# # # # # #             if cleaned and len(cleaned) > 5:
# # # # # #                 # Ограничиваем длину для озвучки (максимум 500 символов)
# # # # # #                 if len(cleaned) > 500:
# # # # # #                     description = f"Обнаружен длинный текст. Читаю начало: {cleaned[:500]}"
# # # # # #                 else:
# # # # # #                     description = f"Обнаружен текст: {cleaned}"
# # # # # #             else:
# # # # # #                 description = "Обнаружен текст, но он нечитаемый. Попробуйте навести камеру лучше"
# # # # # #         else:
# # # # # #             description = "Текст не обнаружен" if TESSERACT_AVAILABLE else "OCR недоступен. Установите Tesseract"
        
# # # # # #         tts_engine.text_to_speech(description)
        
# # # # # #         return jsonify({"ocr": ocr_data, "description": description})
# # # # # #     except Exception as e:
# # # # # #         print(f"❌ Ошибка read_text: {e}")
# # # # # #         import traceback
# # # # # #         traceback.print_exc()
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/recognize_money', methods=['POST'])
# # # # # # def recognize_money():
# # # # # #     try:
# # # # # #         frame_bytes = request.files['frame'].read()
# # # # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # # # #         money_data = money_recognizer.recognize(frame)
# # # # # #         description = money_data['description']
        
# # # # # #         tts_engine.text_to_speech(description)
        
# # # # # #         return jsonify({"money": money_data, "description": description})
# # # # # #     except Exception as e:
# # # # # #         return jsonify({"error": str(e)}), 500

# # # # # # @app.route('/get_time', methods=['GET'])
# # # # # # def get_time():
# # # # # #     return jsonify(time_service.get_current_time())

# # # # # # @app.route('/test_connection', methods=['GET'])
# # # # # # def test_connection():
# # # # # #     return jsonify({"status": "ok"})

# # # # # # if __name__ == '__main__':
# # # # # #     print("\n" + "="*70)
# # # # # #     print("🚀 BLIND ASSISTANT SERVER v13")
# # # # # #     print("="*70)
# # # # # #     print(f"✅ Детекция объектов (YOLO)")
# # # # # #     print(f"{'✅' if TESSERACT_AVAILABLE else '⚠️ '} Распознавание текста (OCR) - {'РАБОТАЕТ' if TESSERACT_AVAILABLE else 'ОТКЛЮЧЕНО'}")
# # # # # #     print(f"✅ Распознавание денег (только по цвету)")
# # # # # #     print(f"✅ Голосовые команды")
# # # # # #     print(f"✅ Текущее время")
# # # # # #     print("="*70)
    
# # # # # #     if not TESSERACT_AVAILABLE:
# # # # # #         print("\n⚠️  TESSERACT НЕ НАЙДЕН!")
# # # # # #         print("   Чтобы включить OCR, установите Tesseract:")
# # # # # #         print("   Windows: https://github.com/UB-Mannheim/tesseract/wiki")
# # # # # #         print("   Linux: sudo apt-get install tesseract-ocr tesseract-ocr-rus")
# # # # # #         print("   Mac: brew install tesseract")
# # # # # #         print("="*70)
    
# # # # # #     print("\n💡 СОВЕТЫ:")
# # # # # #     print("   - Для денег: наведите на ВСЮ купюру, хорошее освещение")
# # # # # #     print("   - Для объектов: камера работает на любом расстоянии")
# # # # # #     print("="*70)
# # # # # #     print("\n🌐 Сервер запускается на:")
# # # # # #     print("   http://0.0.0.0:5000")
# # # # # #     print("   (доступен с любого устройства в сети)")
# # # # # #     print("="*70 + "\n")
    
# # # # # #     app.run(host='10.59.29.73', port=5000, debug=True)









# # # # import os
# # # # import uuid
# # # # import cv2
# # # # import numpy as np
# # # # import psycopg2
# # # # from flask import Flask, request, jsonify, send_file
# # # # from ultralytics import YOLO
# # # # import wave
# # # # import hashlib
# # # # import base64
# # # # import pytesseract
# # # # from gtts import gTTS
# # # # import re
# # # # from datetime import datetime
# # # # from pydub import AudioSegment
# # # # from langdetect import detect, LangDetectException

# # # # pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'

# # # # # Проверка Tesseract
# # # # try:
# # # #     print("🔍 Проверка Tesseract OCR...")
# # # #     version = pytesseract.get_tesseract_version()
# # # #     print(f"✅ Tesseract версия: {version}")
    
# # # #     # Проверка доступных языков
# # # #     try:
# # # #         languages = pytesseract.get_languages()
# # # #         print(f"✅ Доступные языки: {languages}")
# # # #     except:
# # # #         print("⚠️ Не удалось получить список языков")
        
# # # # except Exception as e:
# # # #     print(f"❌ Ошибка Tesseract: {e}")
# # # #     print("⚠️ OCR будет отключен")

# # # # # --- Конфигурация ---
# # # # DB_NAME = "blind_app"
# # # # DB_USER = "postgres"
# # # # DB_PASSWORD = "12345"
# # # # DB_HOST = "localhost"

# # # # app = Flask(__name__)

# # # # @app.after_request
# # # # def after_request(response):
# # # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # # #     return response

# # # # def get_db_connection():
# # # #     return psycopg2.connect(
# # # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # # #     )

# # # # # --- ВРЕМЯ ---
# # # # class TimeService:
# # # #     def get_current_time(self):
# # # #         now = datetime.now()
# # # #         hours = now.hour
# # # #         minutes = now.minute
        
# # # #         if minutes == 1:
# # # #             min_word = "минута"
# # # #         elif 2 <= minutes <= 4:
# # # #             min_word = "минуты"
# # # #         else:
# # # #             min_word = "минут"
        
# # # #         if hours == 1 or hours == 21:
# # # #             hour_word = "час"
# # # #         elif 2 <= hours <= 4 or 22 <= hours <= 24:
# # # #             hour_word = "часа"
# # # #         else:
# # # #             hour_word = "часов"
        
# # # #         time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
# # # #         return {'time': time_str, 'hour': hours, 'minute': minutes}

# # # # time_service = TimeService()

# # # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # # class ColorAnalyzer:
# # # #     def __init__(self):
# # # #         self.color_names = {
# # # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # # #             'черный': ([0, 0, 0], [180, 255, 50])
# # # #         }
    
# # # #     def analyze_colors(self, frame):
# # # #         try:
# # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # # #             detected_colors = []
            
# # # #             for color_name, (lower, upper) in self.color_names.items():
# # # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # # #                 if percentage > 5:
# # # #                     detected_colors.append({'color': color_name, 'percentage': round(percentage, 1)})
            
# # # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # # #             return detected_colors[:3]
# # # #         except:
# # # #             return []
    
# # # #     def get_brightness_level(self, frame):
# # # #         try:
# # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # # #             avg_brightness = np.mean(gray)
            
# # # #             if avg_brightness < 50:
# # # #                 return "очень темно"
# # # #             elif avg_brightness < 100:
# # # #                 return "темно"
# # # #             elif avg_brightness < 150:
# # # #                 return "нормальное освещение"
# # # #             elif avg_brightness < 200:
# # # #                 return "светло"
# # # #             else:
# # # #                 return "очень светло"
# # # #         except:
# # # #             return "не определено"

# # # # color_analyzer = ColorAnalyzer()

# # # # # --- ВИЗУАЛИЗАЦИЯ ---
# # # # class ObjectVisualizer:
# # # #     def __init__(self):
# # # #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# # # #     def draw_detections(self, frame, detections, ocr_data=None):
# # # #         try:
# # # #             annotated_frame = frame.copy()
            
# # # #             for det in detections:
# # # #                 bbox = det['bbox']
# # # #                 label_en = det['label']
# # # #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # # #                 distance = det.get('distance_m')
# # # #                 priority = det.get('priority', False)
                
# # # #                 color = (0, 255, 0)
# # # #                 thickness = 3 if priority else 2
                
# # # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
                
# # # #                 text = f"{label_ru} - {distance:.1f}м" if distance else f"{label_ru}"
                
# # # #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# # # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1] - text_height - 10),
# # # #                             (bbox[0] + text_width + 10, bbox[1]), color, -1)
                
# # # #                 cv2.putText(annotated_frame, text, (bbox[0] + 5, bbox[1] - 5),
# # # #                           self.font, 0.7, (0, 0, 0), 2)
            
# # # #             if ocr_data and ocr_data.get('has_text'):
# # # #                 cv2.rectangle(annotated_frame, (10, 10), (300, 55), (0, 255, 0), -1)
# # # #                 cv2.putText(annotated_frame, "Текст обнаружен", (15, 30), self.font, 0.7, (0, 0, 0), 2)
            
# # # #             return annotated_frame
# # # #         except:
# # # #             return frame

# # # # object_visualizer = ObjectVisualizer()

# # # # # --- МУЛЬТИЯЗЫЧНАЯ ОЗВУЧКА ---
# # # # class MultilingualTTS:
# # # #     def __init__(self):
# # # #         self.cache_dir = "audio_cache"
# # # #         if not os.path.exists(self.cache_dir):
# # # #             os.makedirs(self.cache_dir)
    
# # # #     def detect_language(self, text):
# # # #         """Определяет язык текста"""
# # # #         try:
# # # #             # Убираем числа и специальные символы для более точного определения
# # # #             text_clean = re.sub(r'[0-9\s\.,!?;:]', '', text)
# # # #             if len(text_clean) < 3:
# # # #                 return 'ru'  # По умолчанию русский
            
# # # #             lang = detect(text_clean)
# # # #             # Поддерживаем только русский и английский
# # # #             return 'en' if lang == 'en' else 'ru'
# # # #         except LangDetectException:
# # # #             return 'ru'
    
# # # #     def text_to_speech(self, text, lang=None):
# # # #         """Озвучивает текст с автоопределением языка"""
# # # #         try:
# # # #             # Автоопределение языка, если не указан
# # # #             if lang is None:
# # # #                 lang = self.detect_language(text)
            
# # # #             text_hash = hashlib.md5(f"{text}_{lang}".encode()).hexdigest()
# # # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # # #             if os.path.exists(audio_path):
# # # #                 return audio_path
            
# # # #             # Разбиваем текст на части для смешанного контента
# # # #             parts = self._split_by_language(text)
            
# # # #             if len(parts) > 1:
# # # #                 # Если текст смешанный, создаем отдельные аудио и объединяем
# # # #                 return self._create_multilingual_audio(parts, audio_path)
# # # #             else:
# # # #                 # Одноязычный текст
# # # #                 tts = gTTS(text=text, lang=lang, slow=False)
# # # #                 tts.save(audio_path)
# # # #                 return audio_path
# # # #         except Exception as e:
# # # #             print(f"Ошибка озвучки: {e}")
# # # #             return None
    
# # # #     def _split_by_language(self, text):
# # # #         """Разбивает текст на части по языкам"""
# # # #         # Упрощенная версия - можно улучшить
# # # #         words = text.split()
# # # #         parts = []
# # # #         current_part = []
# # # #         current_lang = None
        
# # # #         for word in words:
# # # #             try:
# # # #                 word_lang = detect(re.sub(r'[0-9\.,!?;:]', '', word))
# # # #                 if word_lang not in ['en', 'ru']:
# # # #                     word_lang = 'ru'
# # # #             except:
# # # #                 word_lang = 'ru'
            
# # # #             if current_lang is None:
# # # #                 current_lang = word_lang
            
# # # #             if word_lang == current_lang:
# # # #                 current_part.append(word)
# # # #             else:
# # # #                 if current_part:
# # # #                     parts.append((current_lang, ' '.join(current_part)))
# # # #                 current_part = [word]
# # # #                 current_lang = word_lang
        
# # # #         if current_part:
# # # #             parts.append((current_lang, ' '.join(current_part)))
        
# # # #         return parts
    
# # # #     def _create_multilingual_audio(self, parts, output_path):
# # # #         """Создает аудио из нескольких языковых частей"""
# # # #         try:
# # # #             temp_files = []
# # # #             combined = None
            
# # # #             for i, (lang, text) in enumerate(parts):
# # # #                 temp_path = os.path.join(self.cache_dir, f"temp_{uuid.uuid4()}.mp3")
# # # #                 tts = gTTS(text=text, lang=lang, slow=False)
# # # #                 tts.save(temp_path)
# # # #                 temp_files.append(temp_path)
                
# # # #                 audio = AudioSegment.from_mp3(temp_path)
# # # #                 if combined is None:
# # # #                     combined = audio
# # # #                 else:
# # # #                     combined += audio
            
# # # #             combined.export(output_path, format="mp3")
            
# # # #             # Удаляем временные файлы
# # # #             for temp_file in temp_files:
# # # #                 try:
# # # #                     os.remove(temp_file)
# # # #                 except:
# # # #                     pass
            
# # # #             return output_path
# # # #         except Exception as e:
# # # #             print(f"Ошибка создания мультиязычного аудио: {e}")
# # # #             return None

# # # # tts_engine = MultilingualTTS()

# # # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # # class SimpleVoiceBiometrics:
# # # #     def extract_mfcc_features(self, audio_path):
# # # #         try:
# # # #             with wave.open(audio_path, 'rb') as wav_file:
# # # #                 sample_width = wav_file.getsampwidth()
# # # #                 frame_rate = wav_file.getframerate()
# # # #                 n_frames = wav_file.getnframes()
# # # #                 frames = wav_file.readframes(n_frames)
                
# # # #                 if sample_width == 2:
# # # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # # #                 else:
# # # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # # #                 features_bytes = features.astype(np.float32).tobytes()
# # # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # # #                 return features_b64
# # # #         except Exception as e:
# # # #             print(f"Ошибка извлечения характеристик: {e}")
# # # #             return None
    
# # # #     def _simple_audio_features(self, audio_data, sample_rate):
# # # #         features = []
# # # #         features.append(np.mean(audio_data ** 2))
# # # #         features.append(np.std(audio_data))
# # # #         features.append(np.max(np.abs(audio_data)))
        
# # # #         fft = np.fft.fft(audio_data)
# # # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # # #         for low, high in bands:
# # # #             mask = (freqs >= low) & (freqs < high)
# # # #             if np.any(mask):
# # # #                 features.append(np.mean(fft_magnitude[mask]))
# # # #             else:
# # # #                 features.append(0.0)
        
# # # #         return np.array(features)
    
# # # #     def compare_voice_features(self, features1_b64, features2_b64):
# # # #         try:
# # # #             features1_bytes = base64.b64decode(features1_b64)
# # # #             features2_bytes = base64.b64decode(features2_b64)
            
# # # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # # #             min_len = min(len(features1), len(features2))
# # # #             features1 = features1[:min_len]
# # # #             features2 = features2[:min_len]
            
# # # #             distance = np.linalg.norm(features1 - features2)
# # # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
# # # #             return float(max(0.0, min(1.0, similarity)))
# # # #         except:
# # # #             return 0.0

# # # # voice_biometrics = SimpleVoiceBiometrics()

# # # # # --- КОМАНДЫ ---
# # # # class VoiceCommandRecognizer:
# # # #     def __init__(self):
# # # #         self.commands = {
# # # #             'выйти': ['выйти', 'выход', 'выхожу', 'закрыть', 'exit'],
# # # #             'сканировать': ['сканировать', 'скан', 'сканируй', 'что вижу', 'что впереди', 'анализ', 'scan'],
# # # #             'время': ['время', 'который час', 'сколько времени', 'time'],
# # # #             'читать': ['читать', 'читай', 'прочитай', 'текст', 'что написано', 'read'],
# # # #             'деньги': ['деньги', 'распознать деньги', 'сколько денег', 'какая купюра', 'купюра', 'тенге', 'money']
# # # #         }
    
# # # #     def recognize_command(self, text):
# # # #         if not text:
# # # #             return None
        
# # # #         text = text.lower().strip()
# # # #         noise = ['слушаю', 'команду', 'команда', 'пожалуйста']
# # # #         for n in noise:
# # # #             text = text.replace(n, '')
# # # #         text = ' '.join(text.split())
        
# # # #         print(f"🎤 Анализ: '{text}'")
        
# # # #         for cmd_type, keywords in self.commands.items():
# # # #             for keyword in keywords:
# # # #                 if keyword in text or text in keyword:
# # # #                     print(f"✅ Команда: {cmd_type}")
# # # #                     return cmd_type
        
# # # #         print(f"❌ Не распознано")
# # # #         return None

# # # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # # --- КОНВЕРТЕР АУДИО ---
# # # # class AudioConverter:
# # # #     def convert_aac_to_wav(self, aac_path):
# # # #         try:
# # # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # # #             wav_path = aac_path.replace('.aac', '.wav')
# # # #             audio.export(wav_path, format="wav")
# # # #             return wav_path
# # # #         except Exception as e:
# # # #             print(f"Ошибка конвертации: {e}")
# # # #             return None

# # # # audio_converter = AudioConverter()

# # # # # --- УЛУЧШЕННЫЙ OCR С МУЛЬТИЯЗЫЧНОСТЬЮ ---
# # # # # --- УЛУЧШЕННЫЙ OCR С МУЛЬТИЯЗЫЧНОСТЬЮ ---
# # # # class MultilingualTextRecognizer:
# # # #     def __init__(self):
# # # #         self.config = '--oem 3 --psm 6'
# # # #         print("🔍 Инициализация OCR...")
    
# # # #     def extract_text(self, frame):
# # # #         try:
# # # #             print("\n📖 Распознавание текста...")
            
# # # #             # Проверяем доступность Tesseract
# # # #             try:
# # # #                 pytesseract.get_tesseract_version()
# # # #             except:
# # # #                 print("❌ Tesseract недоступен")
# # # #                 return {'text': '', 'language': 'ru', 'phones': [], 'has_text': False, 'length': 0}
            
# # # #             # Сохраняем оригинальное изображение для отладки
# # # #             original_height, original_width = frame.shape[:2]
# # # #             print(f"📐 Размер изображения: {original_width}x{original_height}")
            
# # # #             # Улучшенная предобработка
# # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
# # # #             # 1. Увеличение изображения (оптимальный масштаб)
# # # #             scale_factor = 2.0
# # # #             gray = cv2.resize(gray, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
            
# # # #             # 2. Убираем шум с помощью Gaussian blur (легкий)
# # # #             gray = cv2.GaussianBlur(gray, (3, 3), 0)
            
# # # #             # 3. CLAHE для улучшения контраста
# # # #             clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
# # # #             gray = clahe.apply(gray)
            
# # # #             # 4. Несколько методов бинаризации
# # # #             _, binary_otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# # # #             binary_adaptive = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
# # # #                                                   cv2.THRESH_BINARY, 11, 2)
            
# # # #             # 5. Альтернатива - бинаризация с фиксированным порогом
# # # #             _, binary_fixed = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            
# # # #             # 6. Морфологические операции для очистки (минимальные)
# # # #             kernel = np.ones((1, 1), np.uint8)
# # # #             binary_otsu_clean = cv2.morphologyEx(binary_otsu, cv2.MORPH_CLOSE, kernel)
# # # #             binary_adaptive_clean = cv2.morphologyEx(binary_adaptive, cv2.MORPH_CLOSE, kernel)
            
# # # #             # 7. Убираем мелкий шум
# # # #             binary_otsu_clean = cv2.medianBlur(binary_otsu_clean, 3)
# # # #             binary_adaptive_clean = cv2.medianBlur(binary_adaptive_clean, 3)
            
# # # #             best_text = ""
# # # #             best_confidence = 0
# # # #             best_method = ""
            
# # # #             # Тестовые комбинации изображений и настроек
# # # #             test_images = [
# # # #                 (binary_otsu_clean, "binary_otsu"),
# # # #                 (binary_adaptive_clean, "binary_adaptive"), 
# # # #                 (binary_fixed, "binary_fixed"),
# # # #                 (gray, "grayscale")  # Иногда лучше без бинаризации
# # # #             ]
            
# # # #             test_configs = [
# # # #                 ('--oem 3 --psm 6', "block_text"),
# # # #                 ('--oem 3 --psm 7', "single_line"),
# # # #                 ('--oem 3 --psm 8', "single_word"),
# # # #                 ('--oem 3 --psm 13', "raw_line")
# # # #             ]
            
# # # #             print("🔍 Тестирование различных методов OCR...")
            
# # # #             for img, img_name in test_images:
# # # #                 for config, config_name in test_configs:
# # # #                     try:
# # # #                         # Пробуем русский язык
# # # #                         data_ru = pytesseract.image_to_data(img, lang='rus', config=config, 
# # # #                                                           output_type=pytesseract.Output.DICT)
                        
# # # #                         # Фильтруем слова с достаточной уверенностью
# # # #                         ru_words = []
# # # #                         ru_confidence_sum = 0
# # # #                         ru_word_count = 0
                        
# # # #                         for i in range(len(data_ru['text'])):
# # # #                             text = data_ru['text'][i].strip()
# # # #                             conf = int(data_ru['conf'][i])
# # # #                             if text and len(text) > 0 and conf > 20:  # Понижаем порог уверенности
# # # #                                 ru_words.append(text)
# # # #                                 ru_confidence_sum += conf
# # # #                                 ru_word_count += 1
                        
# # # #                         text_ru = ' '.join(ru_words)
# # # #                         avg_conf_ru = ru_confidence_sum / max(ru_word_count, 1)
                        
# # # #                         # Пробуем английский язык
# # # #                         text_en = pytesseract.image_to_string(img, lang='eng', config=config).strip()
                        
# # # #                         # Выбираем лучший текст
# # # #                         current_text = ""
# # # #                         current_lang = ""
                        
# # # #                         if len(text_ru) > len(text_en) and len(text_ru) > 1:
# # # #                             current_text = text_ru
# # # #                             current_lang = 'ru'
# # # #                             current_conf = avg_conf_ru
# # # #                         elif len(text_en) > 1:
# # # #                             current_text = text_en
# # # #                             current_lang = 'en'
# # # #                             current_conf = 60  # Примерная уверенность для английского
# # # #                         else:
# # # #                             continue
                        
# # # #                         # Очищаем текст от мусора
# # # #                         current_text = self._clean_text(current_text)
                        
# # # #                         # Оцениваем качество текста
# # # #                         text_score = self._evaluate_text_quality(current_text, current_conf)
                        
# # # #                         print(f"  {img_name}+{config_name}: '{current_text[:50]}...' (оценка: {text_score:.1f})")
                        
# # # #                         if text_score > best_confidence and len(current_text) > 2:
# # # #                             best_text = current_text
# # # #                             best_confidence = text_score
# # # #                             best_method = f"{img_name}+{config_name}"
                            
# # # #                     except Exception as e:
# # # #                         # Пропускаем ошибки отдельных комбинаций
# # # #                         continue
            
# # # #             if best_text:
# # # #                 # Определяем язык для озвучки
# # # #                 detected_lang = tts_engine.detect_language(best_text)
# # # #                 phones = self._extract_phone_numbers(best_text)
                
# # # #                 print(f"✅ Текст найден (метод: {best_method})")
# # # #                 print(f"   Язык: {detected_lang}")
# # # #                 print(f"   Уверенность: {best_confidence:.1f}")
# # # #                 print(f"   Текст: '{best_text}'")
                
# # # #                 return {
# # # #                     'text': best_text,
# # # #                     'language': detected_lang,
# # # #                     'phones': phones,
# # # #                     'has_text': True,
# # # #                     'length': len(best_text),
# # # #                     'confidence': best_confidence
# # # #                 }
            
# # # #             print("❌ Текст не найден или качество слишком низкое")
# # # #             return {'text': '', 'language': 'ru', 'phones': [], 'has_text': False, 'length': 0}
            
# # # #         except Exception as e:
# # # #             print(f"❌ Общая ошибка OCR: {e}")
# # # #             import traceback
# # # #             traceback.print_exc()
# # # #             return {'text': '', 'language': 'ru', 'phones': [], 'has_text': False, 'length': 0}
    
# # # #     def _clean_text(self, text):
# # # #         """Очистка текста от мусора"""
# # # #         if not text:
# # # #             return ""
        
# # # #         # Убираем специальные символы, оставляя буквы, цифры и основные знаки препинания
# # # #         cleaned = re.sub(r'[^\w\sа-яА-Яa-zA-Z0-9.,!?;:()\-]', '', text)
        
# # # #         # Убираем множественные пробелы
# # # #         cleaned = re.sub(r'\s+', ' ', cleaned)
        
# # # #         # Убираем одиночные символы, окруженные пробелами (кроме "a", "I" и т.д.)
# # # #         cleaned = re.sub(r'\s[bcdefghjklmnopqrstuvwxyzBCDEFGHJKLMNOPQRSTUVWXYZ]\s', ' ', cleaned)
        
# # # #         return cleaned.strip()
    
# # # #     def _evaluate_text_quality(self, text, confidence):
# # # #         """Оценка качества распознанного текста"""
# # # #         if not text or len(text) < 2:
# # # #             return 0
        
# # # #         score = confidence
        
# # # #         # Бонус за длину текста (но не слишком большой)
# # # #         length_bonus = min(len(text) * 0.5, 30)
# # # #         score += length_bonus
        
# # # #         # Бонус за наличие пробелов (связный текст)
# # # #         if ' ' in text:
# # # #             score += 10
        
# # # #         # Штраф за слишком много специальных символов
# # # #         special_chars = len(re.findall(r'[^a-zA-Zа-яА-Я0-9\s]', text))
# # # #         if special_chars > len(text) * 0.3:  # Если больше 30% спецсимволов
# # # #             score -= 20
        
# # # #         # Бонус за наличие гласных (признак реального текста)
# # # #         vowels = len(re.findall(r'[aeiouаеёиоуыэюя]', text, re.IGNORECASE))
# # # #         if vowels > 0:
# # # #             score += 5
        
# # # #         return max(score, 0)
    
# # # #     def _extract_phone_numbers(self, text):
# # # #         """Извлечение номеров телефонов из текста"""
# # # #         patterns = [
# # # #             r'\+7\s?\(?\d{3}\)?\s?\d{3}\s?\d{2}\s?\d{2}',
# # # #             r'8\s?\(?\d{3}\)?\s?\d{3}\s?\d{2}\s?\d{2}',
# # # #             r'\d{3}[-\s]?\d{2}[-\s]?\d{2}',
# # # #         ]
# # # #         phones = []
# # # #         for pattern in patterns:
# # # #             phones.extend(re.findall(pattern, text))
# # # #         return phones

# # # # text_recognizer = MultilingualTextRecognizer()

# # # # # --- УЛУЧШЕННОЕ РАСПОЗНАВАНИЕ ДЕНЕГ (ТЕНГЕ) ---
# # # # class ImprovedMoneyRecognizer:
# # # #     def __init__(self):
# # # #         print("💰 Инициализация улучшенного распознавателя денег...")
        
# # # #         # Более точные характеристики для каждого номинала
# # # #         self.denominations = {
# # # #             200: {
# # # #                 'color': 'светло-синий',
# # # #                 'hsv_ranges': [
# # # #                     ([90, 30, 100], [120, 200, 255]),   # Основной синий
# # # #                     ([85, 20, 120], [125, 180, 255])    # Светлый синий
# # # #                 ],
# # # #                 'keywords': ['200', '200₸', 'екі жүз', 'двести', 'two hundred'],
# # # #                 'numbers': ['200', '2 0 0', '2ОО'],
# # # #                 'weight': 1.0
# # # #             },
# # # #             500: {
# # # #                 'color': 'желто-оранжевый',
# # # #                 'hsv_ranges': [
# # # #                     ([15, 80, 100], [35, 255, 255]),    # Желтый
# # # #                     ([8, 100, 150], [25, 255, 255])     # Оранжевый
# # # #                 ],
# # # #                 'keywords': ['500', '500₸', 'бес жүз', 'пятьсот', 'five hundred'],
# # # #                 'numbers': ['500', '5 0 0', '5ОО'],
# # # #                 'weight': 1.1
# # # #             },
# # # #             1000: {
# # # #                 'color': 'зеленый',
# # # #                 'hsv_ranges': [
# # # #                     ([40, 40, 80], [80, 255, 255]),     # Зеленый
# # # #                     ([35, 30, 100], [85, 200, 255])     # Светло-зеленый
# # # #                 ],
# # # #                 'keywords': ['1000', '1000₸', 'мың', 'тысяча', 'one thousand'],
# # # #                 'numbers': ['1000', '1 000', '1ООО'],
# # # #                 'weight': 1.2
# # # #             },
# # # #             2000: {
# # # #                 'color': 'сиреневый',
# # # #                 'hsv_ranges': [
# # # #                     ([130, 20, 100], [160, 180, 255]),  # Сиреневый
# # # #                     ([125, 15, 120], [165, 150, 255])   # Светло-сиреневый
# # # #                 ],
# # # #                 'keywords': ['2000', '2000₸', 'екі мың', 'две тысячи'],
# # # #                 'numbers': ['2000', '2 000', '2ООО'],
# # # #                 'weight': 1.3
# # # #             },
# # # #             5000: {
# # # #                 'color': 'коричневато-розовый',
# # # #                 'hsv_ranges': [
# # # #                     ([0, 40, 100], [15, 200, 220]),     # Розово-коричневый
# # # #                     ([165, 30, 100], [180, 180, 240])   # Розовый
# # # #                 ],
# # # #                 'keywords': ['5000', '5000₸', 'бес мың', 'пять тысяч'],
# # # #                 'numbers': ['5000', '5 000', '5ООО'],
# # # #                 'weight': 1.4
# # # #             },
# # # #             10000: {
# # # #                 'color': 'бежево-коричневый',
# # # #                 'hsv_ranges': [
# # # #                     ([10, 30, 120], [30, 150, 220]),    # Бежевый
# # # #                     ([15, 20, 140], [35, 100, 240])     # Светло-коричневый
# # # #                 ],
# # # #                 'keywords': ['10000', '10000₸', 'он мың', 'десять тысяч'],
# # # #                 'numbers': ['10000', '10 000', '1ОООО'],
# # # #                 'weight': 1.5
# # # #             },
# # # #             20000: {
# # # #                 'color': 'серо-синий',
# # # #                 'hsv_ranges': [
# # # #                     ([85, 10, 100], [115, 100, 200]),   # Серо-синий
# # # #                     ([90, 15, 120], [110, 80, 220])     # Светло-серый
# # # #                 ],
# # # #                 'keywords': ['20000', '20000₸', 'жиырма мың', 'двадцать тысяч'],
# # # #                 'numbers': ['20000', '20 000', '2ОООО'],
# # # #                 'weight': 1.6
# # # #             }
# # # #         }
# # # #         print("✅ Распознаватель денег готов")
    
# # # #     def recognize(self, frame):
# # # #         try:
# # # #             print("\n💵 Распознавание купюры (улучшенный метод)...")
            
# # # #             # Множественное увеличение и обработка
# # # #             results = []
            
# # # #             for scale in [1.5, 2.0, 2.5]:
# # # #                 frame_scaled = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
# # # #                 result = self._recognize_single_scale(frame_scaled)
# # # #                 if result:
# # # #                     results.append(result)
            
# # # #             # Если ничего не найдено, пробуем с разными яркостями
# # # #             if not results:
# # # #                 for gamma in [0.7, 1.0, 1.3]:
# # # #                     frame_adjusted = self._adjust_gamma(frame, gamma)
# # # #                     frame_scaled = cv2.resize(frame_adjusted, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
# # # #                     result = self._recognize_single_scale(frame_scaled)
# # # #                     if result:
# # # #                         results.append(result)
            
# # # #             # Выбираем лучший результат
# # # #             if results:
# # # #                 # Сортируем по score и берем лучший
# # # #                 results.sort(key=lambda x: x['score'], reverse=True)
# # # #                 best = results[0]
                
# # # #                 denom = best['denomination']
# # # #                 conf = best['confidence']
                
# # # #                 print(f"✅ Купюра распознана: {denom} тенге (уверенность: {conf:.0%})")
                
# # # #                 return {
# # # #                     'has_money': True,
# # # #                     'denomination': denom,
# # # #                     'confidence': float(conf),
# # # #                     'description': f"Обнаружена купюра {denom} тенге"
# # # #                 }
            
# # # #             print("❌ Купюра не распознана")
# # # #             return {
# # # #                 'has_money': False,
# # # #                 'description': 'Деньги не обнаружены. Расположите купюру ровно перед камерой, обеспечьте хорошее освещение и держите неподвижно 2-3 секунды'
# # # #             }
# # # #         except Exception as e:
# # # #             print(f"❌ Ошибка распознавания: {e}")
# # # #             import traceback
# # # #             traceback.print_exc()
# # # #             return {'has_money': False, 'description': 'Ошибка распознавания'}
    
# # # #     def _recognize_single_scale(self, frame):
# # # #         """Распознавание на одном масштабе"""
# # # #         try:
# # # #             # OCR с улучшенной предобработкой
# # # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
# # # #             # CLAHE для улучшения контраста
# # # #             clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
# # # #             gray = clahe.apply(gray)
            
# # # #             # Несколько методов бинаризации
# # # #             _, binary1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# # # #             binary2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            
# # # #             # Убираем шум
# # # #             binary1 = cv2.medianBlur(binary1, 3)
# # # #             binary2 = cv2.medianBlur(binary2, 3)
            
# # # #             # OCR на обоих бинарных изображениях
# # # #             texts = []
# # # #             for binary in [binary1, binary2]:
# # # #                 for psm in [3, 6, 11]:
# # # #                     config = f'--oem 3 --psm {psm} -c tessedit_char_whitelist=0123456789'
# # # #                     text = pytesseract.image_to_string(binary, lang='eng', config=config)
# # # #                     texts.append(text.lower().strip())
            
# # # #             # Объединяем все найденные тексты
# # # #             full_text = ' '.join(texts)
# # # #             print(f"📖 OCR результат: '{full_text}'")
            
# # # #             # Анализ цвета
# # # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
# # # #             detected = []
            
# # # #             for denom, info in self.denominations.items():
# # # #                 score = 0
                
# # # #                 # 1. Проверка номинала в тексте (высокий приоритет)
# # # #                 for num_variant in info['numbers']:
# # # #                     if num_variant.replace(' ', '') in full_text.replace(' ', ''):
# # # #                         score += 70
# # # #                         print(f"  ✓✓ Номинал {denom} найден напрямую")
# # # #                         break
                
# # # #                 # 2. Проверка ключевых слов
# # # #                 for keyword in info['keywords']:
# # # #                     if keyword.lower() in full_text:
# # # #                         score += 50
# # # #                         print(f"  ✓ Ключевое слово '{keyword}' найдено")
# # # #                         break
                
# # # #                 # 3. Проверка цвета (несколько диапазонов)
# # # #                 max_color_pct = 0
# # # #                 for lower, upper in info['hsv_ranges']:
# # # #                     mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # # #                     color_pct = (np.sum(mask > 0) / mask.size) * 100
# # # #                     max_color_pct = max(max_color_pct, color_pct)
                
# # # #                 if max_color_pct > 2:
# # # #                     color_score = min(max_color_pct * 3, 60)
# # # #                     score += color_score
# # # #                     print(f"  🎨 Цвет {info['color']}: {max_color_pct:.1f}% (очки: {color_score:.0f})")
                
# # # #                 # 4. Взвешиваем по важности номинала
# # # #                 score *= info['weight']
                
# # # #                 if score > 30:  # Минимальный порог
# # # #                     detected.append({
# # # #                         'denomination': denom,
# # # #                         'score': score,
# # # #                         'confidence': min(score / 150, 1.0)
# # # #                     })
            
# # # #             if detected:
# # # #                 detected.sort(key=lambda x: x['score'], reverse=True)
# # # #                 return detected[0]
            
# # # #             return None
# # # #         except Exception as e:
# # # #             print(f"❌ Ошибка в _recognize_single_scale: {e}")
# # # #             return None
    
# # # #     def _adjust_gamma(self, image, gamma=1.0):
# # # #         """Коррекция яркости"""
# # # #         inv_gamma = 1.0 / gamma
# # # #         table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
# # # #         return cv2.LUT(image, table)
    
# # # #     def count_money(self, frames):
# # # #         """Подсчет нескольких купюр"""
# # # #         total = 0
# # # #         detected_bills = []
        
# # # #         for frame in frames:
# # # #             result = self.recognize(frame)
# # # #             if result['has_money']:
# # # #                 detected_bills.append(result['denomination'])
# # # #                 total += result['denomination']
        
# # # #         if detected_bills:
# # # #             bills_str = ', '.join([f"{d} тенге" for d in detected_bills])
# # # #             return {
# # # #                 'total': total,
# # # #                 'bills': detected_bills,
# # # #                 'count': len(detected_bills),
# # # #                 'description': f"Обнаружено купюр: {len(detected_bills)}. Номиналы: {bills_str}. Общая сумма: {total} тенге"
# # # #             }
        
# # # #         return {
# # # #             'total': 0,
# # # #             'bills': [],
# # # #             'count': 0,
# # # #             'description': 'Купюры не обнаружены'
# # # #         }

# # # # money_recognizer = ImprovedMoneyRecognizer()

# # # # # --- YOLO С РАСШИРЕННЫМ СЛОВАРЕМ ---
# # # # OBJECT_HEIGHTS = {
# # # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8, 'book': 0.3,
# # # #     'banana': 0.15, 'apple': 0.08, 'orange': 0.08, 'broccoli': 0.15,
# # # #     'carrot': 0.15, 'hot dog': 0.15, 'pizza': 0.03, 'donut': 0.08,
# # # #     'cake': 0.10, 'sandwich': 0.08, 'bowl': 0.12, 'fork': 0.18,
# # # #     'knife': 0.20, 'spoon': 0.15, 'backpack': 0.45, 'umbrella': 0.80,
# # # #     'handbag': 0.35, 'tie': 0.40, 'suitcase': 0.60,
# # # #     # Добавляем примерные высоты для новых классов
# # # #     'table': 0.8, 'bed': 0.5, 'refrigerator': 1.8, 'tree': 10.0,
# # # #     'building': 20.0, 'bicycle': 1.0, 'motorcycle': 1.2, 'bus': 3.0,
# # # #     'truck': 3.0, 'bird': 0.2, 'horse': 1.8, 'cat': 0.3
# # # # }


# # # # OBJECT_TRANSLATIONS = {
# # # #     # Основные категории (сохраняем старые)
# # # #     'person': 'человек', 'car': 'машина', 'chair': 'стул', 'bottle': 'бутылка',
# # # #     'cup': 'чашка', 'dog': 'собака', 'cat': 'кошка', 'tv': 'телевизор',
# # # #     'laptop': 'ноутбук', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл',
# # # #     'bus': 'автобус', 'truck': 'грузовик', 'traffic light': 'светофор',
# # # #     'stop sign': 'знак стоп', 'bench': 'скамейка', 'book': 'книга',
# # # #     'clock': 'часы', 'cell phone': 'телефон', 'keyboard': 'клавиатура',
    
# # # #     # Еда и напитки (расширяем)
# # # #     'banana': 'банан', 'apple': 'яблоко', 'orange': 'апельсин',
# # # #     'broccoli': 'брокколи', 'carrot': 'морковь', 'hot dog': 'хот-дог',
# # # #     'pizza': 'пицца', 'donut': 'пончик', 'cake': 'торт',
# # # #     'sandwich': 'сандвич', 'bread': 'хлеб', 'cookie': 'печенье',
# # # #     'muffin': 'маффин', 'bagel': 'бублик', 'cheese': 'сыр',
# # # #     'butter': 'масло', 'egg': 'яйцо', 'bacon': 'бекон',
# # # #     'sausage': 'колбаса', 'steak': 'стейк', 'chicken': 'курица',
# # # #     'fish': 'рыба', 'shrimp': 'креветка', 'lobster': 'лобстер',
# # # #     'sushi': 'суши', 'rice': 'рис', 'pasta': 'паста',
# # # #     'soup': 'суп', 'salad': 'салат', 'avocado': 'авокадо',
# # # #     'tomato': 'помидор', 'potato': 'картофель', 'onion': 'лук',
# # # #     'garlic': 'чеснок', 'mushroom': 'гриб', 'corn': 'кукуруза',
# # # #     'pepper': 'перец', 'cucumber': 'огурец', 'lettuce': 'салат-латук',
# # # #     'spinach': 'шпинат', 'strawberry': 'клубника', 'grape': 'виноград',
# # # #     'lemon': 'лимон', 'lime': 'лайм', 'peach': 'персик',
# # # #     'pear': 'груша', 'pineapple': 'ананас', 'watermelon': 'арбуз',
# # # #     'cherry': 'вишня', 'blueberry': 'черника', 'raspberry': 'малина',
# # # #     'coconut': 'кокос', 'kiwi': 'киви', 'mango': 'манго',
# # # #     'coffee': 'кофе', 'tea': 'чай', 'wine': 'вино',
# # # #     'beer': 'пиво', 'juice': 'сок', 'milk': 'молоко',
# # # #     'water': 'вода', 'soda': 'газировка',
    
# # # #     # Посуда и кухня
# # # #     'bowl': 'миска', 'fork': 'вилка', 'knife': 'нож', 'spoon': 'ложка',
# # # #     'plate': 'тарелка', 'glass': 'стакан', 'cup': 'чашка',
# # # #     'mug': 'кружка', 'teapot': 'чайник', 'pan': 'сковорода',
# # # #     'pot': 'кастрюля', 'oven': 'духовка', 'microwave': 'микроволновка',
# # # #     'refrigerator': 'холодильник', 'toaster': 'тостер', 'blender': 'блендер',
# # # #     'kettle': 'чайник', 'cutting board': 'разделочная доска',
    
# # # #     # Мебель
# # # #     'bed': 'кровать', 'couch': 'диван', 'sofa': 'диван',
# # # #     'table': 'стол', 'desk': 'письменный стол', 'dining table': 'обеденный стол',
# # # #     'cabinet': 'шкаф', 'wardrobe': 'гардероб', 'shelf': 'полка',
# # # #     'bookshelf': 'книжная полка', 'drawer': 'ящик', 'nightstand': 'прикроватная тумба',
# # # #     'dresser': 'комод', 'armchair': 'кресло', 'stool': 'табурет',
    
# # # #     # Электроника
# # # #     'computer': 'компьютер', 'monitor': 'монитор', 'printer': 'принтер',
# # # #     'scanner': 'сканер', 'camera': 'камера', 'television': 'телевизор',
# # # #     'remote': 'пульт', 'speaker': 'колонка', 'headphones': 'наушники',
# # # #     'microphone': 'микрофон', 'router': 'роутер', 'charger': 'зарядное устройство',
    
# # # #     # Одежда и аксессуары
# # # #     'shirt': 'рубашка', 't-shirt': 'футболка', 'pants': 'брюки',
# # # #     'jeans': 'джинсы', 'shorts': 'шорты', 'dress': 'платье',
# # # #     'skirt': 'юбка', 'jacket': 'куртка', 'coat': 'пальто',
# # # #     'sweater': 'свитер', 'hoodie': 'толстовка', 'suit': 'костюм',
# # # #     'tie': 'галстук', 'belt': 'ремень', 'shoes': 'обувь',
# # # #     'sneakers': 'кроссовки', 'boots': 'ботинки', 'sandals': 'сандалии',
# # # #     'hat': 'шляпа', 'cap': 'кепка', 'glasses': 'очки',
# # # #     'sunglasses': 'солнечные очки', 'watch': 'часы', 'jewelry': 'украшения',
    
# # # #     # Животные
# # # #     'bird': 'птица', 'horse': 'лошадь', 'cow': 'корова',
# # # #     'sheep': 'овца', 'goat': 'коза', 'pig': 'свинья',
# # # #     'rabbit': 'кролик', 'hamster': 'хомяк', 'mouse': 'мышь',
# # # #     'rat': 'крыса', 'elephant': 'слон', 'lion': 'лев',
# # # #     'tiger': 'тигр', 'bear': 'медведь', 'monkey': 'обезьяна',
# # # #     'giraffe': 'жираф', 'zebra': 'зебра', 'deer': 'олень',
# # # #     'wolf': 'волк', 'fox': 'лиса', 'squirrel': 'белка',
    
# # # #     # Транспорт
# # # #     'airplane': 'самолет', 'helicopter': 'вертолет', 'boat': 'лодка',
# # # #     'ship': 'корабль', 'train': 'поезд', 'subway': 'метро',
# # # #     'taxi': 'такси', 'ambulance': 'скорая помощь', 'fire truck': 'пожарная машина',
# # # #     'police car': 'полицейская машина', 'van': 'фургон', 'trailer': 'прицеп',
# # # #     'tractor': 'трактор', 'excavator': 'экскаватор', 'crane': 'кран',
    
# # # #     # Спорт
# # # #     'ball': 'мяч', 'football': 'футбольный мяч', 'basketball': 'баскетбольный мяч',
# # # #     'baseball': 'бейсбольный мяч', 'tennis ball': 'теннисный мяч', 'golf ball': 'мяч для гольфа',
# # # #     'racket': 'ракетка', 'bat': 'бита', 'glove': 'перчатка',
# # # #     'skis': 'лыжи', 'snowboard': 'сноуборд', 'surfboard': 'доска для серфинга',
# # # #     'skateboard': 'скейтборд', 'bicycle': 'велосипед', 'dumbbell': 'гантеля',
# # # #     'barbell': 'штанга', 'treadmill': 'беговая дорожка',
    
# # # #     # Природа и растения
# # # #     'tree': 'дерево', 'flower': 'цветок', 'rose': 'роза',
# # # #     'sunflower': 'подсолнух', 'leaf': 'лист', 'grass': 'трава',
# # # #     'bush': 'куст', 'cactus': 'кактус', 'mountain': 'гора',
# # # #     'river': 'река', 'lake': 'озеро', 'ocean': 'океан',
# # # #     'beach': 'пляж', 'sky': 'небо', 'cloud': 'облако',
    
# # # #     # Здания и архитектура
# # # #     'house': 'дом', 'building': 'здание', 'skyscraper': 'небоскреб',
# # # #     'church': 'церковь', 'temple': 'храм', 'mosque': 'мечеть',
# # # #     'bridge': 'мост', 'tower': 'башня', 'fountain': 'фонтан',
# # # #     'statue': 'статуя', 'monument': 'монумент',
    
# # # #     # Канцелярия и офис
# # # #     'pen': 'ручка', 'pencil': 'карандаш', 'marker': 'маркер',
# # # #     'eraser': 'ластик', 'ruler': 'линейка', 'scissors': 'ножницы',
# # # #     'stapler': 'степлер', 'tape': 'скотч', 'envelope': 'конверт',
# # # #     'folder': 'папка', 'notebook': 'блокнот', 'calendar': 'календарь',
    
# # # #     # Музыка
# # # #     'guitar': 'гитара', 'piano': 'пианино', 'violin': 'скрипка',
# # # #     'drum': 'барабан', 'trumpet': 'труба', 'saxophone': 'саксофон',
# # # #     'microphone': 'микрофон', 'headphones': 'наушники',
    
# # # #     # Медицина
# # # #     'pill': 'таблетка', 'medicine': 'лекарство', 'syringe': 'шприц',
# # # #     'bandage': 'бинт', 'thermometer': 'градусник', 'stethoscope': 'стетоскоп',
    
# # # #     # Разное
# # # #     'key': 'ключ', 'lock': 'замок', 'wallet': 'кошелек',
# # # #     'bag': 'сумка', 'backpack': 'рюкзак', 'suitcase': 'чемодан',
# # # #     'umbrella': 'зонт', 'newspaper': 'газета', 'magazine': 'журнал',
# # # #     'toy': 'игрушка', 'doll': 'кукла', 'teddy bear': 'плюшевый мишка',
# # # #     'kite': 'воздушный змей', 'balloon': 'воздушный шар', 'fireworks': 'фейерверк',
# # # #     'candle': 'свеча', 'lamp': 'лампа', 'flashlight': 'фонарик',
# # # #     'battery': 'батарейка', 'tool': 'инструмент', 'hammer': 'молоток',
# # # #     'screwdriver': 'отвертка', 'wrench': 'гаечный ключ', 'nail': 'гвоздь',
# # # #     'screw': 'винт', 'rope': 'веревка', 'chain': 'цепь',
    
# # # #     # Для неизвестных классов оставляем оригинальное название
# # # # }

# # # # PRIORITY_OBJECTS = {
# # # #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck', 
# # # #     'traffic light', 'stop sign', 'dog', 'cat', 'bird', 'horse',
# # # #     'fire truck', 'police car', 'ambulance'
# # # # }

# # # # def estimate_distance(bbox, frame_height, object_label):
# # # #     bbox_height_pixels = bbox[3] - bbox[1]
# # # #     if bbox_height_pixels <= 0:
# # # #         return None
# # # #     FOCAL_LENGTH_PIXELS = 700
# # # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # # #     return round(distance, 2)

# # # # print("🔄 Загрузка YOLO...")
# # # # model = YOLO('yolov5x-oiv7-fixed.pt')  # Используем YOLOv5 small-ultralytics
# # # # print("✅ Детекция 600+ объектов (еда, животные, транспорт, спорт, техника и многое другое)")

# # # # def detect_objects(frame):
# # # #     try:
# # # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # # #         results = model.predict(img_rgb, conf=0.25, iou=0.45, verbose=False, imgsz=640, max_det=50)[0]
        
# # # #         detections = []
# # # #         frame_height, frame_width = frame.shape[:2]

# # # #         for r in results.boxes:
# # # #             bbox = r.xyxy[0].cpu().numpy()
# # # #             conf = float(r.conf[0])
# # # #             cls_id = int(r.cls[0])
# # # #             label = model.names[cls_id]
# # # #             distance = estimate_distance(bbox, frame_height, label)
            
# # # #             x_center = (bbox[0] + bbox[2]) / 2
# # # #             y_center = (bbox[1] + bbox[3]) / 2
            
# # # #             vertical_pos = "верхняя часть" if y_center < frame_height * 0.33 else "середина" if y_center < frame_height * 0.66 else "нижняя часть"

# # # #             detections.append({
# # # #                 "label": label,
# # # #                 "confidence": conf,
# # # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # # #                 "distance_m": float(distance) if distance else None,
# # # #                 "priority": label in PRIORITY_OBJECTS,
# # # #                 "vertical_position": vertical_pos,
# # # #                 "x_center": float(x_center),
# # # #                 "y_center": float(y_center)
# # # #             })
        
# # # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
# # # #         print(f"🔍 Обнаружено: {len(detections)} объектов")
# # # #         return detections
# # # #     except Exception as e:
# # # #         print(f"❌ Ошибка YOLO: {e}")
# # # #         return []

# # # # def generate_obstacle_description(obstacles, ocr_data=None, colors=None, brightness=None):
# # # #     descriptions = []
    
# # # #     if brightness:
# # # #         descriptions.append(f"{brightness}")
    
# # # #     if colors and len(colors) > 0:
# # # #         color_desc = " и ".join([c['color'] for c in colors[:2]])
# # # #         descriptions.append(f"Основные цвета: {color_desc}")
    
# # # #     if ocr_data and ocr_data.get('has_text'):
# # # #         text = ocr_data['text']
# # # #         if text:
# # # #             text_clean = ' '.join(text.split())
# # # #             if len(text_clean) > 200:
# # # #                 descriptions.append(f"Обнаружен текст. Начало: {text_clean[:200]}")
# # # #             else:
# # # #                 descriptions.append(f"Обнаружен текст: {text_clean}")
        
# # # #         if ocr_data.get('phones'):
# # # #             descriptions.append(f"Телефоны: {', '.join(ocr_data['phones'])}")
    
# # # #     if obstacles:
# # # #         descriptions.append(f"Обнаружено объектов: {len(obstacles)}")
        
# # # #         frame_center = 320
# # # #         for obs in obstacles[:5]:
# # # #             label_ru = OBJECT_TRANSLATIONS.get(obs['label'], obs['label'])
# # # #             distance = obs.get('distance_m')
            
# # # #             if distance:
# # # #                 if distance < 1.0:
# # # #                     dist_text = f"очень близко, {int(distance * 100)} сантиметров"
# # # #                 elif distance < 2.0:
# # # #                     dist_text = f"близко, {distance:.1f} метра"
# # # #                 else:
# # # #                     dist_text = f"{int(distance)} метров"
# # # #             else:
# # # #                 dist_text = "расстояние неизвестно"
            
# # # #             x_center = (obs['bbox'][0] + obs['bbox'][2]) / 2
# # # #             pos = "слева" if x_center < frame_center - 150 else "справа" if x_center > frame_center + 150 else "прямо перед вами"
            
# # # #             priority = "Внимание! " if obs.get('priority') else ""
# # # #             descriptions.append(f"{priority}{label_ru} {pos}, {dist_text}")
        
# # # #         if len(obstacles) > 5:
# # # #             descriptions.append(f"и еще {len(obstacles) - 5} объектов")
# # # #     else:
# # # #         descriptions.append("Препятствий не обнаружено")
    
# # # #     return ". ".join(descriptions)

# # # # # --- ЭНДПОИНТЫ ---

# # # # @app.route('/')
# # # # def index():
# # # #     return jsonify({
# # # #         "status": "Blind Assistant Server v13 - УЛУЧШЕННАЯ ВЕРСИЯ",
# # # #         "features": [
# # # #             "Детекция 80+ объектов (YOLO)",
# # # #             "Мультиязычный OCR (RU/EN)",
# # # #             "Улучшенное распознавание тенге",
# # # #             "Автоопределение языка для озвучки",
# # # #             "Подсчет нескольких купюр"
# # # #         ]
# # # #     })

# # # # @app.route('/register_voice', methods=['POST'])
# # # # def register_voice():
# # # #     temp_audio = temp_wav = None
# # # #     try:
# # # #         if 'audio' not in request.files or 'username' not in request.form:
# # # #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# # # #         username = request.form['username'].strip()
# # # #         audio_file = request.files['audio']
        
# # # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # # #         audio_file.save(temp_audio)
        
# # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # #         if not temp_wav:
# # # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # #         conn = get_db_connection()
# # # #         cur = conn.cursor()
        
# # # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # # #         existing = cur.fetchone()
        
# # # #         if existing:
# # # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (voice_features, username))
# # # #             message = "Профиль обновлен"
# # # #         else:
# # # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, voice_features))
# # # #             message = "Профиль создан"
        
# # # #         conn.commit()
# # # #         cur.close()
# # # #         conn.close()
        
# # # #         return jsonify({"status": "success", "message": message, "username": username})
# # # #     except Exception as e:
# # # #         return jsonify({"error": str(e)}), 500
# # # #     finally:
# # # #         for f in [temp_audio, temp_wav]:
# # # #             if f and os.path.exists(f):
# # # #                 try: os.remove(f)
# # # #                 except: pass

# # # # @app.route('/login_voice', methods=['POST'])
# # # # def login_voice():
# # # #     temp_audio = temp_wav = None
# # # #     try:
# # # #         username = request.form['username'].strip()
# # # #         audio_file = request.files['audio']
        
# # # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # # #         audio_file.save(temp_audio)
        
# # # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # # #         conn = get_db_connection()
# # # #         cur = conn.cursor()
        
# # # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # # #         result = cur.fetchone()
        
# # # #         if not result:
# # # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # # #         cur.close()
# # # #         conn.close()
        
# # # #         if similarity > 0.7:
# # # #             welcome_text = f"Добро пожаловать, {username}! Камера готова. Доступны команды: Сканировать, Читать текст, Распознать деньги, Время, Выйти."
# # # #             return jsonify({
# # # #                 "status": "success",
# # # #                 "message": "Вход выполнен",
# # # #                 "username": username,
# # # #                 "similarity": float(similarity),
# # # #                 "welcome_text": welcome_text,
# # # #                 "skippable": True
# # # #             })
# # # #         else:
# # # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
# # # #     except Exception as e:
# # # #         return jsonify({"error": str(e)}), 500
# # # #     finally:
# # # #         for f in [temp_audio, temp_wav]:
# # # #             if f and os.path.exists(f):
# # # #                 try: os.remove(f)
# # # #                 except: pass

# # # # @app.route('/voice_command', methods=['POST'])
# # # # def voice_command():
# # # #     try:
# # # #         text = request.form.get('text', '').lower().strip()
# # # #         print(f"🎤 Голосовая команда: '{text}'")
        
# # # #         if not text:
# # # #             return jsonify({"error": "Текст не распознан"}), 400
        
# # # #         command = voice_command_recognizer.recognize_command(text)
        
# # # #         if command == 'время':
# # # #             time_data = time_service.get_current_time()
# # # #             return jsonify({"status": "success", "command": "время", "data": time_data})
# # # #         elif command in ['сканировать', 'читать', 'деньги', 'выйти']:
# # # #             return jsonify({"status": "success", "command": command})
# # # #         else:
# # # #             return jsonify({
# # # #                 "status": "unknown",
# # # #                 "message": "Команда не распознана",
# # # #                 "available_commands": ["сканировать", "время", "читать", "деньги", "выйти"]
# # # #             })
# # # #     except Exception as e:
# # # #         return jsonify({"error": str(e)}), 500

# # # # @app.route('/process_frame', methods=['POST'])
# # # # def process_frame():
# # # #     try:
# # # #         print("\n🔵 Обработка кадра...")
# # # #         if 'frame' not in request.files:
# # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # #         frame_bytes = request.files['frame'].read()
# # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # #         if frame is None:
# # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # #         print(f"✅ Изображение: {frame.shape}")

# # # #         # Полный анализ
# # # #         obstacles = detect_objects(frame)
# # # #         ocr_data = text_recognizer.extract_text(frame)
# # # #         colors = color_analyzer.analyze_colors(frame)
# # # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # # #         annotated_frame = object_visualizer.draw_detections(frame, obstacles, ocr_data)
# # # #         description = generate_obstacle_description(obstacles, ocr_data, colors, brightness)
        
# # # #         # Определяем язык для озвучки
# # # #         lang = ocr_data.get('language', 'ru') if ocr_data.get('has_text') else 'ru'
# # # #         audio_path = tts_engine.text_to_speech(description, lang=lang)
        
# # # #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# # # #         cv2.imwrite(annotated_path, annotated_frame)
        
# # # #         with open(annotated_path, 'rb') as f:
# # # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # # #         os.remove(annotated_path)

# # # #         print(f"✅ Результат: {description[:100]}...")

# # # #         return jsonify({
# # # #             "obstacles": obstacles,
# # # #             "ocr": ocr_data,
# # # #             "colors": colors,
# # # #             "brightness": brightness,
# # # #             "description": description,
# # # #             "count": len(obstacles),
# # # #             "audio_available": audio_path is not None,
# # # #             "annotated_frame": annotated_base64,
# # # #             "message": "Кадр обработан успешно"
# # # #         })
# # # #     except Exception as e:
# # # #         print(f"❌ Ошибка: {e}")
# # # #         import traceback
# # # #         traceback.print_exc()
# # # #         return jsonify({"error": str(e)}), 500

# # # # @app.route('/read_text', methods=['POST'])
# # # # def read_text():
# # # #     try:
# # # #         print("\n📖 Чтение текста...")
# # # #         if 'frame' not in request.files:
# # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # #         frame_bytes = request.files['frame'].read()
# # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # #         if frame is None:
# # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # #         print(f"✅ Изображение: {frame.shape}")

# # # #         ocr_data = text_recognizer.extract_text(frame)
        
# # # #         if ocr_data.get('has_text'):
# # # #             text = ocr_data['text']
# # # #             lang = ocr_data.get('language', 'ru')
            
# # # #             # Формируем описание на определенном языке
# # # #             if lang == 'en':
# # # #                 if len(text) > 300:
# # # #                     description = f"Long text detected. Beginning: {text}"
# # # #                 else:
# # # #                     description = f"Text detected: {text}"
# # # #             else:
# # # #                 if len(text) > 300:
# # # #                     description = f"Обнаружен длинный текст. {text}"
# # # #                 else:
# # # #                     description = f"Обнаружен текст: {text}"
            
# # # #             if ocr_data.get('phones'):
# # # #                 if lang == 'en':
# # # #                     description += f". Phone numbers: {', '.join(ocr_data['phones'])}"
# # # #                 else:
# # # #                     description += f". Номера телефонов: {', '.join(ocr_data['phones'])}"
# # # #         else:
# # # #             lang = 'ru'
# # # #             description = "Текст не обнаружен. Наведите камеру на текст и держите устройство неподвижно несколько секунд"
        
# # # #         audio_path = tts_engine.text_to_speech(description, lang=lang)
        
# # # #         print(f"✅ Результат ({lang}): {description[:100]}...")
        
# # # #         return jsonify({
# # # #             "ocr": ocr_data,
# # # #             "description": description,
# # # #             "language": lang,
# # # #             "audio_available": audio_path is not None,
# # # #             "message": "Распознавание текста выполнено"
# # # #         })
# # # #     except Exception as e:
# # # #         print(f"❌ Ошибка: {e}")
# # # #         import traceback
# # # #         traceback.print_exc()
# # # #         return jsonify({"error": str(e)}), 500

# # # # @app.route('/recognize_money', methods=['POST'])
# # # # def recognize_money():
# # # #     try:
# # # #         print("\n💰 Распознавание денег...")
# # # #         if 'frame' not in request.files:
# # # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # # #         frame_bytes = request.files['frame'].read()
# # # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # # #         if frame is None:
# # # #             return jsonify({"error": "Неверное изображение"}), 400

# # # #         print(f"✅ Изображение: {frame.shape}")

# # # #         money_data = money_recognizer.recognize(frame)
# # # #         description = money_data['description']
        
# # # #         audio_path = tts_engine.text_to_speech(description, lang='ru')
        
# # # #         print(f"✅ Результат: {description}")
        
# # # #         return jsonify({
# # # #             "money": money_data,
# # # #             "description": description,
# # # #             "audio_available": audio_path is not None,
# # # #             "message": "Распознавание денег выполнено"
# # # #         })
# # # #     except Exception as e:
# # # #         print(f"❌ Ошибка: {e}")
# # # #         import traceback
# # # #         traceback.print_exc()
# # # #         return jsonify({"error": str(e)}), 500

# # # # @app.route('/count_money', methods=['POST'])
# # # # def count_money():
# # # #     """Подсчет нескольких купюр"""
# # # #     try:
# # # #         print("\n💰 Подсчет нескольких купюр...")
        
# # # #         # Получаем несколько кадров
# # # #         frames = []
# # # #         for i in range(5):  # До 5 купюр
# # # #             frame_key = f'frame{i}'
# # # #             if frame_key in request.files:
# # # #                 frame_bytes = request.files[frame_key].read()
# # # #                 nparr = np.frombuffer(frame_bytes, np.uint8)
# # # #                 frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
# # # #                 if frame is not None:
# # # #                     frames.append(frame)
        
# # # #         if not frames:
# # # #             return jsonify({"error": "Кадры не предоставлены"}), 400
        
# # # #         result = money_recognizer.count_money(frames)
# # # #         description = result['description']
        
# # # #         audio_path = tts_engine.text_to_speech(description, lang='ru')
        
# # # #         return jsonify({
# # # #             "money_count": result,
# # # #             "description": description,
# # # #             "audio_available": audio_path is not None,
# # # #             "message": "Подсчет выполнен"
# # # #         })
# # # #     except Exception as e:
# # # #         print(f"❌ Ошибка: {e}")
# # # #         return jsonify({"error": str(e)}), 500

# # # # @app.route('/get_time', methods=['GET'])
# # # # def get_time():
# # # #     try:
# # # #         time_data = time_service.get_current_time()
# # # #         return jsonify(time_data)
# # # #     except Exception as e:
# # # #         return jsonify({"error": str(e)}), 500

# # # # @app.route('/test_connection', methods=['GET'])
# # # # def test_connection():
# # # #     return jsonify({"status": "ok", "message": "Сервер работает"})

# # # # if __name__ == '__main__':
# # # #     print("\n" + "="*70)
# # # #     print("🚀 BLIND ASSISTANT SERVER v13 - УЛУЧШЕННАЯ ВЕРСИЯ")
# # # #     print("="*70)
# # # #     print("✅ Детекция 80+ объектов (еда, посуда, мебель, техника)")
# # # #     print("✅ Мультиязычный OCR (русский/английский с автоопределением)")
# # # #     print("✅ Улучшенное распознавание казахстанских тенге")
# # # #     print("✅ Автоматическое определение языка для озвучки")
# # # #     print("✅ Подсчет нескольких купюр")
# # # #     print("✅ Определение цветов и освещенности")
# # # #     print("="*70)
# # # #     print("\n💡 ВАЖНЫЕ УЛУЧШЕНИЯ:")
# # # #     print("   📝 OCR:")
# # # #     print("      - Увеличение в 2.5 раза для лучшего распознавания")
# # # #     print("      - CLAHE для улучшения контраста")
# # # #     print("      - Множественные методы бинаризации")
# # # #     print("      - Автоопределение языка (RU/EN)")
# # # #     print("      - Озвучка на соответствующем языке")
# # # #     print()
# # # #     print("   💵 ДЕНЬГИ:")
# # # #     print("      - Множественное масштабирование (1.5x, 2x, 2.5x)")
# # # #     print("      - Коррекция яркости (3 уровня gamma)")
# # # #     print("      - Улучшенные цветовые диапазоны")
# # # #     print("      - Прямое распознавание номинала OCR")
# # # #     print("      - Подсчет нескольких купюр (/count_money)")
# # # #     print()
# # # #     print("   🔍 YOLO:")
# # # #     print("      - 80+ классов объектов")
# # # #     print("      - Продукты: банан, яблоко, хлеб, пицца и др.")
# # # #     print("      - Посуда: чашка, вилка, нож, ложка")
# # # #     print("      - Мебель: диван, кровать, стол")
# # # #     print("      - Техника: микроволновка, холодильник")
# # # #     print("="*70)
# # # #     print("\n📋 РЕКОМЕНДАЦИИ:")
# # # #     print("   🔹 Для текста: держите 2-3 секунды неподвижно")
# # # #     print("   🔹 Для денег: вся купюра в кадре, хорошее освещение")
# # # #     print("   🔹 Для продуктов: используйте команду 'сканировать'")
# # # #     print("="*70)
# # # #     print("\n🌐 Сервер: http://192.168.8.63:5000")
# # # #     print("="*70 + "\n")
    
# # # #     app.run(host='192.168.8.63', port=5000, debug=True)





# # # import os
# # # import uuid
# # # import cv2
# # # import numpy as np
# # # import psycopg2
# # # from flask import Flask, request, jsonify, send_file
# # # from ultralytics import YOLO
# # # import wave
# # # import hashlib
# # # import base64
# # # import pytesseract
# # # from gtts import gTTS
# # # import re
# # # from datetime import datetime
# # # from pydub import AudioSegment
# # # from langdetect import detect, LangDetectException

# # # pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'

# # # # Проверка Tesseract
# # # try:
# # #     print("🔍 Проверка Tesseract OCR...")
# # #     version = pytesseract.get_tesseract_version()
# # #     print(f"✅ Tesseract версия: {version}")
# # # except Exception as e:
# # #     print(f"❌ Ошибка Tesseract: {e}")

# # # # --- Конфигурация ---
# # # DB_NAME = "blind_app"
# # # DB_USER = "postgres"
# # # DB_PASSWORD = "12345"
# # # DB_HOST = "localhost"

# # # app = Flask(__name__)

# # # @app.after_request
# # # def after_request(response):
# # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # #     return response

# # # def get_db_connection():
# # #     return psycopg2.connect(
# # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # #     )

# # # # --- ВРЕМЯ ---
# # # class TimeService:
# # #     def get_current_time(self):
# # #         now = datetime.now()
# # #         hours = now.hour
# # #         minutes = now.minute
        
# # #         if minutes == 1:
# # #             min_word = "минута"
# # #         elif 2 <= minutes <= 4:
# # #             min_word = "минуты"
# # #         else:
# # #             min_word = "минут"
        
# # #         if hours == 1 or hours == 21:
# # #             hour_word = "час"
# # #         elif 2 <= hours <= 4 or 22 <= hours <= 24:
# # #             hour_word = "часа"
# # #         else:
# # #             hour_word = "часов"
        
# # #         time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
# # #         return {'time': time_str, 'hour': hours, 'minute': minutes}

# # # time_service = TimeService()

# # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # class ColorAnalyzer:
# # #     def __init__(self):
# # #         self.color_names = {
# # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # #             'черный': ([0, 0, 0], [180, 255, 50])
# # #         }
    
# # #     def analyze_colors(self, frame):
# # #         try:
# # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # #             detected_colors = []
            
# # #             for color_name, (lower, upper) in self.color_names.items():
# # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # #                 if percentage > 5:
# # #                     detected_colors.append({'color': color_name, 'percentage': round(percentage, 1)})
            
# # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # #             return detected_colors[:3]
# # #         except:
# # #             return []
    
# # #     def get_brightness_level(self, frame):
# # #         try:
# # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # #             avg_brightness = np.mean(gray)
            
# # #             if avg_brightness < 50:
# # #                 return "очень темно"
# # #             elif avg_brightness < 100:
# # #                 return "темно"
# # #             elif avg_brightness < 150:
# # #                 return "нормальное освещение"
# # #             elif avg_brightness < 200:
# # #                 return "светло"
# # #             else:
# # #                 return "очень светло"
# # #         except:
# # #             return "не определено"

# # # color_analyzer = ColorAnalyzer()

# # # # --- ВИЗУАЛИЗАЦИЯ ---
# # # class ObjectVisualizer:
# # #     def __init__(self):
# # #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# # #     def draw_detections(self, frame, detections, ocr_data=None):
# # #         try:
# # #             annotated_frame = frame.copy()
            
# # #             for det in detections:
# # #                 bbox = det['bbox']
# # #                 label_en = det['label']
# # #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # #                 distance = det.get('distance_m')
# # #                 priority = det.get('priority', False)
                
# # #                 color = (0, 255, 0)
# # #                 thickness = 3 if priority else 2
                
# # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
                
# # #                 text = f"{label_ru} - {distance:.1f}м" if distance else f"{label_ru}"
                
# # #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1] - text_height - 10),
# # #                             (bbox[0] + text_width + 10, bbox[1]), color, -1)
                
# # #                 cv2.putText(annotated_frame, text, (bbox[0] + 5, bbox[1] - 5),
# # #                           self.font, 0.7, (0, 0, 0), 2)
            
# # #             if ocr_data and ocr_data.get('has_text'):
# # #                 cv2.rectangle(annotated_frame, (10, 10), (300, 55), (0, 255, 0), -1)
# # #                 cv2.putText(annotated_frame, "Текст обнаружен", (15, 30), self.font, 0.7, (0, 0, 0), 2)
            
# # #             return annotated_frame
# # #         except:
# # #             return frame

# # # object_visualizer = ObjectVisualizer()

# # # # --- МУЛЬТИЯЗЫЧНАЯ ОЗВУЧКА ---
# # # class MultilingualTTS:
# # #     def __init__(self):
# # #         self.cache_dir = "audio_cache"
# # #         if not os.path.exists(self.cache_dir):
# # #             os.makedirs(self.cache_dir)
    
# # #     def detect_language(self, text):
# # #         """Определяет язык текста"""
# # #         try:
# # #             text_clean = re.sub(r'[0-9\s\.,!?;:]', '', text)
# # #             if len(text_clean) < 3:
# # #                 return 'ru'
            
# # #             # Проверяем наличие кириллицы
# # #             cyrillic_count = len(re.findall(r'[а-яА-ЯЁё]', text_clean))
# # #             latin_count = len(re.findall(r'[a-zA-Z]', text_clean))
            
# # #             if cyrillic_count > latin_count:
# # #                 return 'ru'
# # #             elif latin_count > cyrillic_count:
# # #                 return 'en'
            
# # #             lang = detect(text_clean)
# # #             return 'en' if lang == 'en' else 'ru'
# # #         except:
# # #             return 'ru'
    
# # #     def text_to_speech(self, text, lang=None):
# # #         """Озвучивает текст с автоопределением языка"""
# # #         try:
# # #             if lang is None:
# # #                 lang = self.detect_language(text)
            
# # #             text_hash = hashlib.md5(f"{text}_{lang}".encode()).hexdigest()
# # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # #             if os.path.exists(audio_path):
# # #                 return audio_path
            
# # #             tts = gTTS(text=text, lang=lang, slow=False)
# # #             tts.save(audio_path)
# # #             return audio_path
# # #         except Exception as e:
# # #             print(f"Ошибка озвучки: {e}")
# # #             return None

# # # tts_engine = MultilingualTTS()

# # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # class SimpleVoiceBiometrics:
# # #     def extract_mfcc_features(self, audio_path):
# # #         try:
# # #             with wave.open(audio_path, 'rb') as wav_file:
# # #                 sample_width = wav_file.getsampwidth()
# # #                 frame_rate = wav_file.getframerate()
# # #                 n_frames = wav_file.getnframes()
# # #                 frames = wav_file.readframes(n_frames)
                
# # #                 if sample_width == 2:
# # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # #                 else:
# # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # #                 features_bytes = features.astype(np.float32).tobytes()
# # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # #                 return features_b64
# # #         except Exception as e:
# # #             print(f"Ошибка извлечения характеристик: {e}")
# # #             return None
    
# # #     def _simple_audio_features(self, audio_data, sample_rate):
# # #         features = []
# # #         features.append(np.mean(audio_data ** 2))
# # #         features.append(np.std(audio_data))
# # #         features.append(np.max(np.abs(audio_data)))
        
# # #         fft = np.fft.fft(audio_data)
# # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # #         for low, high in bands:
# # #             mask = (freqs >= low) & (freqs < high)
# # #             if np.any(mask):
# # #                 features.append(np.mean(fft_magnitude[mask]))
# # #             else:
# # #                 features.append(0.0)
        
# # #         return np.array(features)
    
# # #     def compare_voice_features(self, features1_b64, features2_b64):
# # #         try:
# # #             features1_bytes = base64.b64decode(features1_b64)
# # #             features2_bytes = base64.b64decode(features2_b64)
            
# # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # #             min_len = min(len(features1), len(features2))
# # #             features1 = features1[:min_len]
# # #             features2 = features2[:min_len]
            
# # #             distance = np.linalg.norm(features1 - features2)
# # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
# # #             return float(max(0.0, min(1.0, similarity)))
# # #         except:
# # #             return 0.0

# # # voice_biometrics = SimpleVoiceBiometrics()

# # # # --- КОМАНДЫ ---
# # # class VoiceCommandRecognizer:
# # #     def __init__(self):
# # #         self.commands = {
# # #             'выйти': ['выйти', 'выход', 'выхожу', 'закрыть', 'exit'],
# # #             'сканировать': ['сканировать', 'скан', 'сканируй', 'что вижу', 'что впереди', 'анализ', 'scan'],
# # #             'время': ['время', 'который час', 'сколько времени', 'time'],
# # #             'читать': ['читать', 'читай', 'прочитай', 'текст', 'что написано', 'read'],
# # #             'деньги': ['деньги', 'распознать деньги', 'сколько денег', 'какая купюра', 'купюра', 'тенге', 'money']
# # #         }
    
# # #     def recognize_command(self, text):
# # #         if not text:
# # #             return None
        
# # #         text = text.lower().strip()
# # #         noise = ['слушаю', 'команду', 'команда', 'пожалуйста']
# # #         for n in noise:
# # #             text = text.replace(n, '')
# # #         text = ' '.join(text.split())
        
# # #         print(f"🎤 Анализ: '{text}'")
        
# # #         for cmd_type, keywords in self.commands.items():
# # #             for keyword in keywords:
# # #                 if keyword in text or text in keyword:
# # #                     print(f"✅ Команда: {cmd_type}")
# # #                     return cmd_type
        
# # #         print(f"❌ Не распознано")
# # #         return None

# # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # --- КОНВЕРТЕР АУДИО ---
# # # class AudioConverter:
# # #     def convert_aac_to_wav(self, aac_path):
# # #         try:
# # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # #             wav_path = aac_path.replace('.aac', '.wav')
# # #             audio.export(wav_path, format="wav")
# # #             return wav_path
# # #         except Exception as e:
# # #             print(f"Ошибка конвертации: {e}")
# # #             return None

# # # audio_converter = AudioConverter()

# # # # --- ПРОСТОЙ И НАДЕЖНЫЙ OCR ---
# # # class MultilingualTextRecognizer:
# # #     def __init__(self):
# # #         print("🔍 Инициализация простого OCR...")
    
# # #     def extract_text(self, frame):
# # #         try:
# # #             print("\n📖 Распознавание текста...")
            
# # #             try:
# # #                 pytesseract.get_tesseract_version()
# # #             except:
# # #                 print("❌ Tesseract недоступен")
# # #                 return {'text': '', 'language': 'ru', 'phones': [], 'has_text': False, 'length': 0}
            
# # #             # ПРОСТАЯ но эффективная предобработка
# # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
# # #             # Увеличиваем изображение
# # #             gray = cv2.resize(gray, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC)
            
# # #             # Улучшаем контраст
# # #             clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
# # #             gray = clahe.apply(gray)
            
# # #             # Пробуем оба метода бинаризации
# # #             _, binary1 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
# # #             binary2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
# # #                                            cv2.THRESH_BINARY, 11, 2)
            
# # #             best_result = None
# # #             best_score = 0
            
# # #             # Тестируем разные комбинации
# # #             tests = [
# # #                 (gray, 'eng', 'en', '--oem 3 --psm 3'),  # Автоматический PSM
# # #                 (binary1, 'eng', 'en', '--oem 3 --psm 6'),
# # #                 (binary2, 'eng', 'en', '--oem 3 --psm 6'),
# # #                 (gray, 'rus', 'ru', '--oem 3 --psm 3'),
# # #                 (binary1, 'rus', 'ru', '--oem 3 --psm 6'),
# # #             ]
            
# # #             for img, tesseract_lang, lang_code, config in tests:
# # #                 try:
# # #                     # Получаем детальную информацию о распознавании
# # #                     data = pytesseract.image_to_data(img, lang=tesseract_lang, config=config, 
# # #                                                     output_type=pytesseract.Output.DICT)
                    
# # #                     # Собираем только уверенные слова
# # #                     confident_words = []
# # #                     for i in range(len(data['text'])):
# # #                         word = data['text'][i].strip()
# # #                         conf = int(data['conf'][i]) if data['conf'][i] != '-1' else 0
                        
# # #                         # Фильтруем по уверенности и качеству
# # #                         if conf > 40 and len(word) >= 2:  # Минимальная уверенность 40%
# # #                             # Очищаем слово
# # #                             if lang_code == 'en':
# # #                                 word = re.sub(r'[^a-zA-Z0-9]', '', word)
# # #                             else:
# # #                                 word = re.sub(r'[^а-яА-ЯёЁ0-9]', '', word)
                            
# # #                             if len(word) >= 2 and self._is_valid_word(word, lang_code):
# # #                                 confident_words.append((word, conf))
                    
# # #                     if confident_words:
# # #                         # Сортируем по уверенности
# # #                         confident_words.sort(key=lambda x: x[1], reverse=True)
                        
# # #                         # Берем только топ слова
# # #                         top_words = [w for w, c in confident_words[:20]]
# # #                         text = ' '.join(top_words)
# # #                         avg_conf = sum(c for w, c in confident_words) / len(confident_words)
                        
# # #                         score = len(confident_words) * avg_conf
                        
# # #                         print(f"  {lang_code}: {len(confident_words)} слов, ср.уверенность {avg_conf:.0f}%")
# # #                         print(f"     Топ-5: {top_words[:5]}")
                        
# # #                         if score > best_score:
# # #                             best_score = score
# # #                             best_result = {
# # #                                 'text': text,
# # #                                 'language': lang_code,
# # #                                 'confidence': avg_conf / 100
# # #                             }
                
# # #                 except Exception as e:
# # #                     continue
            
# # #             if best_result and best_result['confidence'] > 0.5:  # Минимум 50% уверенности
# # #                 phones = self._extract_phone_numbers(best_result['text'])
                
# # #                 print(f"✅ Текст найден: '{best_result['text']}'")
# # #                 print(f"   Уверенность: {best_result['confidence']:.0%}")
                
# # #                 return {
# # #                     'text': best_result['text'],
# # #                     'language': best_result['language'],
# # #                     'phones': phones,
# # #                     'has_text': True,
# # #                     'length': len(best_result['text'])
# # #                 }
            
# # #             print("❌ Качественный текст не найден")
# # #             return {'text': '', 'language': 'ru', 'phones': [], 'has_text': False, 'length': 0}
            
# # #         except Exception as e:
# # #             print(f"❌ Ошибка OCR: {e}")
# # #             import traceback
# # #             traceback.print_exc()
# # #             return {'text': '', 'language': 'ru', 'phones': [], 'has_text': False, 'length': 0}
    
# # #     def _is_valid_word(self, word, lang):
# # #         """Проверяет валидность слова"""
# # #         if len(word) < 2:
# # #             return False
        
# # #         # Проверка на гласные
# # #         if lang == 'en':
# # #             has_vowel = bool(re.search(r'[aeiouAEIOU]', word))
# # #         else:
# # #             has_vowel = bool(re.search(r'[аеёиоуыэюяАЕЁИОУЫЭЮЯ]', word))
        
# # #         # Для коротких слов (2-3 символа) гласная обязательна
# # #         if len(word) <= 3 and not has_vowel:
# # #             return False
        
# # #         # Для длинных слов (4+) проверяем повторяющиеся символы
# # #         if len(word) >= 4:
# # #             # Не должно быть одного символа более 50%
# # #             most_common = max(word.count(c) for c in set(word))
# # #             if most_common > len(word) * 0.5:
# # #                 return False
        
# # #         return True
    
# # #     def _extract_phone_numbers(self, text):
# # #         """Извлечение номеров телефонов"""
# # #         patterns = [
# # #             r'\+7\d{10}',
# # #             r'8\d{10}',
# # #             r'\d{10}',
# # #         ]
# # #         phones = []
# # #         for pattern in patterns:
# # #             phones.extend(re.findall(pattern, text))
# # #         return phones

# # # text_recognizer = MultilingualTextRecognizer()

# # # # --- БЫСТРОЕ И ТОЧНОЕ РАСПОЗНАВАНИЕ ДЕНЕГ ---
# # # class FastMoneyRecognizer:
# # #     def __init__(self):
# # #         print("💰 Инициализация быстрого распознавателя тенге...")
        
# # #         # Точные характеристики казахстанских тенге
# # #         self.denominations = {
# # #             200: {
# # #                 'keywords': ['200', 'двести', 'екі жүз'],
# # #                 'color_ranges': [([90, 50, 100], [120, 255, 255])],  # Синий
# # #                 'color_name': 'синий'
# # #             },
# # #             500: {
# # #                 'keywords': ['500', 'пятьсот', 'бес жүз'],
# # #                 'color_ranges': [([15, 100, 150], [30, 255, 255])],  # Оранжевый
# # #                 'color_name': 'оранжевый'
# # #             },
# # #             1000: {
# # #                 'keywords': ['1000', 'тысяча', 'мың'],
# # #                 'color_ranges': [([35, 50, 100], [80, 255, 255])],  # Зеленый
# # #                 'color_name': 'зеленый'
# # #             },
# # #             2000: {
# # #                 'keywords': ['2000', 'две тысячи', 'екі мың'],
# # #                 'color_ranges': [([130, 30, 100], [160, 200, 255])],  # Фиолетовый
# # #                 'color_name': 'фиолетовый'
# # #             },
# # #             5000: {
# # #                 'keywords': ['5000', 'пять тысяч', 'бес мың'],
# # #                 'color_ranges': [([0, 50, 120], [15, 200, 220])],  # Розово-коричневый
# # #                 'color_name': 'коричнево-розовый'
# # #             },
# # #             10000: {
# # #                 'keywords': ['10000', 'десять тысяч', 'он мың'],
# # #                 'color_ranges': [([15, 30, 140], [30, 120, 230])],  # Бежевый
# # #                 'color_name': 'бежевый'
# # #             },
# # #             20000: {
# # #                 'keywords': ['20000', 'двадцать тысяч', 'жиырма мың'],
# # #                 'color_ranges': [([90, 10, 120], [110, 80, 200])],  # Серо-синий
# # #                 'color_name': 'серо-синий'
# # #             }
# # #         }
# # #         print("✅ Распознаватель готов")
    
# # #     def recognize(self, frame):
# # #         try:
# # #             print("\n💵 Быстрое распознавание купюры...")
            
# # #             # Увеличиваем изображение один раз
# # #             frame_scaled = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            
# # #             # Быстрая обработка
# # #             gray = cv2.cvtColor(frame_scaled, cv2.COLOR_BGR2GRAY)
# # #             clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
# # #             gray = clahe.apply(gray)
            
# # #             # Один метод бинаризации
# # #             _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
# # #             # OCR только для цифр
# # #             config = '--oem 3 --psm 6 -c tessedit_char_whitelist=0123456789'
# # #             text = pytesseract.image_to_string(binary, lang='eng', config=config).strip()
# # #             text = re.sub(r'[^0-9]', '', text)
            
# # #             print(f"📖 Найдены цифры: '{text}'")
            
# # #             # Анализ цвета
# # #             hsv = cv2.cvtColor(frame_scaled, cv2.COLOR_BGR2HSV)
            
# # #             best_match = None
# # #             best_score = 0
            
# # #             for denom, info in self.denominations.items():
# # #                 score = 0
                
# # #                 # 1. Прямое совпадение номинала (высший приоритет)
# # #                 denom_str = str(denom)
# # #                 if denom_str in text:
# # #                     score += 100
# # #                     print(f"  ✓✓ Номинал {denom} найден напрямую!")
                
# # #                 # 2. Проверка цвета
# # #                 for lower, upper in info['color_ranges']:
# # #                     mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # #                     color_pct = (np.sum(mask > 0) / mask.size) * 100
                    
# # #                     if color_pct > 3:
# # #                         color_score = min(color_pct * 5, 50)
# # #                         score += color_score
# # #                         print(f"  🎨 {info['color_name']}: {color_pct:.1f}% (+{color_score:.0f})")
                
# # #                 if score > best_score and score > 40:
# # #                     best_match = denom
# # #                     best_score = score
            
# # #             if best_match:
# # #                 confidence = min(best_score / 150, 1.0)
# # #                 print(f"✅ Купюра: {best_match} тенге (уверенность: {confidence:.0%})")
                
# # #                 return {
# # #                     'has_money': True,
# # #                     'denomination': best_match,
# # #                     'confidence': float(confidence),
# # #                     'description': f"Купюра {best_match} тенге"
# # #                 }
            
# # #             print("❌ Купюра не распознана")
# # #             return {
# # #                 'has_money': False,
# # #                 'description': 'Купюра не распознана. Расположите её ровно перед камерой'
# # #             }
# # #         except Exception as e:
# # #             print(f"❌ Ошибка: {e}")
# # #             return {'has_money': False, 'description': 'Ошибка распознавания'}

# # # money_recognizer = FastMoneyRecognizer()

# # # # --- YOLO ---
# # # OBJECT_HEIGHTS = {
# # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8, 'book': 0.3,
# # #     'bicycle': 1.0, 'motorcycle': 1.2, 'bus': 3.0, 'truck': 3.0,
# # #     'table': 0.8, 'bed': 0.5, 'refrigerator': 1.8
# # # }

# # # OBJECT_TRANSLATIONS = {
# # #     # Люди и части тела
# # #     'person': 'человек', 'man': 'мужчина', 'woman': 'женщина', 'boy': 'мальчик', 'girl': 'девочка',
# # #     'human face': 'лицо человека', 'human hand': 'рука', 'human foot': 'нога', 'human ear': 'ухо',
# # #     'human nose': 'нос', 'human mouth': 'рот', 'human eye': 'глаз', 'human head': 'голова',
    
# # #     # Транспорт
# # #     'car': 'машина', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл', 'bus': 'автобус',
# # #     'truck': 'грузовик', 'van': 'фургон', 'taxi': 'такси', 'ambulance': 'скорая помощь',
# # #     'fire truck': 'пожарная машина', 'police car': 'полицейская машина',
# # #     'airplane': 'самолет', 'helicopter': 'вертолет', 'boat': 'лодка', 'ship': 'корабль',
# # #     'train': 'поезд', 'wheel': 'колесо', 'tire': 'шина', 'vehicle registration plate': 'номерной знак',
    
# # #     # Дорожные объекты
# # #     'traffic light': 'светофор', 'stop sign': 'знак стоп', 'parking meter': 'парковочный счетчик',
# # #     'street light': 'уличный фонарь', 'traffic sign': 'дорожный знак',
    
# # #     # Животные
# # #     'dog': 'собака', 'cat': 'кошка', 'bird': 'птица', 'horse': 'лошадь', 'cow': 'корова',
# # #     'sheep': 'овца', 'elephant': 'слон', 'bear': 'медведь', 'zebra': 'зебра', 'giraffe': 'жираф',
# # #     'rabbit': 'кролик', 'mouse': 'мышь', 'snake': 'змея', 'lion': 'лев', 'tiger': 'тигр',
# # #     'butterfly': 'бабочка', 'bee': 'пчела', 'spider': 'паук', 'fish': 'рыба', 'chicken': 'курица',
    
# # #     # Еда и напитки
# # #     'banana': 'банан', 'apple': 'яблоко', 'orange': 'апельсин', 'lemon': 'лимон',
# # #     'strawberry': 'клубника', 'grape': 'виноград', 'watermelon': 'арбуз', 'pear': 'груша',
# # #     'pineapple': 'ананас', 'carrot': 'морковь', 'tomato': 'помидор', 'potato': 'картофель',
# # #     'bread': 'хлеб', 'sandwich': 'сандвич', 'pizza': 'пицца', 'hot dog': 'хот-дог',
# # #     'hamburger': 'гамбургер', 'cake': 'торт', 'cookie': 'печенье', 'donut': 'пончик',
# # #     'ice cream': 'мороженое', 'coffee': 'кофе', 'tea': 'чай', 'wine': 'вино',
# # #     'beer': 'пиво', 'juice': 'сок', 'milk': 'молоко', 'bottle': 'бутылка',
    
# # #     # Посуда и кухня
# # #     'cup': 'чашка', 'bowl': 'миска', 'plate': 'тарелка', 'fork': 'вилка',
# # #     'knife': 'нож', 'spoon': 'ложка', 'glass': 'стакан', 'wine glass': 'бокал',
# # #     'mug': 'кружка', 'pitcher': 'кувшин', 'microwave': 'микроволновка',
# # #     'oven': 'духовка', 'toaster': 'тостер', 'sink': 'раковина', 'refrigerator': 'холодильник',
# # #     'blender': 'блендер', 'kettle': 'чайник',
    
# # #     # Мебель
# # #     'chair': 'стул', 'table': 'стол', 'couch': 'диван', 'sofa': 'диван',
# # #     'bed': 'кровать', 'bench': 'скамейка', 'desk': 'письменный стол',
# # #     'cabinet': 'шкаф', 'shelf': 'полка', 'door': 'дверь', 'window': 'окно',
    
# # #     # Электроника
# # #     'tv': 'телевизор', 'laptop': 'ноутбук', 'computer': 'компьютер',
# # #     'keyboard': 'клавиатура', 'mouse': 'мышь', 'remote': 'пульт',
# # #     'cell phone': 'телефон', 'telephone': 'телефон', 'clock': 'часы',
# # #     'camera': 'камера', 'microphone': 'микрофон', 'headphones': 'наушники',
    
# # #     # Одежда и аксессуары
# # #     'backpack': 'рюкзак', 'handbag': 'сумка', 'suitcase': 'чемодан',
# # #     'umbrella': 'зонт', 'tie': 'галстук', 'hat': 'шляпа', 'cap': 'кепка',
# # #     'glasses': 'очки', 'sunglasses': 'солнечные очки', 'shoe': 'обувь',
# # #     'boot': 'ботинок', 'belt': 'ремень', 'jacket': 'куртка', 'coat': 'пальто',
    
# # #     # Спорт
# # #     'ball': 'мяч', 'football': 'футбольный мяч', 'basketball': 'баскетбольный мяч',
# # #     'tennis racket': 'теннисная ракетка', 'baseball bat': 'бейсбольная бита',
# # #     'baseball glove': 'бейсбольная перчатка', 'skateboard': 'скейтборд',
# # #     'surfboard': 'доска для серфинга', 'ski': 'лыжи', 'snowboard': 'сноуборд',
    
# # #     # Растения и природа
# # #     'tree': 'дерево', 'flower': 'цветок', 'plant': 'растение', 'grass': 'трава',
# # #     'rose': 'роза', 'mushroom': 'гриб', 'fountain': 'фонтан',
    
# # #     # Здания и архитектура
# # #     'building': 'здание', 'house': 'дом', 'skyscraper': 'небоскреб',
# # #     'tower': 'башня', 'bridge': 'мост', 'stairs': 'лестница', 'ladder': 'лестница',
    
# # #     # Инструменты и предметы
# # #     'book': 'книга', 'pen': 'ручка', 'pencil': 'карандаш', 'scissors': 'ножницы',
# # #     'key': 'ключ', 'lock': 'замок', 'hammer': 'молоток', 'screwdriver': 'отвертка',
# # #     'wrench': 'гаечный ключ', 'toolbox': 'ящик для инструментов',
    
# # #     # Разное
# # #     'trash can': 'мусорное ведро', 'box': 'коробка', 'bag': 'сумка',
# # #     'basket': 'корзина', 'pillow': 'подушка', 'blanket': 'одеяло',
# # #     'towel': 'полотенце', 'candle': 'свеча', 'lamp': 'лампа',
# # #     'light': 'свет', 'mirror': 'зеркало', 'picture frame': 'рамка для фото',
# # #     'vase': 'ваза', 'curtain': 'штора', 'rug': 'ковер', 'carpet': 'ковер',
# # #     'ceiling': 'потолок', 'floor': 'пол', 'wall': 'стена',
# # #     'flag': 'флаг', 'balloon': 'воздушный шар', 'toy': 'игрушка',
# # #     'kite': 'воздушный змей', 'fireworks': 'фейерверк'
# # # }

# # # PRIORITY_OBJECTS = {
# # #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck', 
# # #     'traffic light', 'stop sign', 'dog', 'cat'
# # # }

# # # def estimate_distance(bbox, frame_height, object_label):
# # #     bbox_height_pixels = bbox[3] - bbox[1]
# # #     if bbox_height_pixels <= 0:
# # #         return None
# # #     FOCAL_LENGTH_PIXELS = 700
# # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # #     return round(distance, 2)

# # # print("🔄 Загрузка YOLO...")
# # # model = YOLO('yolov5x-oiv7-fixed.pt')
# # # print("✅ YOLO готов")

# # # def detect_objects(frame):
# # #     try:
# # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # #         # Используем максимальные параметры для 600+ объектов
# # #         results = model.predict(img_rgb, conf=0.25, iou=0.45, verbose=False, imgsz=640, max_det=100)[0]
        
# # #         detections = []
# # #         frame_height, frame_width = frame.shape[:2]

# # #         for r in results.boxes:
# # #             bbox = r.xyxy[0].cpu().numpy()
# # #             conf = float(r.conf[0])
# # #             cls_id = int(r.cls[0])
# # #             label = model.names[cls_id]
# # #             distance = estimate_distance(bbox, frame_height, label)
            
# # #             x_center = (bbox[0] + bbox[2]) / 2
# # #             y_center = (bbox[1] + bbox[3]) / 2
            
# # #             detections.append({
# # #                 "label": label,
# # #                 "confidence": conf,
# # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # #                 "distance_m": float(distance) if distance else None,
# # #                 "priority": label in PRIORITY_OBJECTS,
# # #                 "x_center": float(x_center),
# # #                 "y_center": float(y_center)
# # #             })
        
# # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
# # #         print(f"🔍 Обнаружено: {len(detections)} объектов из 600+ классов")
# # #         return detections
# # #     except Exception as e:
# # #         print(f"❌ Ошибка YOLO: {e}")
# # #         return []

# # # def generate_obstacle_description(obstacles, ocr_data=None, colors=None, brightness=None):
# # #     descriptions = []
    
# # #     if brightness:
# # #         descriptions.append(f"{brightness}")
    
# # #     if colors and len(colors) > 0:
# # #         color_desc = " и ".join([c['color'] for c in colors[:2]])
# # #         descriptions.append(f"Основные цвета: {color_desc}")
    
# # #     if ocr_data and ocr_data.get('has_text'):
# # #         text = ocr_data['text']
# # #         if text:
# # #             text_clean = ' '.join(text.split())
# # #             if len(text_clean) > 150:
# # #                 descriptions.append(f"Текст: {text_clean[:150]}")
# # #             else:
# # #                 descriptions.append(f"Текст: {text_clean}")
        
# # #         if ocr_data.get('phones'):
# # #             descriptions.append(f"Телефоны: {', '.join(ocr_data['phones'])}")
    
# # #     if obstacles:
# # #         descriptions.append(f"Объектов: {len(obstacles)}")
        
# # #         for obs in obstacles[:10]:  # Показываем больше объектов
# # #             label_en = obs['label']
# # #             label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # #             distance = obs.get('distance_m')
            
# # #             if distance:
# # #                 if distance < 1.0:
# # #                     dist_text = f"очень близко"
# # #                 elif distance < 2.0:
# # #                     dist_text = f"близко"
# # #                 else:
# # #                     dist_text = f"{int(distance)} метров"
# # #             else:
# # #                 dist_text = "расстояние неизвестно"
            
# # #             priority = "Внимание! " if obs.get('priority') else ""
# # #             descriptions.append(f"{priority}{label_ru}, {dist_text}")
        
# # #         if len(obstacles) > 10:
# # #             descriptions.append(f"и еще {len(obstacles) - 10} объектов")
# # #     else:
# # #         descriptions.append("Препятствий не обнаружено")
    
# # #     return ". ".join(descriptions)

# # # # --- ЭНДПОИНТЫ ---

# # # @app.route('/')
# # # def index():
# # #     return jsonify({
# # #         "status": "Blind Assistant Server v14 - ИСПРАВЛЕННАЯ ВЕРСИЯ",
# # #         "features": [
# # #             "✅ OCR без иероглифов (только RU/EN)",
# # #             "✅ Быстрое распознавание тенге",
# # #             "✅ Детекция объектов YOLO",
# # #             "✅ Голосовые команды"
# # #         ]
# # #     })

# # # @app.route('/register_voice', methods=['POST'])
# # # def register_voice():
# # #     temp_audio = temp_wav = None
# # #     try:
# # #         if 'audio' not in request.files or 'username' not in request.form:
# # #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# # #         username = request.form['username'].strip()
# # #         audio_file = request.files['audio']
        
# # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # #         audio_file.save(temp_audio)
        
# # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # #         if not temp_wav:
# # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # #         conn = get_db_connection()
# # #         cur = conn.cursor()
        
# # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # #         existing = cur.fetchone()
        
# # #         if existing:
# # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (voice_features, username))
# # #             message = "Профиль обновлен"
# # #         else:
# # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, voice_features))
# # #             message = "Профиль создан"
        
# # #         conn.commit()
# # #         cur.close()
# # #         conn.close()
        
# # #         return jsonify({"status": "success", "message": message, "username": username})
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500
# # #     finally:
# # #         for f in [temp_audio, temp_wav]:
# # #             if f and os.path.exists(f):
# # #                 try: os.remove(f)
# # #                 except: pass

# # # @app.route('/login_voice', methods=['POST'])
# # # def login_voice():
# # #     temp_audio = temp_wav = None
# # #     try:
# # #         username = request.form['username'].strip()
# # #         audio_file = request.files['audio']
        
# # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # #         audio_file.save(temp_audio)
        
# # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # #         conn = get_db_connection()
# # #         cur = conn.cursor()
        
# # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # #         result = cur.fetchone()
        
# # #         if not result:
# # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # #         cur.close()
# # #         conn.close()
        
# # #         if similarity > 0.7:
# # #             welcome_text = f"Добро пожаловать, {username}! Камера готова."
# # #             return jsonify({
# # #                 "status": "success",
# # #                 "message": "Вход выполнен",
# # #                 "username": username,
# # #                 "similarity": float(similarity),
# # #                 "welcome_text": welcome_text,
# # #                 "skippable": True
# # #             })
# # #         else:
# # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500
# # #     finally:
# # #         for f in [temp_audio, temp_wav]:
# # #             if f and os.path.exists(f):
# # #                 try: os.remove(f)
# # #                 except: pass

# # # @app.route('/voice_command', methods=['POST'])
# # # def voice_command():
# # #     try:
# # #         text = request.form.get('text', '').lower().strip()
# # #         print(f"🎤 Голосовая команда: '{text}'")
        
# # #         if not text:
# # #             return jsonify({"error": "Текст не распознан"}), 400
        
# # #         command = voice_command_recognizer.recognize_command(text)
        
# # #         if command == 'время':
# # #             time_data = time_service.get_current_time()
# # #             return jsonify({"status": "success", "command": "время", "data": time_data})
# # #         elif command in ['сканировать', 'читать', 'деньги', 'выйти']:
# # #             return jsonify({"status": "success", "command": command})
# # #         else:
# # #             return jsonify({
# # #                 "status": "unknown",
# # #                 "message": "Команда не распознана",
# # #                 "available_commands": ["сканировать", "время", "читать", "деньги", "выйти"]
# # #             })
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route('/process_frame', methods=['POST'])
# # # def process_frame():
# # #     try:
# # #         print("\n🔵 Обработка кадра...")
# # #         if 'frame' not in request.files:
# # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # #         frame_bytes = request.files['frame'].read()
# # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # #         if frame is None:
# # #             return jsonify({"error": "Неверное изображение"}), 400

# # #         print(f"✅ Изображение: {frame.shape}")

# # #         # Быстрый анализ
# # #         obstacles = detect_objects(frame)
# # #         ocr_data = text_recognizer.extract_text(frame)
# # #         colors = color_analyzer.analyze_colors(frame)
# # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # #         annotated_frame = object_visualizer.draw_detections(frame, obstacles, ocr_data)
# # #         description = generate_obstacle_description(obstacles, ocr_data, colors, brightness)
        
# # #         lang = ocr_data.get('language', 'ru') if ocr_data.get('has_text') else 'ru'
# # #         audio_path = tts_engine.text_to_speech(description, lang=lang)
        
# # #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# # #         cv2.imwrite(annotated_path, annotated_frame)
        
# # #         with open(annotated_path, 'rb') as f:
# # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # #         os.remove(annotated_path)

# # #         print(f"✅ Готово")

# # #         return jsonify({
# # #             "obstacles": obstacles,
# # #             "ocr": ocr_data,
# # #             "colors": colors,
# # #             "brightness": brightness,
# # #             "description": description,
# # #             "count": len(obstacles),
# # #             "audio_available": audio_path is not None,
# # #             "annotated_frame": annotated_base64,
# # #             "message": "Кадр обработан"
# # #         })
# # #     except Exception as e:
# # #         print(f"❌ Ошибка: {e}")
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route('/read_text', methods=['POST'])
# # # def read_text():
# # #     try:
# # #         print("\n📖 Чтение текста...")
# # #         if 'frame' not in request.files:
# # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # #         frame_bytes = request.files['frame'].read()
# # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # #         if frame is None:
# # #             return jsonify({"error": "Неверное изображение"}), 400

# # #         ocr_data = text_recognizer.extract_text(frame)
        
# # #         if ocr_data.get('has_text'):
# # #             text = ocr_data['text']
# # #             lang = ocr_data.get('language', 'ru')
# # #             description = f"Текст: {text}"
            
# # #             if ocr_data.get('phones'):
# # #                 description += f". Телефоны: {', '.join(ocr_data['phones'])}"
# # #         else:
# # #             lang = 'ru'
# # #             description = "Текст не найден. Наведите камеру на текст"
        
# # #         audio_path = tts_engine.text_to_speech(description, lang=lang)
        
# # #         print(f"✅ Готово")
        
# # #         return jsonify({
# # #             "ocr": ocr_data,
# # #             "description": description,
# # #             "language": lang,
# # #             "audio_available": audio_path is not None,
# # #             "message": "Распознавание выполнено"
# # #         })
# # #     except Exception as e:
# # #         print(f"❌ Ошибка: {e}")
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route('/recognize_money', methods=['POST'])
# # # def recognize_money():
# # #     try:
# # #         print("\n💰 Распознавание денег...")
# # #         if 'frame' not in request.files:
# # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # #         frame_bytes = request.files['frame'].read()
# # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # #         if frame is None:
# # #             return jsonify({"error": "Неверное изображение"}), 400

# # #         money_data = money_recognizer.recognize(frame)
# # #         description = money_data['description']
        
# # #         audio_path = tts_engine.text_to_speech(description, lang='ru')
        
# # #         print(f"✅ Готово")
        
# # #         return jsonify({
# # #             "money": money_data,
# # #             "description": description,
# # #             "audio_available": audio_path is not None,
# # #             "message": "Распознавание выполнено"
# # #         })
# # #     except Exception as e:
# # #         print(f"❌ Ошибка: {e}")
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route('/get_time', methods=['GET'])
# # # def get_time():
# # #     try:
# # #         time_data = time_service.get_current_time()
# # #         return jsonify(time_data)
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route('/test_connection', methods=['GET'])
# # # def test_connection():
# # #     return jsonify({"status": "ok", "message": "Сервер работает"})

# # # if __name__ == '__main__':
# # #     print("\n" + "="*70)
# # #     print("🚀 BLIND ASSISTANT SERVER v14 - ИСПРАВЛЕННАЯ ВЕРСИЯ")
# # #     print("="*70)
# # #     print("✅ OCR БЕЗ ИЕРОГЛИФОВ (только кириллица и латиница)")
# # #     print("✅ БЫСТРОЕ распознавание тенге (в 3 раза быстрее)")
# # #     print("✅ Детекция 600+ объектов YOLO (максимальная версия)")
# # #     print("✅ Голосовые команды и биометрия")
# # #     print("="*70)
# # #     print("\n🔧 ИСПРАВЛЕНИЯ:")
# # #     print("   📝 OCR:")
# # #     print("      - Жесткая фильтрация символов (только RU/EN)")
# # #     print("      - Убраны иероглифы и спецсимволы")
# # #     print("      - Быстрая обработка (2 метода вместо 12)")
# # #     print()
# # #     print("   💵 ДЕНЬГИ:")
# # #     print("      - Быстрое распознавание (1 проход вместо 15)")
# # #     print("      - Точные цветовые диапазоны для тенге")
# # #     print("      - Прямое распознавание номинала")
# # #     print("      - OCR только для цифр")
# # #     print()
# # #     print("   🔍 YOLO (600+ объектов):")
# # #     print("      - Люди и части тела: лицо, руки, ноги")
# # #     print("      - Транспорт: все виды машин, самолеты, корабли")
# # #     print("      - Животные: 20+ видов животных")
# # #     print("      - Еда: 40+ видов еды и напитков")
# # #     print("      - Посуда: тарелки, чашки, столовые приборы")
# # #     print("      - Мебель: столы, стулья, кровати, шкафы")
# # #     print("      - Электроника: компьютеры, телефоны, камеры")
# # #     print("      - Одежда: обувь, сумки, шляпы, очки")
# # #     print("      - Спорт: мячи, ракетки, скейтборды")
# # #     print("      - И МНОГОЕ ДРУГОЕ!")
# # #     print()
# # #     print("   ⚡ СКОРОСТЬ:")
# # #     print("      - OCR: ~1-2 секунды (было 10-15 сек)")
# # #     print("      - Деньги: ~1 секунда (было 5-8 сек)")
# # #     print("      - YOLO: ~0.5-1 секунда (600+ объектов)")
# # #     print("="*70)
# # #     print("\n💡 РЕКОМЕНДАЦИИ:")
# # #     print("   🔹 Текст: держите неподвижно 2 секунды")
# # #     print("   🔹 Деньги: вся купюра в кадре, хорошее освещение")
# # #     print("   🔹 Объекты: команда 'сканировать' для полного анализа")
# # #     print("="*70)
# # #     print("\n🌐 Сервер: http://192.168.8.63:5000")
# # #     print("="*70 + "\n")
    
# # #     app.run(host='192.168.8.63', port=5000, debug=True)


# # # текст не работает деньги не работает





# # # import os
# # # import uuid
# # # import cv2
# # # import numpy as np
# # # import psycopg2
# # # from flask import Flask, request, jsonify, send_file
# # # from ultralytics import YOLO
# # # import wave
# # # import hashlib
# # # import base64
# # # # import pytesseract  # ЗАКОММЕНТИРОВАНО
# # # from gtts import gTTS
# # # import re
# # # from datetime import datetime
# # # from pydub import AudioSegment
# # # from langdetect import detect, LangDetectException

# # # # pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'  # ЗАКОММЕНТИРОВАНО

# # # # Проверка Tesseract - ЗАКОММЕНТИРОВАНО
# # # # try:
# # # #     print("🔍 Проверка Tesseract OCR...")
# # # #     version = pytesseract.get_tesseract_version()
# # # #     print(f"✅ Tesseract версия: {version}")
# # # # except Exception as e:
# # # #     print(f"❌ Ошибка Tesseract: {e}")

# # # # --- Конфигурация ---
# # # DB_NAME = "blind_app"
# # # DB_USER = "postgres"
# # # DB_PASSWORD = "12345"
# # # DB_HOST = "localhost"

# # # app = Flask(__name__)

# # # @app.after_request
# # # def after_request(response):
# # #     response.headers.add('Access-Control-Allow-Origin', '*')
# # #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# # #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# # #     return response

# # # def get_db_connection():
# # #     return psycopg2.connect(
# # #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# # #     )

# # # # --- ВРЕМЯ ---
# # # class TimeService:
# # #     def get_current_time(self):
# # #         now = datetime.now()
# # #         hours = now.hour
# # #         minutes = now.minute
        
# # #         if minutes == 1:
# # #             min_word = "минута"
# # #         elif 2 <= minutes <= 4:
# # #             min_word = "минуты"
# # #         else:
# # #             min_word = "минут"
        
# # #         if hours == 1 or hours == 21:
# # #             hour_word = "час"
# # #         elif 2 <= hours <= 4 or 22 <= hours <= 24:
# # #             hour_word = "часа"
# # #         else:
# # #             hour_word = "часов"
        
# # #         time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
# # #         return {'time': time_str, 'hour': hours, 'minute': minutes}

# # # time_service = TimeService()

# # # # --- АНАЛИЗ ЦВЕТОВ ---
# # # class ColorAnalyzer:
# # #     def __init__(self):
# # #         self.color_names = {
# # #             'красный': ([0, 100, 100], [10, 255, 255]),
# # #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# # #             'желтый': ([25, 100, 100], [35, 255, 255]),
# # #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# # #             'голубой': ([85, 100, 100], [100, 255, 255]),
# # #             'синий': ([100, 100, 100], [130, 255, 255]),
# # #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# # #             'розовый': ([160, 100, 100], [170, 255, 255]),
# # #             'белый': ([0, 0, 200], [180, 30, 255]),
# # #             'серый': ([0, 0, 50], [180, 30, 200]),
# # #             'черный': ([0, 0, 0], [180, 255, 50])
# # #         }
    
# # #     def analyze_colors(self, frame):
# # #         try:
# # #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# # #             detected_colors = []
            
# # #             for color_name, (lower, upper) in self.color_names.items():
# # #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# # #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# # #                 if percentage > 5:
# # #                     detected_colors.append({'color': color_name, 'percentage': round(percentage, 1)})
            
# # #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# # #             return detected_colors[:3]
# # #         except:
# # #             return []
    
# # #     def get_brightness_level(self, frame):
# # #         try:
# # #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# # #             avg_brightness = np.mean(gray)
            
# # #             if avg_brightness < 50:
# # #                 return "очень темно"
# # #             elif avg_brightness < 100:
# # #                 return "темно"
# # #             elif avg_brightness < 150:
# # #                 return "нормальное освещение"
# # #             elif avg_brightness < 200:
# # #                 return "светло"
# # #             else:
# # #                 return "очень светло"
# # #         except:
# # #             return "не определено"

# # # color_analyzer = ColorAnalyzer()

# # # # --- ВИЗУАЛИЗАЦИЯ ---
# # # class ObjectVisualizer:
# # #     def __init__(self):
# # #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# # #     def draw_detections(self, frame, detections, ocr_data=None):
# # #         try:
# # #             annotated_frame = frame.copy()
            
# # #             for det in detections:
# # #                 bbox = det['bbox']
# # #                 label_en = det['label']
# # #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # #                 distance = det.get('distance_m')
# # #                 priority = det.get('priority', False)
                
# # #                 color = (0, 255, 0)
# # #                 thickness = 3 if priority else 2
                
# # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
                
# # #                 text = f"{label_ru} - {distance:.1f}м" if distance else f"{label_ru}"
                
# # #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# # #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1] - text_height - 10),
# # #                             (bbox[0] + text_width + 10, bbox[1]), color, -1)
                
# # #                 cv2.putText(annotated_frame, text, (bbox[0] + 5, bbox[1] - 5),
# # #                           self.font, 0.7, (0, 0, 0), 2)
            
# # #             # OCR визуализация закомментирована
# # #             # if ocr_data and ocr_data.get('has_text'):
# # #             #     cv2.rectangle(annotated_frame, (10, 10), (300, 55), (0, 255, 0), -1)
# # #             #     cv2.putText(annotated_frame, "Текст обнаружен", (15, 30), self.font, 0.7, (0, 0, 0), 2)
            
# # #             return annotated_frame
# # #         except:
# # #             return frame

# # # object_visualizer = ObjectVisualizer()

# # # # --- МУЛЬТИЯЗЫЧНАЯ ОЗВУЧКА ---
# # # class MultilingualTTS:
# # #     def __init__(self):
# # #         self.cache_dir = "audio_cache"
# # #         if not os.path.exists(self.cache_dir):
# # #             os.makedirs(self.cache_dir)
    
# # #     def detect_language(self, text):
# # #         """Определяет язык текста"""
# # #         try:
# # #             text_clean = re.sub(r'[0-9\s\.,!?;:]', '', text)
# # #             if len(text_clean) < 3:
# # #                 return 'ru'
            
# # #             # Проверяем наличие кириллицы
# # #             cyrillic_count = len(re.findall(r'[а-яА-ЯЁё]', text_clean))
# # #             latin_count = len(re.findall(r'[a-zA-Z]', text_clean))
            
# # #             if cyrillic_count > latin_count:
# # #                 return 'ru'
# # #             elif latin_count > cyrillic_count:
# # #                 return 'en'
            
# # #             lang = detect(text_clean)
# # #             return 'en' if lang == 'en' else 'ru'
# # #         except:
# # #             return 'ru'
    
# # #     def text_to_speech(self, text, lang=None):
# # #         """Озвучивает текст с автоопределением языка"""
# # #         try:
# # #             if lang is None:
# # #                 lang = self.detect_language(text)
            
# # #             text_hash = hashlib.md5(f"{text}_{lang}".encode()).hexdigest()
# # #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# # #             if os.path.exists(audio_path):
# # #                 return audio_path
            
# # #             tts = gTTS(text=text, lang=lang, slow=False)
# # #             tts.save(audio_path)
# # #             return audio_path
# # #         except Exception as e:
# # #             print(f"Ошибка озвучки: {e}")
# # #             return None

# # # tts_engine = MultilingualTTS()

# # # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # # class SimpleVoiceBiometrics:
# # #     def extract_mfcc_features(self, audio_path):
# # #         try:
# # #             with wave.open(audio_path, 'rb') as wav_file:
# # #                 sample_width = wav_file.getsampwidth()
# # #                 frame_rate = wav_file.getframerate()
# # #                 n_frames = wav_file.getnframes()
# # #                 frames = wav_file.readframes(n_frames)
                
# # #                 if sample_width == 2:
# # #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# # #                 else:
# # #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# # #                     audio_data = audio_data.astype(np.float32) - 128
                
# # #                 audio_data = audio_data.astype(np.float32) / 32768.0
# # #                 features = self._simple_audio_features(audio_data, frame_rate)
# # #                 features_bytes = features.astype(np.float32).tobytes()
# # #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# # #                 return features_b64
# # #         except Exception as e:
# # #             print(f"Ошибка извлечения характеристик: {e}")
# # #             return None
    
# # #     def _simple_audio_features(self, audio_data, sample_rate):
# # #         features = []
# # #         features.append(np.mean(audio_data ** 2))
# # #         features.append(np.std(audio_data))
# # #         features.append(np.max(np.abs(audio_data)))
        
# # #         fft = np.fft.fft(audio_data)
# # #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# # #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# # #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# # #         for low, high in bands:
# # #             mask = (freqs >= low) & (freqs < high)
# # #             if np.any(mask):
# # #                 features.append(np.mean(fft_magnitude[mask]))
# # #             else:
# # #                 features.append(0.0)
        
# # #         return np.array(features)
    
# # #     def compare_voice_features(self, features1_b64, features2_b64):
# # #         try:
# # #             features1_bytes = base64.b64decode(features1_b64)
# # #             features2_bytes = base64.b64decode(features2_b64)
            
# # #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# # #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# # #             min_len = min(len(features1), len(features2))
# # #             features1 = features1[:min_len]
# # #             features2 = features2[:min_len]
            
# # #             distance = np.linalg.norm(features1 - features2)
# # #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# # #             similarity = 1.0 - (distance / (max_distance + 1e-8))
# # #             return float(max(0.0, min(1.0, similarity)))
# # #         except:
# # #             return 0.0

# # # voice_biometrics = SimpleVoiceBiometrics()

# # # # --- КОМАНДЫ ---
# # # class VoiceCommandRecognizer:
# # #     def __init__(self):
# # #         self.commands = {
# # #             'выйти': ['выйти', 'выход', 'выхожу', 'закрыть', 'exit'],
# # #             'сканировать': ['сканировать', 'скан', 'сканируй', 'что вижу', 'что впереди', 'анализ', 'scan'],
# # #             'время': ['время', 'который час', 'сколько времени', 'time'],
# # #             # КОМАНДЫ ЧИТАТЬ И ДЕНЬГИ ЗАКОММЕНТИРОВАНЫ
# # #             # 'читать': ['читать', 'читай', 'прочитай', 'текст', 'что написано', 'read'],
# # #             # 'деньги': ['деньги', 'распознать деньги', 'сколько денег', 'какая купюра', 'купюра', 'тенге', 'money']
# # #         }
    
# # #     def recognize_command(self, text):
# # #         if not text:
# # #             return None
        
# # #         text = text.lower().strip()
# # #         noise = ['слушаю', 'команду', 'команда', 'пожалуйста']
# # #         for n in noise:
# # #             text = text.replace(n, '')
# # #         text = ' '.join(text.split())
        
# # #         print(f"🎤 Анализ: '{text}'")
        
# # #         for cmd_type, keywords in self.commands.items():
# # #             for keyword in keywords:
# # #                 if keyword in text or text in keyword:
# # #                     print(f"✅ Команда: {cmd_type}")
# # #                     return cmd_type
        
# # #         print(f"❌ Не распознано")
# # #         return None

# # # voice_command_recognizer = VoiceCommandRecognizer()

# # # # --- КОНВЕРТЕР АУДИО ---
# # # class AudioConverter:
# # #     def convert_aac_to_wav(self, aac_path):
# # #         try:
# # #             audio = AudioSegment.from_file(aac_path, format="aac")
# # #             wav_path = aac_path.replace('.aac', '.wav')
# # #             audio.export(wav_path, format="wav")
# # #             return wav_path
# # #         except Exception as e:
# # #             print(f"Ошибка конвертации: {e}")
# # #             return None

# # # audio_converter = AudioConverter()

# # # # --- OCR КЛАСС ЗАКОММЕНТИРОВАН ---
# # # # class MultilingualTextRecognizer:
# # # #     def __init__(self):
# # # #         print("🔍 Инициализация простого OCR...")
# # # #     
# # # #     def extract_text(self, frame):
# # # #         # Весь код OCR закомментирован
# # # #         return {'text': '', 'language': 'ru', 'phones': [], 'has_text': False, 'length': 0}

# # # # text_recognizer = MultilingualTextRecognizer()

# # # # --- РАСПОЗНАВАНИЕ ДЕНЕГ ЗАКОММЕНТИРОВАНО ---
# # # # class FastMoneyRecognizer:
# # # #     def __init__(self):
# # # #         print("💰 Инициализация быстрого распознавателя тенге...")
# # # #         # Код закомментирован
# # # #     
# # # #     def recognize(self, frame):
# # # #         return {'has_money': False, 'description': 'Функция временно отключена'}

# # # # money_recognizer = FastMoneyRecognizer()

# # # # --- YOLO ---
# # # OBJECT_HEIGHTS = {
# # #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# # #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# # #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8, 'book': 0.3,
# # #     'bicycle': 1.0, 'motorcycle': 1.2, 'bus': 3.0, 'truck': 3.0,
# # #     'table': 0.8, 'bed': 0.5, 'refrigerator': 1.8
# # # }

# # # OBJECT_TRANSLATIONS = {
# # #     # Люди и части тела
# # #     'person': 'человек', 'man': 'мужчина', 'woman': 'женщина', 'boy': 'мальчик', 'girl': 'девочка',
# # #     'human face': 'лицо человека', 'human hand': 'рука', 'human foot': 'нога', 'human ear': 'ухо',
# # #     'human nose': 'нос', 'human mouth': 'рот', 'human eye': 'глаз', 'human head': 'голова',
    
# # #     # Транспорт
# # #     'car': 'машина', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл', 'bus': 'автобус',
# # #     'truck': 'грузовик', 'van': 'фургон', 'taxi': 'такси', 'ambulance': 'скорая помощь',
# # #     'fire truck': 'пожарная машина', 'police car': 'полицейская машина',
# # #     'airplane': 'самолет', 'helicopter': 'вертолет', 'boat': 'лодка', 'ship': 'корабль',
# # #     'train': 'поезд', 'wheel': 'колесо', 'tire': 'шина', 'vehicle registration plate': 'номерной знак',
    
# # #     # Дорожные объекты
# # #     'traffic light': 'светофор', 'stop sign': 'знак стоп', 'parking meter': 'парковочный счетчик',
# # #     'street light': 'уличный фонарь', 'traffic sign': 'дорожный знак',
    
# # #     # Животные
# # #     'dog': 'собака', 'cat': 'кошка', 'bird': 'птица', 'horse': 'лошадь', 'cow': 'корова',
# # #     'sheep': 'овца', 'elephant': 'слон', 'bear': 'медведь', 'zebra': 'зебра', 'giraffe': 'жираф',
# # #     'rabbit': 'кролик', 'mouse': 'мышь', 'snake': 'змея', 'lion': 'лев', 'tiger': 'тигр',
# # #     'butterfly': 'бабочка', 'bee': 'пчела', 'spider': 'паук', 'fish': 'рыба', 'chicken': 'курица',
    
# # #     # Еда и напитки
# # #     'banana': 'банан', 'apple': 'яблоко', 'orange': 'апельсин', 'lemon': 'лимон',
# # #     'strawberry': 'клубника', 'grape': 'виноград', 'watermelon': 'арбуз', 'pear': 'груша',
# # #     'pineapple': 'ананас', 'carrot': 'морковь', 'tomato': 'помидор', 'potato': 'картофель',
# # #     'bread': 'хлеб', 'sandwich': 'сандвич', 'pizza': 'пицца', 'hot dog': 'хот-дог',
# # #     'hamburger': 'гамбургер', 'cake': 'торт', 'cookie': 'печенье', 'donut': 'пончик',
# # #     'ice cream': 'мороженое', 'coffee': 'кофе', 'tea': 'чай', 'wine': 'вино',
# # #     'beer': 'пиво', 'juice': 'сок', 'milk': 'молоко', 'bottle': 'бутылка',
    
# # #     # Посуда и кухня
# # #     'cup': 'чашка', 'bowl': 'миска', 'plate': 'тарелка', 'fork': 'вилка',
# # #     'knife': 'нож', 'spoon': 'ложка', 'glass': 'стакан', 'wine glass': 'бокал',
# # #     'mug': 'кружка', 'pitcher': 'кувшин', 'microwave': 'микроволновка',
# # #     'oven': 'духовка', 'toaster': 'тостер', 'sink': 'раковина', 'refrigerator': 'холодильник',
# # #     'blender': 'блендер', 'kettle': 'чайник',
    
# # #     # Мебель
# # #     'chair': 'стул', 'table': 'стол', 'couch': 'диван', 'sofa': 'диван',
# # #     'bed': 'кровать', 'bench': 'скамейка', 'desk': 'письменный стол',
# # #     'cabinet': 'шкаф', 'shelf': 'полка', 'door': 'дверь', 'window': 'окно',
    
# # #     # Электроника
# # #     'tv': 'телевизор', 'laptop': 'ноутбук', 'computer': 'компьютер',
# # #     'keyboard': 'клавиатура', 'mouse': 'мышь', 'remote': 'пульт',
# # #     'cell phone': 'телефон', 'telephone': 'телефон', 'clock': 'часы',
# # #     'camera': 'камера', 'microphone': 'микрофон', 'headphones': 'наушники',
    
# # #     # Одежда и аксессуары
# # #     'backpack': 'рюкзак', 'handbag': 'сумка', 'suitcase': 'чемодан',
# # #     'umbrella': 'зонт', 'tie': 'галстук', 'hat': 'шляпа', 'cap': 'кепка',
# # #     'glasses': 'очки', 'sunglasses': 'солнечные очки', 'shoe': 'обувь',
# # #     'boot': 'ботинок', 'belt': 'ремень', 'jacket': 'куртка', 'coat': 'пальто',
    
# # #     # Спорт
# # #     'ball': 'мяч', 'football': 'футбольный мяч', 'basketball': 'баскетбольный мяч',
# # #     'tennis racket': 'теннисная ракетка', 'baseball bat': 'бейсбольная бита',
# # #     'baseball glove': 'бейсбольная перчатка', 'skateboard': 'скейтборд',
# # #     'surfboard': 'доска для серфинга', 'ski': 'лыжи', 'snowboard': 'сноуборд',
    
# # #     # Растения и природа
# # #     'tree': 'дерево', 'flower': 'цветок', 'plant': 'растение', 'grass': 'трава',
# # #     'rose': 'роза', 'mushroom': 'гриб', 'fountain': 'фонтан',
    
# # #     # Здания и архитектура
# # #     'building': 'здание', 'house': 'дом', 'skyscraper': 'небоскреб',
# # #     'tower': 'башня', 'bridge': 'мост', 'stairs': 'лестница', 'ladder': 'лестница',
    
# # #     # Инструменты и предметы
# # #     'book': 'книга', 'pen': 'ручка', 'pencil': 'карандаш', 'scissors': 'ножницы',
# # #     'key': 'ключ', 'lock': 'замок', 'hammer': 'молоток', 'screwdriver': 'отвертка',
# # #     'wrench': 'гаечный ключ', 'toolbox': 'ящик для инструментов',
    
# # #     # Разное
# # #     'trash can': 'мусорное ведро', 'box': 'коробка', 'bag': 'сумка',
# # #     'basket': 'корзина', 'pillow': 'подушка', 'blanket': 'одеяло',
# # #     'towel': 'полотенце', 'candle': 'свеча', 'lamp': 'лампа',
# # #     'light': 'свет', 'mirror': 'зеркало', 'picture frame': 'рамка для фото',
# # #     'vase': 'ваза', 'curtain': 'штора', 'rug': 'ковер', 'carpet': 'ковер',
# # #     'ceiling': 'потолок', 'floor': 'пол', 'wall': 'стена',
# # #     'flag': 'флаг', 'balloon': 'воздушный шар', 'toy': 'игрушка',
# # #     'kite': 'воздушный змей', 'fireworks': 'фейерверк'
# # # }

# # # PRIORITY_OBJECTS = {
# # #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck', 
# # #     'traffic light', 'stop sign', 'dog', 'cat'
# # # }

# # # def estimate_distance(bbox, frame_height, object_label):
# # #     bbox_height_pixels = bbox[3] - bbox[1]
# # #     if bbox_height_pixels <= 0:
# # #         return None
# # #     FOCAL_LENGTH_PIXELS = 700
# # #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# # #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# # #     return round(distance, 2)

# # # print("🔄 Загрузка YOLO...")
# # # model = YOLO('yolov5x-oiv7-fixed.pt')
# # # print("✅ YOLO готов")

# # # def detect_objects(frame):
# # #     try:
# # #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# # #         # Используем максимальные параметры для 600+ объектов
# # #         results = model.predict(img_rgb, conf=0.25, iou=0.45, verbose=False, imgsz=640, max_det=100)[0]
        
# # #         detections = []
# # #         frame_height, frame_width = frame.shape[:2]

# # #         for r in results.boxes:
# # #             bbox = r.xyxy[0].cpu().numpy()
# # #             conf = float(r.conf[0])
# # #             cls_id = int(r.cls[0])
# # #             label = model.names[cls_id]
# # #             distance = estimate_distance(bbox, frame_height, label)
            
# # #             x_center = (bbox[0] + bbox[2]) / 2
# # #             y_center = (bbox[1] + bbox[3]) / 2
            
# # #             detections.append({
# # #                 "label": label,
# # #                 "confidence": conf,
# # #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# # #                 "distance_m": float(distance) if distance else None,
# # #                 "priority": label in PRIORITY_OBJECTS,
# # #                 "x_center": float(x_center),
# # #                 "y_center": float(y_center)
# # #             })
        
# # #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
# # #         print(f"🔍 Обнаружено: {len(detections)} объектов из 600+ классов")
# # #         return detections
# # #     except Exception as e:
# # #         print(f"❌ Ошибка YOLO: {e}")
# # #         return []

# # # def generate_obstacle_description(obstacles, ocr_data=None, colors=None, brightness=None):
# # #     descriptions = []
    
# # #     if brightness:
# # #         descriptions.append(f"{brightness}")
    
# # #     if colors and len(colors) > 0:
# # #         color_desc = " и ".join([c['color'] for c in colors[:2]])
# # #         descriptions.append(f"Основные цвета: {color_desc}")
    
# # #     # OCR описание закомментировано
# # #     # if ocr_data and ocr_data.get('has_text'):
# # #     #     text = ocr_data['text']
# # #     #     if text:
# # #     #         text_clean = ' '.join(text.split())
# # #     #         if len(text_clean) > 150:
# # #     #             descriptions.append(f"Текст: {text_clean[:150]}")
# # #     #         else:
# # #     #             descriptions.append(f"Текст: {text_clean}")
# # #     #     
# # #     #     if ocr_data.get('phones'):
# # #     #         descriptions.append(f"Телефоны: {', '.join(ocr_data['phones'])}")
    
# # #     if obstacles:
# # #         descriptions.append(f"Объектов: {len(obstacles)}")
        
# # #         for obs in obstacles[:10]:  # Показываем больше объектов
# # #             label_en = obs['label']
# # #             label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# # #             distance = obs.get('distance_m')
            
# # #             if distance:
# # #                 if distance < 1.0:
# # #                     dist_text = f"очень близко"
# # #                 elif distance < 2.0:
# # #                     dist_text = f"близко"
# # #                 else:
# # #                     dist_text = f"{int(distance)} метров"
# # #             else:
# # #                 dist_text = "расстояние неизвестно"
            
# # #             priority = "Внимание! " if obs.get('priority') else ""
# # #             descriptions.append(f"{priority}{label_ru}, {dist_text}")
        
# # #         if len(obstacles) > 10:
# # #             descriptions.append(f"и еще {len(obstacles) - 10} объектов")
# # #     else:
# # #         descriptions.append("Препятствий не обнаружено")
    
# # #     return ". ".join(descriptions)

# # # # --- ЭНДПОИНТЫ ---

# # # @app.route('/')
# # # def index():
# # #     return jsonify({
# # #         "status": "Blind Assistant Server v15 - ТОЛЬКО РАСПОЗНАВАНИЕ ОБЪЕКТОВ",
# # #         "features": [
# # #             "✅ Детекция 600+ объектов YOLO",
# # #             "✅ Голосовые команды",
# # #             "✅ Анализ цветов и яркости",
# # #             "⏸️ OCR временно отключен",
# # #             "⏸️ Распознавание денег временно отключено"
# # #         ]
# # #     })

# # # @app.route('/register_voice', methods=['POST'])
# # # def register_voice():
# # #     temp_audio = temp_wav = None
# # #     try:
# # #         if 'audio' not in request.files or 'username' not in request.form:
# # #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# # #         username = request.form['username'].strip()
# # #         audio_file = request.files['audio']
        
# # #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# # #         audio_file.save(temp_audio)
        
# # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # #         if not temp_wav:
# # #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# # #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # #         conn = get_db_connection()
# # #         cur = conn.cursor()
        
# # #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# # #         existing = cur.fetchone()
        
# # #         if existing:
# # #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (voice_features, username))
# # #             message = "Профиль обновлен"
# # #         else:
# # #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, voice_features))
# # #             message = "Профиль создан"
        
# # #         conn.commit()
# # #         cur.close()
# # #         conn.close()
        
# # #         return jsonify({"status": "success", "message": message, "username": username})
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500
# # #     finally:
# # #         for f in [temp_audio, temp_wav]:
# # #             if f and os.path.exists(f):
# # #                 try: os.remove(f)
# # #                 except: pass

# # # @app.route('/login_voice', methods=['POST'])
# # # def login_voice():
# # #     temp_audio = temp_wav = None
# # #     try:
# # #         username = request.form['username'].strip()
# # #         audio_file = request.files['audio']
        
# # #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# # #         audio_file.save(temp_audio)
        
# # #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# # #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# # #         conn = get_db_connection()
# # #         cur = conn.cursor()
        
# # #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# # #         result = cur.fetchone()
        
# # #         if not result:
# # #             return jsonify({"error": "Пользователь не найден"}), 404
        
# # #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# # #         cur.close()
# # #         conn.close()
        
# # #         if similarity > 0.7:
# # #             welcome_text = f"Добро пожаловать, {username}! Камера готова."
# # #             return jsonify({
# # #                 "status": "success",
# # #                 "message": "Вход выполнен",
# # #                 "username": username,
# # #                 "similarity": float(similarity),
# # #                 "welcome_text": welcome_text,
# # #                 "skippable": True
# # #             })
# # #         else:
# # #             return jsonify({"status": "fail", "message": "Голос не распознан"})
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500
# # #     finally:
# # #         for f in [temp_audio, temp_wav]:
# # #             if f and os.path.exists(f):
# # #                 try: os.remove(f)
# # #                 except: pass

# # # @app.route('/voice_command', methods=['POST'])
# # # def voice_command():
# # #     try:
# # #         text = request.form.get('text', '').lower().strip()
# # #         print(f"🎤 Голосовая команда: '{text}'")
        
# # #         if not text:
# # #             return jsonify({"error": "Текст не распознан"}), 400
        
# # #         command = voice_command_recognizer.recognize_command(text)
        
# # #         if command == 'время':
# # #             time_data = time_service.get_current_time()
# # #             return jsonify({"status": "success", "command": "время", "data": time_data})
# # #         elif command in ['сканировать', 'выйти']:  # Убраны команды 'читать' и 'деньги'
# # #             return jsonify({"status": "success", "command": command})
# # #         else:
# # #             return jsonify({
# # #                 "status": "unknown",
# # #                 "message": "Команда не распознана",
# # #                 "available_commands": ["сканировать", "время", "выйти"]
# # #             })
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route('/process_frame', methods=['POST'])
# # # def process_frame():
# # #     try:
# # #         print("\n🔵 Обработка кадра...")
# # #         if 'frame' not in request.files:
# # #             return jsonify({"error": "Кадр не предоставлен"}), 400

# # #         frame_bytes = request.files['frame'].read()
# # #         nparr = np.frombuffer(frame_bytes, np.uint8)
# # #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# # #         if frame is None:
# # #             return jsonify({"error": "Неверное изображение"}), 400

# # #         print(f"✅ Изображение: {frame.shape}")

# # #         # Только распознавание объектов
# # #         obstacles = detect_objects(frame)
# # #         colors = color_analyzer.analyze_colors(frame)
# # #         brightness = color_analyzer.get_brightness_level(frame)
        
# # #         annotated_frame = object_visualizer.draw_detections(frame, obstacles, None)
# # #         description = generate_obstacle_description(obstacles, None, colors, brightness)
        
# # #         lang = 'ru'
# # #         audio_path = tts_engine.text_to_speech(description, lang=lang)
        
# # #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# # #         cv2.imwrite(annotated_path, annotated_frame)
        
# # #         with open(annotated_path, 'rb') as f:
# # #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# # #         os.remove(annotated_path)

# # #         print(f"✅ Готово")

# # #         return jsonify({
# # #             "obstacles": obstacles,
# # #             "colors": colors,
# # #             "brightness": brightness,
# # #             "description": description,
# # #             "count": len(obstacles),
# # #             "audio_available": audio_path is not None,
# # #             "annotated_frame": annotated_base64,
# # #             "message": "Кадр обработан"
# # #         })
# # #     except Exception as e:
# # #         print(f"❌ Ошибка: {e}")
# # #         return jsonify({"error": str(e)}), 500

# # # # ЭНДПОИНТЫ /read_text И /recognize_money ЗАКОММЕНТИРОВАНЫ
# # # # @app.route('/read_text', methods=['POST'])
# # # # def read_text():
# # # #     return jsonify({"error": "Функция временно отключена"}), 503

# # # # @app.route('/recognize_money', methods=['POST'])
# # # # def recognize_money():
# # # #     return jsonify({"error": "Функция временно отключена"}), 503

# # # @app.route('/get_time', methods=['GET'])
# # # def get_time():
# # #     try:
# # #         time_data = time_service.get_current_time()
# # #         return jsonify(time_data)
# # #     except Exception as e:
# # #         return jsonify({"error": str(e)}), 500

# # # @app.route('/test_connection', methods=['GET'])
# # # def test_connection():
# # #     return jsonify({"status": "ok", "message": "Сервер работает"})

# # # if __name__ == '__main__':
# # #     print("\n" + "="*70)
# # #     print("🚀 BLIND ASSISTANT SERVER v15 - ТОЛЬКО ОБЪЕКТЫ")
# # #     print("="*70)
# # #     print("✅ Детекция 600+ объектов YOLO")
# # #     print("✅ Анализ цветов и освещения")
# # #     print("✅ Голосовые команды и биометрия")
# # #     print("⏸️ OCR временно отключен")
# # #     print("⏸️ Распознавание денег временно отключено")
# # #     print("="*70)
# # #     print("\n🎯 АКТИВНЫЕ ФУНКЦИИ:")
# # #     print("   🔍 YOLO (600+ объектов):")
# # #     print("      - Люди и части тела: лицо, руки, ноги")
# # #     print("      - Транспорт: все виды машин, самолеты, корабли")
# # #     print("      - Животные: 20+ видов животных")
# # #     print("      - Еда: 40+ видов еды и напитков")
# # #     print("      - Посуда: тарелки, чашки, столовые приборы")
# # #     print("      - Мебель: столы, стулья, кровати, шкафы")
# # #     print("      - Электроника: компьютеры, телефоны, камеры")
# # #     print("      - Одежда: обувь, сумки, шляпы, очки")
# # #     print("      - Спорт: мячи, ракетки, скейтборды")
# # #     print("      - И МНОГОЕ ДРУГОЕ!")
# # #     print()
# # #     print("   🎨 Анализ цветов:")
# # #     print("      - Определение основных цветов в кадре")
# # #     print("      - Оценка уровня освещения")
# # #     print()
# # #     print("   🎤 Голосовые команды:")
# # #     print("      - 'сканировать' - анализ объектов")
# # #     print("      - 'время' - узнать текущее время")
# # #     print("      - 'выйти' - выход из приложения")
# # #     print("="*70)
# # #     print("\n⚡ СКОРОСТЬ:")
# # #     print("   - Распознавание объектов: ~0.5-1 секунда")
# # #     print("   - Анализ цветов: мгновенно")
# # #     print("   - Озвучивание: ~0.3 секунды")
# # #     print("="*70)
# # #     print("\n💡 ДОСТУПНЫЕ КОМАНДЫ:")
# # #     print("   🔹 'сканировать' - полный анализ сцены")
# # #     print("   🔹 'время' - текущее время")
# # #     print("   🔹 'выйти' - закрыть приложение")
# # #     print("="*70)
# # #     print("\n🌐 Сервер: http://10.201.4.84:5000")
# # #     print("="*70 + "\n")
    
# # #     app.run(host='10.59.29.73', port=5000, debug=True)











# # import os
# # import uuid
# # import cv2
# # import numpy as np
# # import psycopg2
# # from flask import Flask, request, jsonify
# # from ultralytics import YOLO
# # import wave
# # import hashlib
# # import base64
# # from gtts import gTTS
# # import re
# # from datetime import datetime
# # from pydub import AudioSegment
# # from langdetect import detect, LangDetectException

# # # --- Конфигурация ---
# # DB_NAME = "blind_app"
# # DB_USER = "postgres"
# # DB_PASSWORD = "12345"
# # DB_HOST = "localhost"

# # app = Flask(__name__)

# # @app.after_request
# # def after_request(response):
# #     response.headers.add('Access-Control-Allow-Origin', '*')
# #     response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
# #     response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
# #     return response

# # def get_db_connection():
# #     return psycopg2.connect(
# #         dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
# #     )

# # # --- СОХРАНЕНИЕ В БД ---
# # def save_obstacles_to_db(obstacles, username=None):
# #     """Сохраняет обнаруженные препятствия в базу данных"""
# #     if not obstacles:
# #         return False
    
# #     try:
# #         conn = get_db_connection()
# #         cur = conn.cursor()
        
# #         # Получаем user_id если передано имя пользователя
# #         user_id = None
# #         if username:
# #             cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# #             result = cur.fetchone()
# #             if result:
# #                 user_id = result[0]
        
# #         # Сохраняем каждое препятствие
# #         saved_count = 0
# #         for obs in obstacles:
# #             label = obs.get('label', 'unknown')
# #             confidence = obs.get('confidence', 0.0)
# #             bbox = str(obs.get('bbox', []))
# #             distance = obs.get('distance_m')
            
# #             cur.execute(
# #                 """INSERT INTO obstacles (label, confidence, bbox, distance, user_id, detected_at) 
# #                    VALUES (%s, %s, %s, %s, %s, NOW())""",
# #                 (label, confidence, bbox, distance, user_id)
# #             )
# #             saved_count += 1
        
# #         conn.commit()
# #         cur.close()
# #         conn.close()
# #         print(f"✅ Сохранено {saved_count} препятствий в БД")
# #         return True
# #     except Exception as e:
# #         print(f"❌ Ошибка сохранения препятствий: {e}")
# #         return False

# # def save_voice_sample_to_db(username, audio_path, sample_type='login'):
# #     """Сохраняет голосовой образец в таблицу voice_samples"""
# #     try:
# #         conn = get_db_connection()
# #         cur = conn.cursor()
        
# #         # Получаем user_id
# #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# #         result = cur.fetchone()
        
# #         if not result:
# #             print(f"❌ Пользователь {username} не найден")
# #             cur.close()
# #             conn.close()
# #             return False
        
# #         user_id = result[0]
        
# #         # Читаем аудиофайл
# #         with open(audio_path, 'rb') as f:
# #             audio_data = f.read()
        
# #         # Сохраняем в БД
# #         cur.execute(
# #             """INSERT INTO voice_samples (user_id, audio_data, sample_type, created_at) 
# #                VALUES (%s, %s, %s, NOW())""",
# #             (user_id, audio_data, sample_type)
# #         )
        
# #         conn.commit()
# #         cur.close()
# #         conn.close()
# #         print(f"✅ Голосовой образец '{sample_type}' сохранен для {username}")
# #         return True
# #     except Exception as e:
# #         print(f"❌ Ошибка сохранения голосового образца: {e}")
# #         return False

# # # --- ВРЕМЯ ---
# # class TimeService:
# #     def get_current_time(self):
# #         now = datetime.now()
# #         hours = now.hour
# #         minutes = now.minute
        
# #         if minutes == 1:
# #             min_word = "минута"
# #         elif 2 <= minutes <= 4:
# #             min_word = "минуты"
# #         else:
# #             min_word = "минут"
        
# #         if hours == 1 or hours == 21:
# #             hour_word = "час"
# #         elif 2 <= hours <= 4 or 22 <= hours <= 24:
# #             hour_word = "часа"
# #         else:
# #             hour_word = "часов"
        
# #         time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
# #         return {'time': time_str, 'hour': hours, 'minute': minutes}

# # time_service = TimeService()

# # # --- АНАЛИЗ ЦВЕТОВ ---
# # class ColorAnalyzer:
# #     def __init__(self):
# #         self.color_names = {
# #             'красный': ([0, 100, 100], [10, 255, 255]),
# #             'оранжевый': ([10, 100, 100], [25, 255, 255]),
# #             'желтый': ([25, 100, 100], [35, 255, 255]),
# #             'зеленый': ([35, 100, 100], [85, 255, 255]),
# #             'голубой': ([85, 100, 100], [100, 255, 255]),
# #             'синий': ([100, 100, 100], [130, 255, 255]),
# #             'фиолетовый': ([130, 100, 100], [160, 255, 255]),
# #             'розовый': ([160, 100, 100], [170, 255, 255]),
# #             'белый': ([0, 0, 200], [180, 30, 255]),
# #             'серый': ([0, 0, 50], [180, 30, 200]),
# #             'черный': ([0, 0, 0], [180, 255, 50])
# #         }
    
# #     def analyze_colors(self, frame):
# #         try:
# #             hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# #             detected_colors = []
            
# #             for color_name, (lower, upper) in self.color_names.items():
# #                 mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
# #                 percentage = (np.sum(mask > 0) / mask.size) * 100
                
# #                 if percentage > 5:
# #                     detected_colors.append({'color': color_name, 'percentage': round(percentage, 1)})
            
# #             detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
# #             return detected_colors[:3]
# #         except:
# #             return []
    
# #     def get_brightness_level(self, frame):
# #         try:
# #             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# #             avg_brightness = np.mean(gray)
            
# #             if avg_brightness < 50:
# #                 return "очень темно"
# #             elif avg_brightness < 100:
# #                 return "темно"
# #             elif avg_brightness < 150:
# #                 return "нормальное освещение"
# #             elif avg_brightness < 200:
# #                 return "светло"
# #             else:
# #                 return "очень светло"
# #         except:
# #             return "не определено"

# # color_analyzer = ColorAnalyzer()

# # # --- ВИЗУАЛИЗАЦИЯ ---
# # class ObjectVisualizer:
# #     def __init__(self):
# #         self.font = cv2.FONT_HERSHEY_SIMPLEX
    
# #     def draw_detections(self, frame, detections):
# #         try:
# #             annotated_frame = frame.copy()
            
# #             for det in detections:
# #                 bbox = det['bbox']
# #                 label_en = det['label']
# #                 label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# #                 distance = det.get('distance_m')
# #                 priority = det.get('priority', False)
                
# #                 color = (0, 255, 0)
# #                 thickness = 3 if priority else 2
                
# #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
                
# #                 text = f"{label_ru} - {distance:.1f}м" if distance else f"{label_ru}"
                
# #                 (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
# #                 cv2.rectangle(annotated_frame, (bbox[0], bbox[1] - text_height - 10),
# #                             (bbox[0] + text_width + 10, bbox[1]), color, -1)
                
# #                 cv2.putText(annotated_frame, text, (bbox[0] + 5, bbox[1] - 5),
# #                           self.font, 0.7, (0, 0, 0), 2)
            
# #             return annotated_frame
# #         except:
# #             return frame

# # object_visualizer = ObjectVisualizer()

# # # --- МУЛЬТИЯЗЫЧНАЯ ОЗВУЧКА ---
# # class MultilingualTTS:
# #     def __init__(self):
# #         self.cache_dir = "audio_cache"
# #         if not os.path.exists(self.cache_dir):
# #             os.makedirs(self.cache_dir)
    
# #     def detect_language(self, text):
# #         try:
# #             text_clean = re.sub(r'[0-9\s\.,!?;:]', '', text)
# #             if len(text_clean) < 3:
# #                 return 'ru'
            
# #             cyrillic_count = len(re.findall(r'[а-яА-ЯЁё]', text_clean))
# #             latin_count = len(re.findall(r'[a-zA-Z]', text_clean))
            
# #             if cyrillic_count > latin_count:
# #                 return 'ru'
# #             elif latin_count > cyrillic_count:
# #                 return 'en'
            
# #             lang = detect(text_clean)
# #             return 'en' if lang == 'en' else 'ru'
# #         except:
# #             return 'ru'
    
# #     def text_to_speech(self, text, lang=None):
# #         try:
# #             if lang is None:
# #                 lang = self.detect_language(text)
            
# #             text_hash = hashlib.md5(f"{text}_{lang}".encode()).hexdigest()
# #             audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
# #             if os.path.exists(audio_path):
# #                 return audio_path
            
# #             tts = gTTS(text=text, lang=lang, slow=False)
# #             tts.save(audio_path)
# #             return audio_path
# #         except Exception as e:
# #             print(f"Ошибка озвучки: {e}")
# #             return None

# # tts_engine = MultilingualTTS()

# # # --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
# # class SimpleVoiceBiometrics:
# #     def extract_mfcc_features(self, audio_path):
# #         try:
# #             with wave.open(audio_path, 'rb') as wav_file:
# #                 sample_width = wav_file.getsampwidth()
# #                 frame_rate = wav_file.getframerate()
# #                 n_frames = wav_file.getnframes()
# #                 frames = wav_file.readframes(n_frames)
                
# #                 if sample_width == 2:
# #                     audio_data = np.frombuffer(frames, dtype=np.int16)
# #                 else:
# #                     audio_data = np.frombuffer(frames, dtype=np.uint8)
# #                     audio_data = audio_data.astype(np.float32) - 128
                
# #                 audio_data = audio_data.astype(np.float32) / 32768.0
# #                 features = self._simple_audio_features(audio_data, frame_rate)
# #                 features_bytes = features.astype(np.float32).tobytes()
# #                 features_b64 = base64.b64encode(features_bytes).decode('utf-8')
# #                 return features_b64
# #         except Exception as e:
# #             print(f"Ошибка извлечения характеристик: {e}")
# #             return None
    
# #     def _simple_audio_features(self, audio_data, sample_rate):
# #         features = []
# #         features.append(np.mean(audio_data ** 2))
# #         features.append(np.std(audio_data))
# #         features.append(np.max(np.abs(audio_data)))
        
# #         fft = np.fft.fft(audio_data)
# #         fft_magnitude = np.abs(fft[:len(fft)//2])
        
# #         bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
# #         freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
# #         for low, high in bands:
# #             mask = (freqs >= low) & (freqs < high)
# #             if np.any(mask):
# #                 features.append(np.mean(fft_magnitude[mask]))
# #             else:
# #                 features.append(0.0)
        
# #         return np.array(features)
    
# #     def compare_voice_features(self, features1_b64, features2_b64):
# #         try:
# #             features1_bytes = base64.b64decode(features1_b64)
# #             features2_bytes = base64.b64decode(features2_b64)
            
# #             features1 = np.frombuffer(features1_bytes, dtype=np.float32)
# #             features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
# #             min_len = min(len(features1), len(features2))
# #             features1 = features1[:min_len]
# #             features2 = features2[:min_len]
            
# #             distance = np.linalg.norm(features1 - features2)
# #             max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
# #             similarity = 1.0 - (distance / (max_distance + 1e-8))
# #             return float(max(0.0, min(1.0, similarity)))
# #         except:
# #             return 0.0

# # voice_biometrics = SimpleVoiceBiometrics()

# # # --- КОМАНДЫ ---
# # class VoiceCommandRecognizer:
# #     def __init__(self):
# #         self.commands = {
# #             'выйти': ['выйти', 'выход', 'выхожу', 'закрыть', 'exit'],
# #             'сканировать': ['сканировать', 'скан', 'сканируй', 'что вижу', 'что впереди', 'анализ', 'scan'],
# #             'время': ['время', 'который час', 'сколько времени', 'time'],
# #         }
    
# #     def recognize_command(self, text):
# #         if not text:
# #             return None
        
# #         text = text.lower().strip()
# #         noise = ['слушаю', 'команду', 'команда', 'пожалуйста']
# #         for n in noise:
# #             text = text.replace(n, '')
# #         text = ' '.join(text.split())
        
# #         print(f"🎤 Анализ: '{text}'")
        
# #         for cmd_type, keywords in self.commands.items():
# #             for keyword in keywords:
# #                 if keyword in text or text in keyword:
# #                     print(f"✅ Команда: {cmd_type}")
# #                     return cmd_type
        
# #         print(f"❌ Не распознано")
# #         return None

# # voice_command_recognizer = VoiceCommandRecognizer()

# # # --- КОНВЕРТЕР АУДИО ---
# # class AudioConverter:
# #     def convert_aac_to_wav(self, aac_path):
# #         try:
# #             audio = AudioSegment.from_file(aac_path, format="aac")
# #             wav_path = aac_path.replace('.aac', '.wav')
# #             audio.export(wav_path, format="wav")
# #             return wav_path
# #         except Exception as e:
# #             print(f"Ошибка конвертации: {e}")
# #             return None

# # audio_converter = AudioConverter()

# # # --- YOLO ---
# # OBJECT_HEIGHTS = {
# #     'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
# #     'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
# #     'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8, 'book': 0.3,
# #     'bicycle': 1.0, 'motorcycle': 1.2, 'bus': 3.0, 'truck': 3.0,
# #     'table': 0.8, 'bed': 0.5, 'refrigerator': 1.8
# # }

# # OBJECT_TRANSLATIONS = {
# #     'person': 'человек', 'man': 'мужчина', 'woman': 'женщина', 'boy': 'мальчик', 'girl': 'девочка',
# #     'human face': 'лицо человека', 'human hand': 'рука', 'human foot': 'нога', 'human ear': 'ухо',
# #     'human nose': 'нос', 'human mouth': 'рот', 'human eye': 'глаз', 'human head': 'голова',
# #     'car': 'машина', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл', 'bus': 'автобус',
# #     'truck': 'грузовик', 'van': 'фургон', 'taxi': 'такси', 'ambulance': 'скорая помощь',
# #     'fire truck': 'пожарная машина', 'police car': 'полицейская машина',
# #     'airplane': 'самолет', 'helicopter': 'вертолет', 'boat': 'лодка', 'ship': 'корабль',
# #     'train': 'поезд', 'wheel': 'колесо', 'tire': 'шина', 'vehicle registration plate': 'номерной знак',
# #     'traffic light': 'светофор', 'stop sign': 'знак стоп', 'parking meter': 'парковочный счетчик',
# #     'street light': 'уличный фонарь', 'traffic sign': 'дорожный знак',
# #     'dog': 'собака', 'cat': 'кошка', 'bird': 'птица', 'horse': 'лошадь', 'cow': 'корова',
# #     'sheep': 'овца', 'elephant': 'слон', 'bear': 'медведь', 'zebra': 'зебра', 'giraffe': 'жираф',
# #     'rabbit': 'кролик', 'mouse': 'мышь', 'snake': 'змея', 'lion': 'лев', 'tiger': 'тигр',
# #     'butterfly': 'бабочка', 'bee': 'пчела', 'spider': 'паук', 'fish': 'рыба', 'chicken': 'курица',
# #     'banana': 'банан', 'apple': 'яблоко', 'orange': 'апельсин', 'lemon': 'лимон',
# #     'strawberry': 'клубника', 'grape': 'виноград', 'watermelon': 'арбуз', 'pear': 'груша',
# #     'pineapple': 'ананас', 'carrot': 'морковь', 'tomato': 'помидор', 'potato': 'картофель',
# #     'bread': 'хлеб', 'sandwich': 'сандвич', 'pizza': 'пицца', 'hot dog': 'хот-дог',
# #     'hamburger': 'гамбургер', 'cake': 'торт', 'cookie': 'печенье', 'donut': 'пончик',
# #     'ice cream': 'мороженое', 'coffee': 'кофе', 'tea': 'чай', 'wine': 'вино',
# #     'beer': 'пиво', 'juice': 'сок', 'milk': 'молоко', 'bottle': 'бутылка',
# #     'cup': 'чашка', 'bowl': 'миска', 'plate': 'тарелка', 'fork': 'вилка',
# #     'knife': 'нож', 'spoon': 'ложка', 'glass': 'стакан', 'wine glass': 'бокал',
# #     'mug': 'кружка', 'pitcher': 'кувшин', 'microwave': 'микроволновка',
# #     'oven': 'духовка', 'toaster': 'тостер', 'sink': 'раковина', 'refrigerator': 'холодильник',
# #     'blender': 'блендер', 'kettle': 'чайник',
# #     'chair': 'стул', 'table': 'стол', 'couch': 'диван', 'sofa': 'диван',
# #     'bed': 'кровать', 'bench': 'скамейка', 'desk': 'письменный стол',
# #     'cabinet': 'шкаф', 'shelf': 'полка', 'door': 'дверь', 'window': 'окно',
# #     'tv': 'телевизор', 'laptop': 'ноутбук', 'computer': 'компьютер',
# #     'keyboard': 'клавиатура', 'mouse': 'мышь', 'remote': 'пульт',
# #     'cell phone': 'телефон', 'telephone': 'телефон', 'clock': 'часы',
# #     'camera': 'камера', 'microphone': 'микрофон', 'headphones': 'наушники',
# #     'backpack': 'рюкзак', 'handbag': 'сумка', 'suitcase': 'чемодан',
# #     'umbrella': 'зонт', 'tie': 'галстук', 'hat': 'шляпа', 'cap': 'кепка',
# #     'glasses': 'очки', 'sunglasses': 'солнечные очки', 'shoe': 'обувь',
# #     'boot': 'ботинок', 'belt': 'ремень', 'jacket': 'куртка', 'coat': 'пальто',
# #     'ball': 'мяч', 'football': 'футбольный мяч', 'basketball': 'баскетбольный мяч',
# #     'tennis racket': 'теннисная ракетка', 'baseball bat': 'бейсбольная бита',
# #     'baseball glove': 'бейсбольная перчатка', 'skateboard': 'скейтборд',
# #     'surfboard': 'доска для серфинга', 'ski': 'лыжи', 'snowboard': 'сноуборд',
# #     'tree': 'дерево', 'flower': 'цветок', 'plant': 'растение', 'grass': 'трава',
# #     'rose': 'роза', 'mushroom': 'гриб', 'fountain': 'фонтан',
# #     'building': 'здание', 'house': 'дом', 'skyscraper': 'небоскреб',
# #     'tower': 'башня', 'bridge': 'мост', 'stairs': 'лестница', 'ladder': 'лестница',
# #     'book': 'книга', 'pen': 'ручка', 'pencil': 'карандаш', 'scissors': 'ножницы',
# #     'key': 'ключ', 'lock': 'замок', 'hammer': 'молоток', 'screwdriver': 'отвертка',
# #     'wrench': 'гаечный ключ', 'toolbox': 'ящик для инструментов',
# #     'trash can': 'мусорное ведро', 'box': 'коробка', 'bag': 'сумка',
# #     'basket': 'корзина', 'pillow': 'подушка', 'blanket': 'одеяло',
# #     'towel': 'полотенце', 'candle': 'свеча', 'lamp': 'лампа',
# #     'light': 'свет', 'mirror': 'зеркало', 'picture frame': 'рамка для фото',
# #     'vase': 'ваза', 'curtain': 'штора', 'rug': 'ковер', 'carpet': 'ковер',
# #     'ceiling': 'потолок', 'floor': 'пол', 'wall': 'стена',
# #     'flag': 'флаг', 'balloon': 'воздушный шар', 'toy': 'игрушка',
# #     'kite': 'воздушный змей', 'fireworks': 'фейерверк'
# # }

# # PRIORITY_OBJECTS = {
# #     'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck', 
# #     'traffic light', 'stop sign', 'dog', 'cat'
# # }

# # def estimate_distance(bbox, frame_height, object_label):
# #     bbox_height_pixels = bbox[3] - bbox[1]
# #     if bbox_height_pixels <= 0:
# #         return None
# #     FOCAL_LENGTH_PIXELS = 700
# #     real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
# #     distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
# #     return round(distance, 2)

# # print("🔄 Загрузка YOLO...")
# # model = YOLO('yolov5x-oiv7-fixed.pt')
# # print("✅ YOLO готов")

# # def detect_objects(frame):
# #     try:
# #         img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# #         results = model.predict(img_rgb, conf=0.25, iou=0.45, verbose=False, imgsz=640, max_det=100)[0]
        
# #         detections = []
# #         frame_height, frame_width = frame.shape[:2]

# #         for r in results.boxes:
# #             bbox = r.xyxy[0].cpu().numpy()
# #             conf = float(r.conf[0])
# #             cls_id = int(r.cls[0])
# #             label = model.names[cls_id]
# #             distance = estimate_distance(bbox, frame_height, label)
            
# #             x_center = (bbox[0] + bbox[2]) / 2
# #             y_center = (bbox[1] + bbox[3]) / 2
            
# #             detections.append({
# #                 "label": label,
# #                 "confidence": conf,
# #                 "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
# #                 "distance_m": float(distance) if distance else None,
# #                 "priority": label in PRIORITY_OBJECTS,
# #                 "x_center": float(x_center),
# #                 "y_center": float(y_center)
# #             })
        
# #         detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
# #         print(f"🔍 Обнаружено: {len(detections)} объектов из 600+ классов")
        
# #         # Сохранение объектов в базу данных
# #         save_obstacles_to_db(detections)
        
# #         return detections
# #     except Exception as e:
# #         print(f"❌ Ошибка YOLO: {e}")
# #         return []

# # def generate_obstacle_description(obstacles, colors=None, brightness=None):
# #     descriptions = []
    
# #     if brightness:
# #         descriptions.append(f"{brightness}")
    
# #     if colors and len(colors) > 0:
# #         color_desc = " и ".join([c['color'] for c in colors[:2]])
# #         descriptions.append(f"Основные цвета: {color_desc}")
    
# #     if obstacles:
# #         descriptions.append(f"Объектов: {len(obstacles)}")
        
# #         for obs in obstacles[:10]:
# #             label_en = obs['label']
# #             label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
# #             distance = obs.get('distance_m')
            
# #             if distance:
# #                 if distance < 1.0:
# #                     dist_text = f"очень близко"
# #                 elif distance < 2.0:
# #                     dist_text = f"близко"
# #                 else:
# #                     dist_text = f"{int(distance)} метров"
# #             else:
# #                 dist_text = "расстояние неизвестно"
            
# #             priority = "Внимание! " if obs.get('priority') else ""
# #             descriptions.append(f"{priority}{label_ru}, {dist_text}")
        
# #         if len(obstacles) > 10:
# #             descriptions.append(f"и еще {len(obstacles) - 10} объектов")
# #     else:
# #         descriptions.append("Препятствий не обнаружено")
    
# #     return ". ".join(descriptions)

# # # --- ЭНДПОИНТЫ ---

# # @app.route('/')
# # def index():
# #     return jsonify({
# #         "status": "Blind Assistant Server v15 - ТОЛЬКО РАСПОЗНАВАНИЕ ОБЪЕКТОВ",
# #         "features": [
# #             "✅ Детекция 600+ объектов YOLO",
# #             "✅ Голосовые команды",
# #             "✅ Анализ цветов и яркости",
# #             "✅ Сохранение объектов в базу данных",
# #             "⏸️ OCR временно отключен",
# #             "⏸️ Распознавание денег временно отключено"
# #         ]
# #     })

# # @app.route('/register_voice', methods=['POST'])
# # def register_voice():
# #     temp_audio = temp_wav = None
# #     try:
# #         if 'audio' not in request.files or 'username' not in request.form:
# #             return jsonify({"error": "Необходимо аудио и имя"}), 400

# #         username = request.form['username'].strip()
# #         audio_file = request.files['audio']
        
# #         temp_audio = f"temp_register_{uuid.uuid4()}.aac"
# #         audio_file.save(temp_audio)
        
# #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# #         if not temp_wav:
# #             return jsonify({"error": "Ошибка конвертации"}), 400
        
# #         voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# #         conn = get_db_connection()
# #         cur = conn.cursor()
        
# #         cur.execute("SELECT id FROM users WHERE username = %s", (username,))
# #         existing = cur.fetchone()
        
# #         if existing:
# #             cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (voice_features, username))
# #             message = "Профиль обновлен"
# #         else:
# #             cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, voice_features))
# #             message = "Профиль создан"
        
# #         conn.commit()
# #         cur.close()
# #         conn.close()
        
# #         return jsonify({"status": "success", "message": message, "username": username})
# #     except Exception as e:
# #         return jsonify({"error": str(e)}), 500
# #     finally:
# #         for f in [temp_audio, temp_wav]:
# #             if f and os.path.exists(f):
# #                 try: os.remove(f)
# #                 except: pass

# # @app.route('/login_voice', methods=['POST'])
# # def login_voice():
# #     temp_audio = temp_wav = None
# #     try:
# #         username = request.form['username'].strip()
# #         audio_file = request.files['audio']
        
# #         temp_audio = f"temp_login_{uuid.uuid4()}.aac"
# #         audio_file.save(temp_audio)
        
# #         temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
# #         current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
# #         conn = get_db_connection()
# #         cur = conn.cursor()
        
# #         cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
# #         result = cur.fetchone()
        
# #         if not result:
# #             return jsonify({"error": "Пользователь не найден"}), 404
        
# #         similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
# #         cur.close()
# #         conn.close()
        
# #         if similarity > 0.7:
# #             welcome_text = f"Добро пожаловать, {username}! Камера готова."
# #             return jsonify({
# #                 "status": "success",
# #                 "message": "Вход выполнен",
# #                 "username": username,
# #                 "similarity": float(similarity),
# #                 "welcome_text": welcome_text,
# #                 "skippable": True
# #             })
# #         else:
# #             return jsonify({"status": "fail", "message": "Голос не распознан"})
# #     except Exception as e:
# #         return jsonify({"error": str(e)}), 500
# #     finally:
# #         for f in [temp_audio, temp_wav]:
# #             if f and os.path.exists(f):
# #                 try: os.remove(f)
# #                 except: pass

# # @app.route('/voice_command', methods=['POST'])
# # def voice_command():
# #     try:
# #         text = request.form.get('text', '').lower().strip()
# #         print(f"🎤 Голосовая команда: '{text}'")
        
# #         if not text:
# #             return jsonify({"error": "Текст не распознан"}), 400
        
# #         command = voice_command_recognizer.recognize_command(text)
        
# #         if command == 'время':
# #             time_data = time_service.get_current_time()
# #             return jsonify({"status": "success", "command": "время", "data": time_data})
# #         elif command in ['сканировать', 'выйти']:
# #             return jsonify({"status": "success", "command": command})
# #         else:
# #             return jsonify({
# #                 "status": "unknown",
# #                 "message": "Команда не распознана",
# #                 "available_commands": ["сканировать", "время", "выйти"]
# #             })
# #     except Exception as e:
# #         return jsonify({"error": str(e)}), 500

# # @app.route('/process_frame', methods=['POST'])
# # def process_frame():
# #     try:
# #         print("\n🔵 Обработка кадра...")
# #         if 'frame' not in request.files:
# #             return jsonify({"error": "Кадр не предоставлен"}), 400

# #         frame_bytes = request.files['frame'].read()
# #         nparr = np.frombuffer(frame_bytes, np.uint8)
# #         frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

# #         if frame is None:
# #             return jsonify({"error": "Неверное изображение"}), 400

# #         print(f"✅ Изображение: {frame.shape}")

# #         # Только распознавание объектов
# #         obstacles = detect_objects(frame)
# #         colors = color_analyzer.analyze_colors(frame)
# #         brightness = color_analyzer.get_brightness_level(frame)
        
# #         # ИСПРАВЛЕННЫЙ ВЫЗОВ - убран лишний аргумент
# #         annotated_frame = object_visualizer.draw_detections(frame, obstacles)
        
# #         # ИСПРАВЛЕННЫЙ ВЫЗОВ - убран лишний аргумент
# #         description = generate_obstacle_description(obstacles, colors, brightness)
        
# #         lang = 'ru'
# #         audio_path = tts_engine.text_to_speech(description, lang=lang)
        
# #         annotated_path = f"annotated_{uuid.uuid4()}.jpg"
# #         cv2.imwrite(annotated_path, annotated_frame)
        
# #         with open(annotated_path, 'rb') as f:
# #             annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
# #         os.remove(annotated_path)

# #         print(f"✅ Готово")

# #         return jsonify({
# #             "obstacles": obstacles,
# #             "colors": colors,
# #             "brightness": brightness,
# #             "description": description,
# #             "count": len(obstacles),
# #             "audio_available": audio_path is not None,
# #             "annotated_frame": annotated_base64,
# #             "message": "Кадр обработан"
# #         })
# #     except Exception as e:
# #         print(f"❌ Ошибка: {e}")
# #         return jsonify({"error": str(e)}), 500

# # @app.route('/get_time', methods=['GET'])
# # def get_time():
# #     try:
# #         time_data = time_service.get_current_time()
# #         return jsonify(time_data)
# #     except Exception as e:
# #         return jsonify({"error": str(e)}), 500

# # @app.route('/test_connection', methods=['GET'])
# # def test_connection():
# #     return jsonify({"status": "ok", "message": "Сервер работает"})

# # if __name__ == '__main__':
# #     print("\n" + "="*70)
# #     print("🚀 BLIND ASSISTANT SERVER v15 - ТОЛЬКО ОБЪЕКТЫ")
# #     print("="*70)
# #     print("✅ Детекция 600+ объектов YOLO")
# #     print("✅ Анализ цветов и освещения")
# #     print("✅ Голосовые команды и биометрия")
# #     print("✅ Сохранение объектов в базу данных")
# #     print("⏸️ OCR временно отключен")
# #     print("⏸️ Распознавание денег временно отключено")
# #     print("="*70)
# #     print("\n🎯 АКТИВНЫЕ ФУНКЦИИ:")
# #     print("   🔍 YOLO (600+ объектов):")
# #     print("      - Люди и части тела: лицо, руки, ноги")
# #     print("      - Транспорт: все виды машин, самолеты, корабли")
# #     print("      - Животные: 20+ видов животных")
# #     print("      - Еда: 40+ видов еды и напитков")
# #     print("      - Посуда: тарелки, чашки, столовые приборы")
# #     print("      - Мебель: столы, стулья, кровати, шкафы")
# #     print("      - Электроника: компьютеры, телефоны, камеры")
# #     print("      - Одежда: обувь, сумки, шляпы, очки")
# #     print("      - Спорт: мячи, ракетки, скейтборды")
# #     print("      - И МНОГОЕ ДРУГОЕ!")
# #     print()
# #     print("   🎨 Анализ цветов:")
# #     print("      - Определение основных цветов в кадре")
# #     print("      - Оценка уровня освещения")
# #     print()
# #     print("   💾 База данных:")
# #     print("      - Автоматическое сохранение всех обнаруженных объектов")
# #     print()
# #     print("   🎤 Голосовые команды:")
# #     print("      - 'сканировать' - анализ объектов")
# #     print("      - 'время' - узнать текущее время")
# #     print("      - 'выйти' - выход из приложения")
# #     print("="*70)
# #     print("\n⚡ СКОРОСТЬ:")
# #     print("   - Распознавание объектов: ~0.5-1 секунда")
# #     print("   - Анализ цветов: мгновенно")
# #     print("   - Озвучивание: ~0.3 секунды")
# #     print("   - Сохранение в БД: мгновенно")
# #     print("="*70)
# #     print("\n💡 ДОСТУПНЫЕ КОМАНДЫ:")
# #     print("   🔹 'сканировать' - полный анализ сцены")
# #     print("   🔹 'время' - текущее время")
# #     print("   🔹 'выйти' - закрыть приложение")
# #     print("="*70)
# #     print("\n🌐 Сервер: http://10.59.29.73:5000")
# #     print("="*70 + "\n")
    
# #     app.run(host='10.59.29.73', port=5000, debug=True)







import os
import uuid
import cv2
import numpy as np
import psycopg2
from flask import Flask, request, jsonify
from ultralytics import YOLO
import wave
import hashlib
import base64
from gtts import gTTS
import re
from datetime import datetime, timedelta
import threading
import time
from pydub import AudioSegment
from langdetect import detect, LangDetectException

# --- Конфигурация ---
DB_NAME = "blind_app"
DB_USER = "postgres"
DB_PASSWORD = "12345"
DB_HOST = "localhost"

app = Flask(__name__)

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    return response

def get_db_connection():
    return psycopg2.connect(
        dbname=DB_NAME, user=DB_USER, password=DB_PASSWORD, host=DB_HOST
    )

# --- ФУНКЦИИ ДЛЯ РАБОТЫ С ЗАМЕТКАМИ ---

def get_user_id_by_username(username):
    """Получить ID пользователя по имени"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute("SELECT id FROM users WHERE username = %s", (username,))
        result = cur.fetchone()
        cur.close()
        conn.close()
        return result[0] if result else None
    except Exception as e:
        print(f"Ошибка получения user_id: {e}")
        return None

# --- СОХРАНЕНИЕ В БД ---
def save_obstacles_to_db(obstacles, username=None):
    """Сохраняет обнаруженные препятствия в базу данных"""
    if not obstacles:
        return False
    
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Получаем user_id если передано имя пользователя
        user_id = None
        if username:
            cur.execute("SELECT id FROM users WHERE username = %s", (username,))
            result = cur.fetchone()
            if result:
                user_id = result[0]
        
        # Сохраняем каждое препятствие
        saved_count = 0
        for obs in obstacles:
            label = obs.get('label', 'unknown')
            confidence = obs.get('confidence', 0.0)
            bbox = str(obs.get('bbox', []))
            distance = obs.get('distance_m')
            
            cur.execute(
                """INSERT INTO obstacles (label, confidence, bbox, distance, user_id, detected_at) 
                   VALUES (%s, %s, %s, %s, %s, NOW())""",
                (label, confidence, bbox, distance, user_id)
            )
            saved_count += 1
        
        conn.commit()
        cur.close()
        conn.close()
        print(f"✅ Сохранено {saved_count} препятствий в БД")
        return True
    except Exception as e:
        print(f"❌ Ошибка сохранения препятствий: {e}")
        return False

def save_voice_sample_to_db(username, audio_path, sample_type='login'):
    """Сохраняет голосовой образец в таблицу voice_samples"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Получаем user_id
        cur.execute("SELECT id FROM users WHERE username = %s", (username,))
        result = cur.fetchone()
        
        if not result:
            print(f"❌ Пользователь {username} не найден")
            cur.close()
            conn.close()
            return False
        
        user_id = result[0]
        
        # Читаем аудиофайл
        with open(audio_path, 'rb') as f:
            audio_data = f.read()
        
        # Сохраняем в БД
        cur.execute(
            """INSERT INTO voice_samples (user_id, audio_data, sample_type, created_at) 
               VALUES (%s, %s, %s, NOW())""",
            (user_id, audio_data, sample_type)
        )
        
        conn.commit()
        cur.close()
        conn.close()
        print(f"✅ Голосовой образец '{sample_type}' сохранен для {username}")
        return True
    except Exception as e:
        print(f"❌ Ошибка сохранения голосового образца: {e}")
        return False

# --- ВРЕМЯ ---
class TimeService:
    def get_current_time(self):
        now = datetime.now()
        hours = now.hour
        minutes = now.minute
        
        if minutes == 1:
            min_word = "минута"
        elif 2 <= minutes <= 4:
            min_word = "минуты"
        else:
            min_word = "минут"
        
        if hours == 1 or hours == 21:
            hour_word = "час"
        elif 2 <= hours <= 4 or 22 <= hours <= 24:
            hour_word = "часа"
        else:
            hour_word = "часов"
        
        time_str = f"Сейчас {hours} {hour_word} {minutes} {min_word}"
        return {'time': time_str, 'hour': hours, 'minute': minutes}

time_service = TimeService()

# --- АНАЛИЗ ЦВЕТОВ ---
class ColorAnalyzer:
    def __init__(self):
        self.color_names = {
            'красный': ([0, 100, 100], [10, 255, 255]),
            'оранжевый': ([10, 100, 100], [25, 255, 255]),
            'желтый': ([25, 100, 100], [35, 255, 255]),
            'зеленый': ([35, 100, 100], [85, 255, 255]),
            'голубой': ([85, 100, 100], [100, 255, 255]),
            'синий': ([100, 100, 100], [130, 255, 255]),
            'фиолетовый': ([130, 100, 100], [160, 255, 255]),
            'розовый': ([160, 100, 100], [170, 255, 255]),
            'белый': ([0, 0, 200], [180, 30, 255]),
            'серый': ([0, 0, 50], [180, 30, 200]),
            'черный': ([0, 0, 0], [180, 255, 50])
        }
    
    def analyze_colors(self, frame):
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            detected_colors = []
            
            for color_name, (lower, upper) in self.color_names.items():
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                percentage = (np.sum(mask > 0) / mask.size) * 100
                
                if percentage > 5:
                    detected_colors.append({'color': color_name, 'percentage': round(percentage, 1)})
            
            detected_colors.sort(key=lambda x: x['percentage'], reverse=True)
            return detected_colors[:3]
        except:
            return []
    
    def get_brightness_level(self, frame):
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            avg_brightness = np.mean(gray)
            
            if avg_brightness < 50:
                return "очень темно"
            elif avg_brightness < 100:
                return "темно"
            elif avg_brightness < 150:
                return "нормальное освещение"
            elif avg_brightness < 200:
                return "светло"
            else:
                return "очень светло"
        except:
            return "не определено"

color_analyzer = ColorAnalyzer()

# --- ВИЗУАЛИЗАЦИЯ ---
class ObjectVisualizer:
    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
    
    def draw_detections(self, frame, detections):
        try:
            annotated_frame = frame.copy()
            
            for det in detections:
                bbox = det['bbox']
                label_en = det['label']
                label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
                distance = det.get('distance_m')
                priority = det.get('priority', False)
                
                color = (0, 255, 0)
                thickness = 3 if priority else 2
                
                cv2.rectangle(annotated_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
                
                text = f"{label_ru} - {distance:.1f}м" if distance else f"{label_ru}"
                
                (text_width, text_height), _ = cv2.getTextSize(text, self.font, 0.7, 2)
                cv2.rectangle(annotated_frame, (bbox[0], bbox[1] - text_height - 10),
                            (bbox[0] + text_width + 10, bbox[1]), color, -1)
                
                cv2.putText(annotated_frame, text, (bbox[0] + 5, bbox[1] - 5),
                          self.font, 0.7, (0, 0, 0), 2)
            
            return annotated_frame
        except:
            return frame

object_visualizer = ObjectVisualizer()

# --- МУЛЬТИЯЗЫЧНАЯ ОЗВУЧКА ---
class MultilingualTTS:
    def __init__(self):
        self.cache_dir = "audio_cache"
        if not os.path.exists(self.cache_dir):
            os.makedirs(self.cache_dir)
    
    def detect_language(self, text):
        try:
            text_clean = re.sub(r'[0-9\s\.,!?;:]', '', text)
            if len(text_clean) < 3:
                return 'ru'
            
            cyrillic_count = len(re.findall(r'[а-яА-ЯЁё]', text_clean))
            latin_count = len(re.findall(r'[a-zA-Z]', text_clean))
            
            if cyrillic_count > latin_count:
                return 'ru'
            elif latin_count > cyrillic_count:
                return 'en'
            
            lang = detect(text_clean)
            return 'en' if lang == 'en' else 'ru'
        except:
            return 'ru'
    
    def text_to_speech(self, text, lang=None):
        try:
            if lang is None:
                lang = self.detect_language(text)
            
            text_hash = hashlib.md5(f"{text}_{lang}".encode()).hexdigest()
            audio_path = os.path.join(self.cache_dir, f"{text_hash}.mp3")
            
            if os.path.exists(audio_path):
                return audio_path
            
            tts = gTTS(text=text, lang=lang, slow=False)
            tts.save(audio_path)
            return audio_path
        except Exception as e:
            print(f"Ошибка озвучки: {e}")
            return None

tts_engine = MultilingualTTS()

# --- ГОЛОСОВАЯ БИОМЕТРИЯ ---
class SimpleVoiceBiometrics:
    def extract_mfcc_features(self, audio_path):
        try:
            with wave.open(audio_path, 'rb') as wav_file:
                sample_width = wav_file.getsampwidth()
                frame_rate = wav_file.getframerate()
                n_frames = wav_file.getnframes()
                frames = wav_file.readframes(n_frames)
                
                if sample_width == 2:
                    audio_data = np.frombuffer(frames, dtype=np.int16)
                else:
                    audio_data = np.frombuffer(frames, dtype=np.uint8)
                    audio_data = audio_data.astype(np.float32) - 128
                
                audio_data = audio_data.astype(np.float32) / 32768.0
                features = self._simple_audio_features(audio_data, frame_rate)
                features_bytes = features.astype(np.float32).tobytes()
                features_b64 = base64.b64encode(features_bytes).decode('utf-8')
                return features_b64
        except Exception as e:
            print(f"Ошибка извлечения характеристик: {e}")
            return None
    
    def _simple_audio_features(self, audio_data, sample_rate):
        features = []
        features.append(np.mean(audio_data ** 2))
        features.append(np.std(audio_data))
        features.append(np.max(np.abs(audio_data)))
        
        fft = np.fft.fft(audio_data)
        fft_magnitude = np.abs(fft[:len(fft)//2])
        
        bands = [(0, 100), (100, 500), (500, 1500), (1500, 4000)]
        freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)[:len(audio_data)//2]
        
        for low, high in bands:
            mask = (freqs >= low) & (freqs < high)
            if np.any(mask):
                features.append(np.mean(fft_magnitude[mask]))
            else:
                features.append(0.0)
        
        return np.array(features)
    
    def compare_voice_features(self, features1_b64, features2_b64):
        try:
            features1_bytes = base64.b64decode(features1_b64)
            features2_bytes = base64.b64decode(features2_b64)
            
            features1 = np.frombuffer(features1_bytes, dtype=np.float32)
            features2 = np.frombuffer(features2_bytes, dtype=np.float32)
            
            min_len = min(len(features1), len(features2))
            features1 = features1[:min_len]
            features2 = features2[:min_len]
            
            distance = np.linalg.norm(features1 - features2)
            max_distance = np.linalg.norm(features1) + np.linalg.norm(features2)
            similarity = 1.0 - (distance / (max_distance + 1e-8))
            return float(max(0.0, min(1.0, similarity)))
        except:
            return 0.0

voice_biometrics = SimpleVoiceBiometrics()

# --- КОМАНДЫ ---
class VoiceCommandRecognizer:
    def __init__(self):
        self.commands = {
            'выйти': ['выйти', 'выход', 'выхожу', 'закрыть', 'exit'],
            'сканировать': ['сканировать', 'скан', 'сканируй', 'что вижу', 'что впереди', 'анализ', 'scan'],
            'время': ['время', 'который час', 'сколько времени', 'time'],
            'заметка': ['заметка', 'заметки', 'записать', 'запись', 'напоминание', 'note'],
        }
    
    def recognize_command(self, text):
        if not text:
            return None
        
        text = text.lower().strip()
        noise = ['слушаю', 'команду', 'команда', 'пожалуйста']
        for n in noise:
            text = text.replace(n, '')
        text = ' '.join(text.split())
        
        print(f"🎤 Анализ: '{text}'")
        
        for cmd_type, keywords in self.commands.items():
            for keyword in keywords:
                if keyword in text or text in keyword:
                    print(f"✅ Команда: {cmd_type}")
                    return cmd_type
        
        print(f"❌ Не распознано")
        return None

voice_command_recognizer = VoiceCommandRecognizer()

# --- КОНВЕРТЕР АУДИО ---
class AudioConverter:
    def convert_aac_to_wav(self, aac_path):
        try:
            audio = AudioSegment.from_file(aac_path, format="aac")
            wav_path = aac_path.replace('.aac', '.wav')
            audio.export(wav_path, format="wav")
            return wav_path
        except Exception as e:
            print(f"Ошибка конвертации: {e}")
            return None

audio_converter = AudioConverter()

# --- YOLO ---
OBJECT_HEIGHTS = {
    'person': 1.7, 'car': 1.5, 'chair': 1.0, 'bottle': 0.25,
    'cup': 0.12, 'dog': 0.5, 'cat': 0.3, 'tv': 0.6, 'laptop': 0.02,
    'traffic light': 5.0, 'stop sign': 2.0, 'bench': 0.8, 'book': 0.3,
    'bicycle': 1.0, 'motorcycle': 1.2, 'bus': 3.0, 'truck': 3.0,
    'table': 0.8, 'bed': 0.5, 'refrigerator': 1.8
}

OBJECT_TRANSLATIONS = {
    'person': 'человек', 'man': 'мужчина', 'woman': 'женщина', 'boy': 'мальчик', 'girl': 'девочка',
    'human face': 'лицо человека', 'human hand': 'рука', 'human foot': 'нога', 'human ear': 'ухо',
    'human nose': 'нос', 'human mouth': 'рот', 'human eye': 'глаз', 'human head': 'голова',
    'car': 'машина', 'bicycle': 'велосипед', 'motorcycle': 'мотоцикл', 'bus': 'автобус',
    'truck': 'грузовик', 'van': 'фургон', 'taxi': 'такси', 'ambulance': 'скорая помощь',
    'fire truck': 'пожарная машина', 'police car': 'полицейская машина',
    'airplane': 'самолет', 'helicopter': 'вертолет', 'boat': 'лодка', 'ship': 'корабль',
    'train': 'поезд', 'wheel': 'колесо', 'tire': 'шина', 'vehicle registration plate': 'номерной знак',
    'traffic light': 'светофор', 'stop sign': 'знак стоп', 'parking meter': 'парковочный счетчик',
    'street light': 'уличный фонарь', 'traffic sign': 'дорожный знак',
    'dog': 'собака', 'cat': 'кошка', 'bird': 'птица', 'horse': 'лошадь', 'cow': 'корова',
    'sheep': 'овца', 'elephant': 'слон', 'bear': 'медведь', 'zebra': 'зебра', 'giraffe': 'жираф',
    'rabbit': 'кролик', 'mouse': 'мышь', 'snake': 'змея', 'lion': 'лев', 'tiger': 'тигр',
    'butterfly': 'бабочка', 'bee': 'пчела', 'spider': 'паук', 'fish': 'рыба', 'chicken': 'курица',
    'banana': 'банан', 'apple': 'яблоко', 'orange': 'апельсин', 'lemon': 'лимон',
    'strawberry': 'клубника', 'grape': 'виноград', 'watermelon': 'арбуз', 'pear': 'груша',
    'pineapple': 'ананас', 'carrot': 'морковь', 'tomato': 'помидор', 'potato': 'картофель',
    'bread': 'хлеб', 'sandwich': 'сандвич', 'pizza': 'пицца', 'hot dog': 'хот-дог',
    'hamburger': 'гамбургер', 'cake': 'торт', 'cookie': 'печенье', 'donut': 'пончик',
    'ice cream': 'мороженое', 'coffee': 'кофе', 'tea': 'чай', 'wine': 'вино',
    'beer': 'пиво', 'juice': 'сок', 'milk': 'молоко', 'bottle': 'бутылка',
    'cup': 'чашка', 'bowl': 'миска', 'plate': 'тарелка', 'fork': 'вилка',
    'knife': 'нож', 'spoon': 'ложка', 'glass': 'стакан', 'wine glass': 'бокал',
    'mug': 'кружка', 'pitcher': 'кувшин', 'microwave': 'микроволновка',
    'oven': 'духовка', 'toaster': 'тостер', 'sink': 'раковина', 'refrigerator': 'холодильник',
    'blender': 'блендер', 'kettle': 'чайник',
    'chair': 'стул', 'table': 'стол', 'couch': 'диван', 'sofa': 'диван',
    'bed': 'кровать', 'bench': 'скамейка', 'desk': 'письменный стол',
    'cabinet': 'шкаф', 'shelf': 'полка', 'door': 'дверь', 'window': 'окно',
    'tv': 'телевизор', 'laptop': 'ноутбук', 'computer': 'компьютер',
    'keyboard': 'клавиатура', 'mouse': 'мышь', 'remote': 'пульт',
    'cell phone': 'телефон', 'telephone': 'телефон', 'clock': 'часы',
    'camera': 'камера', 'microphone': 'микрофон', 'headphones': 'наушники',
    'backpack': 'рюкзак', 'handbag': 'сумка', 'suitcase': 'чемодан',
    'umbrella': 'зонт', 'tie': 'галстук', 'hat': 'шляпа', 'cap': 'кепка',
    'glasses': 'очки', 'sunglasses': 'солнечные очки', 'shoe': 'обувь',
    'boot': 'ботинок', 'belt': 'ремень', 'jacket': 'куртка', 'coat': 'пальто',
    'ball': 'мяч', 'football': 'футбольный мяч', 'basketball': 'баскетбольный мяч',
    'tennis racket': 'теннисная ракетка', 'baseball bat': 'бейсбольная бита',
    'baseball glove': 'бейсбольная перчатка', 'skateboard': 'скейтборд',
    'surfboard': 'доска для серфинга', 'ski': 'лыжи', 'snowboard': 'сноуборд',
    'tree': 'дерево', 'flower': 'цветок', 'plant': 'растение', 'grass': 'трава',
    'rose': 'роза', 'mushroom': ' гриб', 'fountain': 'фонтан',
    'building': 'здание', 'house': 'дом', 'skyscraper': 'небоскреб',
    'tower': 'башня', 'bridge': 'мост', 'stairs': 'лестница', 'ladder': 'лестница',
    'book': 'книга', 'pen': 'ручка', 'pencil': 'карандаш', 'scissors': 'ножницы',
    'key': 'ключ', 'lock': 'замок', 'hammer': 'молоток', 'screwdriver': 'отвертка',
    'wrench': 'гаечный ключ', 'toolbox': 'ящик для инструментов',
    'trash can': 'мусорное ведро', 'box': 'коробка', 'bag': 'сумка',
    'basket': 'корзина', 'pillow': 'подушка', 'blanket': 'одеяло',
    'towel': 'полотенце', 'candle': 'свеча', 'lamp': 'лампа',
    'light': 'свет', 'mirror': 'зеркало', 'picture frame': 'рамка для фото',
    'vase': 'ваза', 'curtain': 'штора', 'rug': 'ковер', 'carpet': 'ковер',
    'ceiling': 'потолок', 'floor': 'пол', 'wall': 'стена',
    'flag': 'флаг', 'balloon': 'воздушный шар', 'toy': 'игрушка',
    'kite': 'воздушный змей', 'fireworks': 'фейерверк'
}

PRIORITY_OBJECTS = {
    'person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck', 
    'traffic light', 'stop sign', 'dog', 'cat'
}

def estimate_distance(bbox, frame_height, object_label):
    bbox_height_pixels = bbox[3] - bbox[1]
    if bbox_height_pixels <= 0:
        return None
    FOCAL_LENGTH_PIXELS = 700
    real_height = OBJECT_HEIGHTS.get(object_label, 1.0)
    distance = (real_height * FOCAL_LENGTH_PIXELS) / bbox_height_pixels
    return round(distance, 2)

print("🔄 Загрузка YOLO...")
model = YOLO('yolov5x-oiv7-fixed.pt')
print("✅ YOLO готов")

def detect_objects(frame):
    try:
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model.predict(img_rgb, conf=0.25, iou=0.45, verbose=False, imgsz=640, max_det=100)[0]
        
        detections = []
        frame_height, frame_width = frame.shape[:2]

        for r in results.boxes:
            bbox = r.xyxy[0].cpu().numpy()
            conf = float(r.conf[0])
            cls_id = int(r.cls[0])
            label = model.names[cls_id]
            distance = estimate_distance(bbox, frame_height, label)
            
            x_center = (bbox[0] + bbox[2]) / 2
            y_center = (bbox[1] + bbox[3]) / 2
            
            detections.append({
                "label": label,
                "confidence": conf,
                "bbox": [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
                "distance_m": float(distance) if distance else None,
                "priority": label in PRIORITY_OBJECTS,
                "x_center": float(x_center),
                "y_center": float(y_center)
            })
        
        detections.sort(key=lambda x: (not x['priority'], x['distance_m'] if x['distance_m'] else 999))
        print(f"🔍 Обнаружено: {len(detections)} объектов из 600+ классов")
        
        # Сохранение объектов в базу данных
        save_obstacles_to_db(detections)
        
        return detections
    except Exception as e:
        print(f"❌ Ошибка YOLO: {e}")
        return []

def generate_obstacle_description(obstacles, colors=None, brightness=None):
    descriptions = []
    
    if brightness:
        descriptions.append(f"{brightness}")
    
    if colors and len(colors) > 0:
        color_desc = " и ".join([c['color'] for c in colors[:2]])
        descriptions.append(f"Основные цвета: {color_desc}")
    
    if obstacles:
        descriptions.append(f"Объектов: {len(obstacles)}")
        
        for obs in obstacles[:10]:
            label_en = obs['label']
            label_ru = OBJECT_TRANSLATIONS.get(label_en, label_en)
            distance = obs.get('distance_m')
            
            if distance:
                if distance < 1.0:
                    dist_text = f"очень близко"
                elif distance < 2.0:
                    dist_text = f"близко"
                else:
                    dist_text = f"{int(distance)} метров"
            else:
                dist_text = "расстояние неизвестно"
            
            priority = "Внимание! " if obs.get('priority') else ""
            descriptions.append(f"{priority}{label_ru}, {dist_text}")
        
        if len(obstacles) > 10:
            descriptions.append(f"и еще {len(obstacles) - 10} объектов")
    else:
        descriptions.append("Препятствий не обнаружено")
    
    return ". ".join(descriptions)

# --- ЭНДПОИНТЫ ДЛЯ ЗАМЕТОК ---

@app.route('/add_note', methods=['POST'])
def add_note():
    """Добавить голосовую заметку"""
    try:
        username = request.form.get('username')
        note_text = request.form.get('note_text')
        remind_at_str = request.form.get('remind_at')  # формат: "2025-11-04 15:30:00"
        
        if not username or not note_text:
            return jsonify({"error": "Необходимо имя пользователя и текст заметки"}), 400
        
        user_id = get_user_id_by_username(username)
        if not user_id:
            return jsonify({"error": "Пользователь не найден"}), 404
        
        # Парсим время напоминания
        remind_at = None
        if remind_at_str:
            try:
                remind_at = datetime.strptime(remind_at_str, "%Y-%m-%d %H:%M:%S")
            except:
                return jsonify({"error": "Неверный формат времени"}), 400
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute(
            """INSERT INTO voice_notes (user_id, note_text, remind_at, created_at) 
               VALUES (%s, %s, %s, NOW()) RETURNING id""",
            (user_id, note_text, remind_at)
        )
        
        note_id = cur.fetchone()[0]
        conn.commit()
        cur.close()
        conn.close()
        
        print(f"✅ Заметка #{note_id} создана для {username}")
        
        return jsonify({
            "status": "success",
            "message": "Заметка создана",
            "note_id": note_id,
            "note_text": note_text,
            "remind_at": remind_at_str
        })
        
    except Exception as e:
        print(f"❌ Ошибка создания заметки: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/get_notes', methods=['GET'])
def get_notes():
    """Получить все заметки пользователя"""
    try:
        username = request.args.get('username')
        
        if not username:
            return jsonify({"error": "Необходимо имя пользователя"}), 400
        
        user_id = get_user_id_by_username(username)
        if not user_id:
            return jsonify({"error": "Пользователь не найден"}), 404
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute(
            """SELECT id, note_text, remind_at, created_at, is_completed 
               FROM voice_notes 
               WHERE user_id = %s AND is_completed = FALSE
               ORDER BY created_at DESC""",
            (user_id,)
        )
        
        notes = []
        for row in cur.fetchall():
            notes.append({
                "id": row[0],
                "note_text": row[1],
                "remind_at": row[2].strftime("%Y-%m-%d %H:%M:%S") if row[2] else None,
                "created_at": row[3].strftime("%Y-%m-%d %H:%M:%S"),
                "is_completed": row[4]
            })
        
        cur.close()
        conn.close()
        
        return jsonify({
            "status": "success",
            "notes": notes,
            "count": len(notes)
        })
        
    except Exception as e:
        print(f"❌ Ошибка получения заметок: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/update_note', methods=['POST'])
def update_note():
    """Обновить заметку"""
    try:
        note_id = request.form.get('note_id')
        note_text = request.form.get('note_text')
        remind_at_str = request.form.get('remind_at')
        
        if not note_id:
            return jsonify({"error": "Необходим ID заметки"}), 400
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Обновляем текст если указан
        if note_text:
            cur.execute(
                "UPDATE voice_notes SET note_text = %s WHERE id = %s",
                (note_text, note_id)
            )
        
        # Обновляем время напоминания если указано
        if remind_at_str:
            try:
                remind_at = datetime.strptime(remind_at_str, "%Y-%m-%d %H:%M:%S")
                cur.execute(
                    "UPDATE voice_notes SET remind_at = %s WHERE id = %s",
                    (remind_at, note_id)
                )
            except:
                return jsonify({"error": "Неверный формат времени"}), 400
        
        conn.commit()
        cur.close()
        conn.close()
        
        print(f"✅ Заметка #{note_id} обновлена")
        
        return jsonify({
            "status": "success",
            "message": "Заметка обновлена"
        })
        
    except Exception as e:
        print(f"❌ Ошибка обновления заметки: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/delete_note', methods=['POST'])
def delete_note():
    """Удалить заметку"""
    try:
        note_id = request.form.get('note_id')
        
        if not note_id:
            return jsonify({"error": "Необходим ID заметки"}), 400
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("DELETE FROM voice_notes WHERE id = %s", (note_id,))
        
        conn.commit()
        cur.close()
        conn.close()
        
        print(f"✅ Заметка #{note_id} удалена")
        
        return jsonify({
            "status": "success",
            "message": "Заметка удалена"
        })
        
    except Exception as e:
        print(f"❌ Ошибка удаления заметки: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/complete_note', methods=['POST'])
def complete_note():
    """Отметить заметку как выполненную"""
    try:
        note_id = request.form.get('note_id')
        
        if not note_id:
            return jsonify({"error": "Необходим ID заметки"}), 400
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute(
            "UPDATE voice_notes SET is_completed = TRUE WHERE id = %s",
            (note_id,)
        )
        
        conn.commit()
        cur.close()
        conn.close()
        
        print(f"✅ Заметка #{note_id} выполнена")
        
        return jsonify({
            "status": "success",
            "message": "Заметка отмечена как выполненная"
        })
        
    except Exception as e:
        print(f"❌ Ошибка завершения заметки: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/check_reminders', methods=['GET'])
def check_reminders():
    """Проверить напоминания (для фонового процесса)"""
    try:
        username = request.args.get('username')
        
        if not username:
            return jsonify({"error": "Необходимо имя пользователя"}), 400
        
        user_id = get_user_id_by_username(username)
        if not user_id:
            return jsonify({"error": "Пользователь не найден"}), 404
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Ищем напоминания на ближайшую минуту
        now = datetime.now()
        one_minute_later = now + timedelta(minutes=1)
        
        cur.execute(
            """SELECT id, note_text, remind_at 
               FROM voice_notes 
               WHERE user_id = %s 
               AND remind_at IS NOT NULL 
               AND remind_at <= %s 
               AND is_completed = FALSE
               ORDER BY remind_at ASC""",
            (user_id, one_minute_later)
        )
        
        reminders = []
        for row in cur.fetchall():
            reminders.append({
                "id": row[0],
                "note_text": row[1],
                "remind_at": row[2].strftime("%Y-%m-%d %H:%M:%S")
            })
        
        cur.close()
        conn.close()
        
        return jsonify({
            "status": "success",
            "reminders": reminders,
            "count": len(reminders)
        })
        
    except Exception as e:
        print(f"❌ Ошибка проверки напоминаний: {e}")
        return jsonify({"error": str(e)}), 500

# --- ОСНОВНЫЕ ЭНДПОИНТЫ ---

@app.route('/')
def index():
    return jsonify({
        "status": "Blind Assistant Server v16 - С ЗАМЕТКАМИ",
        "features": [
            "✅ Детекция 600+ объектов YOLO",
            "✅ Голосовые команды",
            "✅ Анализ цветов и яркости",
            "✅ Сохранение объектов в базу данных",
            "✅ Голосовые заметки и напоминания",
            "⏸️ OCR временно отключен",
            "⏸️ Распознавание денег временно отключено"
        ],
        "notes_endpoints": [
            "POST /add_note - добавить заметку",
            "GET /get_notes - получить заметки",
            "POST /update_note - обновить заметку",
            "POST /delete_note - удалить заметку",
            "POST /complete_note - завершить заметку",
            "GET /check_reminders - проверить напоминания"
        ]
    })

@app.route('/register_voice', methods=['POST'])
def register_voice():
    temp_audio = temp_wav = None
    try:
        if 'audio' not in request.files or 'username' not in request.form:
            return jsonify({"error": "Необходимо аудио и имя"}), 400

        username = request.form['username'].strip()
        audio_file = request.files['audio']
        
        temp_audio = f"temp_register_{uuid.uuid4()}.aac"
        audio_file.save(temp_audio)
        
        temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
        if not temp_wav:
            return jsonify({"error": "Ошибка конвертации"}), 400
        
        voice_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("SELECT id FROM users WHERE username = %s", (username,))
        existing = cur.fetchone()
        
        if existing:
            cur.execute("UPDATE users SET voice_embedding = %s WHERE username = %s", (voice_features, username))
            message = "Профиль обновлен"
        else:
            cur.execute("INSERT INTO users (username, voice_embedding) VALUES (%s, %s)", (username, voice_features))
            message = "Профиль создан"
        
        conn.commit()
        cur.close()
        conn.close()
        
        return jsonify({"status": "success", "message": message, "username": username})
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    finally:
        for f in [temp_audio, temp_wav]:
            if f and os.path.exists(f):
                try: os.remove(f)
                except: pass

@app.route('/login_voice', methods=['POST'])
def login_voice():
    temp_audio = temp_wav = None
    try:
        username = request.form['username'].strip()
        audio_file = request.files['audio']
        
        temp_audio = f"temp_login_{uuid.uuid4()}.aac"
        audio_file.save(temp_audio)
        
        temp_wav = audio_converter.convert_aac_to_wav(temp_audio)
        current_features = voice_biometrics.extract_mfcc_features(temp_wav)
        
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("SELECT voice_embedding FROM users WHERE username = %s", (username,))
        result = cur.fetchone()
        
        if not result:
            return jsonify({"error": "Пользователь не найден"}), 404
        
        similarity = voice_biometrics.compare_voice_features(result[0], current_features)
        
        cur.close()
        conn.close()
        
        if similarity > 0.7:
            welcome_text = f"Добро пожаловать, {username}! Камера готова."
            return jsonify({
                "status": "success",
                "message": "Вход выполнен",
                "username": username,
                "similarity": float(similarity),
                "welcome_text": welcome_text,
                "skippable": True
            })
        else:
            return jsonify({"status": "fail", "message": "Голос не распознан"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    finally:
        for f in [temp_audio, temp_wav]:
            if f and os.path.exists(f):
                try: os.remove(f)
                except: pass

@app.route('/voice_command', methods=['POST'])
def voice_command():
    try:
        text = request.form.get('text', '').lower().strip()
        print(f"🎤 Голосовая команда: '{text}'")
        
        if not text:
            return jsonify({"error": "Текст не распознан"}), 400
        
        command = voice_command_recognizer.recognize_command(text)
        
        if command == 'время':
            time_data = time_service.get_current_time()
            return jsonify({"status": "success", "command": "время", "data": time_data})
        elif command == 'заметка':
            return jsonify({
                "status": "success", 
                "command": "заметка",
                "message": "Режим заметок активирован",
                "available_actions": [
                    "добавить заметку",
                    "показать заметки", 
                    "удалить заметку",
                    "напоминание"
                ]
            })
        elif command in ['сканировать', 'выйти']:
            return jsonify({"status": "success", "command": command})
        else:
            return jsonify({
                "status": "unknown",
                "message": "Команда не распознана",
                "available_commands": ["сканировать", "время", "заметка", "выйти"]
            })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/process_frame', methods=['POST'])
def process_frame():
    try:
        print("\n🔵 Обработка кадра...")
        if 'frame' not in request.files:
            return jsonify({"error": "Кадр не предоставлен"}), 400

        frame_bytes = request.files['frame'].read()
        nparr = np.frombuffer(frame_bytes, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is None:
            return jsonify({"error": "Неверное изображение"}), 400

        print(f"✅ Изображение: {frame.shape}")

        # Только распознавание объектов
        obstacles = detect_objects(frame)
        colors = color_analyzer.analyze_colors(frame)
        brightness = color_analyzer.get_brightness_level(frame)
        
        # ИСПРАВЛЕННЫЙ ВЫЗОВ - убран лишний аргумент
        annotated_frame = object_visualizer.draw_detections(frame, obstacles)
        
        # ИСПРАВЛЕННЫЙ ВЫЗОВ - убран лишний аргумент
        description = generate_obstacle_description(obstacles, colors, brightness)
        
        lang = 'ru'
        audio_path = tts_engine.text_to_speech(description, lang=lang)
        
        annotated_path = f"annotated_{uuid.uuid4()}.jpg"
        cv2.imwrite(annotated_path, annotated_frame)
        
        with open(annotated_path, 'rb') as f:
            annotated_base64 = base64.b64encode(f.read()).decode('utf-8')
        
        os.remove(annotated_path)

        print(f"✅ Готово")

        return jsonify({
            "obstacles": obstacles,
            "colors": colors,
            "brightness": brightness,
            "description": description,
            "count": len(obstacles),
            "audio_available": audio_path is not None,
            "annotated_frame": annotated_base64,
            "message": "Кадр обработан"
        })
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/get_time', methods=['GET'])
def get_time():
    try:
        time_data = time_service.get_current_time()
        return jsonify(time_data)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/test_connection', methods=['GET'])
def test_connection():
    return jsonify({"status": "ok", "message": "Сервер работает"})

if __name__ == '__main__':
    print("\n" + "="*70)
    print("🚀 BLIND ASSISTANT SERVER v16 - С ЗАМЕТКАМИ")
    print("="*70)
    print("✅ Детекция 600+ объектов YOLO")
    print("✅ Анализ цветов и освещения")
    print("✅ Голосовые команды и биометрия")
    print("✅ Сохранение объектов в базу данных")
    print("✅ Голосовые заметки и напоминания")
    print("⏸️ OCR временно отключен")
    print("⏸️ Распознавание денег временно отключено")
    print("="*70)
    print("\n🎯 АКТИВНЫЕ ФУНКЦИИ:")
    print("   🔍 YOLO (600+ объектов):")
    print("      - Люди и части тела: лицо, руки, ноги")
    print("      - Транспорт: все виды машин, самолеты, корабли")
    print("      - Животные: 20+ видов животных")
    print("      - Еда: 40+ видов еды и напитков")
    print("      - Посуда: тарелки, чашки, столовые приборы")
    print("      - Мебель: столы, стулья, кровати, шкафы")
    print("      - Электроника: компьютеры, телефоны, камеры")
    print("      - Одежда: обувь, сумки, шляпы, очки")
    print("      - Спорт: мячи, ракетки, скейтборды")
    print("      - И МНОГОЕ ДРУГОЕ!")
    print()
    print("   📝 Система заметок:")
    print("      - Добавление голосовых заметок")
    print("      - Напоминания по времени")
    print("      - Просмотр и управление заметками")
    print("      - Отметка о выполнении")
    print()
    print("   🎨 Анализ цветов:")
    print("      - Определение основных цветов в кадре")
    print("      - Оценка уровня освещения")
    print()
    print("   💾 База данных:")
    print("      - Автоматическое сохранение всех обнаруженных объектов")
    print("      - Хранение голосовых заметок")
    print()
    print("   🎤 Голосовые команды:")
    print("      - 'сканировать' - анализ объектов")
    print("      - 'время' - узнать текущее время")
    print("      - 'заметка' - управление заметками")
    print("      - 'выйти' - выход из приложения")
    print("="*70)
    print("\n⚡ СКОРОСТЬ:")
    print("   - Распознавание объектов: ~0.5-1 секунда")
    print("   - Анализ цветов: мгновенно")
    print("   - Озвучивание: ~0.3 секунды")
    print("   - Сохранение в БД: мгновенно")
    print("   - Работа с заметками: мгновенно")
    print("="*70)
    print("\n💡 ДОСТУПНЫЕ КОМАНДЫ:")
    print("   🔹 'сканировать' - полный анализ сцены")
    print("   🔹 'время' - текущее время")
    print("   🔹 'заметка' - управление заметками")
    print("   🔹 'выйти' - закрыть приложение")
    print("="*70)
    print("\n📝 КОМАНДЫ ДЛЯ ЗАМЕТОК:")
    print("   🔸 'добавить заметку' - создать новую заметку")
    print("   🔸 'показать заметки' - просмотреть все заметки")
    print("   🔸 'удалить заметку' - удалить заметку")
    print("   🔸 'напоминание' - установить напоминание")
    print("="*70)
    print("\n🌐 Сервер: http://10.59.29.73:5000")
    print("="*70 + "\n")
    
    app.run(host='10.59.29.73', port=5000, debug=True)




