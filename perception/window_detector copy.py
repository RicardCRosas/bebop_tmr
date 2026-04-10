#!/usr/bin/env python3

import os
from ultralytics import YOLO
import cv2
import numpy as np
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List

current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, 'models', 'best.pt')
print("Loading model YOLO from:", model_path)

if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model was not found in the directory: {model_path}")

class BebopCameraProcessor:
    def __init__(self):
        print(f"Loading model YOLO from: {model_path}")

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.model = YOLO(model_path)

        # Centro de la imagen (lo ajustas dinámicamente luego)
        self.center = (320, 240)

        # rgb(223, 75, 24) naranja
        # Color objetivo (amarillo/verde que pusiste) 166, 169, 14
        self.r, self.g, self.b = 166, 169, 14
        self.tolerancia = 10

    def filtropasa_rgb(self, frame):
        color_bgr = np.uint8([[[self.b, self.g, self.r]]])
        hsv_target = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]

        bajo = np.array([max(0, hsv_target[0] - self.tolerancia), 50, 50])
        alto = np.array([min(180, hsv_target[0] + self.tolerancia), 255, 255])

        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), bajo, alto)
        return cv2.bitwise_and(frame, frame, mask=mask)

    def detect_with_yolo(self, image):
        results = self.model.predict(source=image, conf=0.5, verbose=False)

        annotated = results[0].plot()

        best_conf = 0
        best_box = None
        cx = None

        if results[0].boxes is not None:
            boxes = results[0].boxes

            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()

            for box, conf in zip(xyxy, confs):
                if conf < 0.5:
                    continue

                x1, y1, x2, y2 = box

                # Buscar el de mayor confianza
                if conf > best_conf:
                    best_conf = conf
                    best_box = [int(x1), int(y1), int(x2), int(y2)]

        if best_box:
            x1, y1, x2, y2 = best_box

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(annotated, f"cx: {cx}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            detected = True
        else:
            detected = False

        # Dibujar centro de imagen
        cv2.circle(annotated, self.center, 5, (255, 0, 0), -1)

        return annotated, detected, cx

    def process_image(self, frame):
        # Ajustar centro dinámicamente
        h, w = frame.shape[:2]
        self.center = (w // 2, h // 2)

        # 1️⃣ Filtro de color
        filtered = self.filtropasa_rgb(frame)

        # 2️⃣ YOLO sobre imagen filtrada
        annotated, detected, cx = self.detect_with_yolo(filtered)

        data = {
            "detected": detected,
            "cx": cx
        }

        return annotated, data, filtered

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # o video.mp4
    #"videos/GH0000.mp4"

    detector = BebopCameraProcessor() 

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        processed, data, filtered = detector.process_image(frame)

        # Mostrar TODO para debug
        cv2.imshow("Original", frame)
        cv2.imshow("Filtro Color", filtered)
        cv2.imshow("YOLO Result", processed)

        if data["detected"]:
            print("Centro detectado:", data["cx"])

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()