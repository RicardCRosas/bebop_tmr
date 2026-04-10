#!/usr/bin/env python3

import cv2
import numpy as np


class BebopCameraProcessor:

    def __init__(self):
        # Color objetivo
        self.r, self.g, self.b = 30, 62, 101
        self.tolerancia = 25

    # =========================
    # FILTRO COLOR
    # =========================
    def filtropasa_rgb(self, frame):

        color_bgr = np.uint8([[[self.b, self.g, self.r]]])
        hsv_target = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]
        
        bajo = np.array([max(0, hsv_target[0] - self.tolerancia), 50, 50])
        alto = np.array([min(180, hsv_target[0] + self.tolerancia), 255, 255])
        
        mascara = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), bajo, alto)
        return cv2.bitwise_and(frame, frame, mask=mascara)

    # =========================
    # DETECTOR + ANALISIS
    # =========================
    def detectar_cuadrados(self, frame, area_minima=500):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (15, 15), 0)

        canny = cv2.Canny(gray, 10, 150)

        kernel = np.ones((5, 5), np.uint8)
        canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        output = frame.copy()
        candidatos = []  # (cx, cy, area, aspect_ratio)

        for c in contornos:
            area = cv2.contourArea(c)
            if area < area_minima:
                continue

            epsilon = 0.05 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)

            if 4 <= len(approx) <= 6:

                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h

                rect_area = w * h
                extent = area / rect_area

                if 0.75 <= aspect_ratio <= 1.25 and extent > 0.6:

                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx, cy = x + w // 2, y + h // 2

                    candidatos.append((cx, cy, area, aspect_ratio))

                    # Dibujar
                    cv2.drawContours(output, [approx], -1, (0, 255, 0), 2)
                    cv2.circle(output, (cx, cy), 4, (255, 0, 0), -1)

        # =========================
        # ANALISIS DE ALINEACION
        # =========================
        if len(candidatos) > 0:

            candidatos.sort(key=lambda x: x[2], reverse=True)
            top_n = min(3, len(candidatos))
            candidatos = candidatos[:top_n]

            errores = []

            for cx, cy, area, ar in candidatos:
                error = abs(ar - 1.0)
                errores.append(error)

                cv2.putText(output, f"{ar:.2f}", (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            error_prom = np.mean(errores)

            # Dibujar error global
            cv2.putText(output,
                        f"Error alineacion: {error_prom:.3f}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2)

            # 🔥 usar el más grande como referencia
            cx_out, cy_out = candidatos[0][0], candidatos[0][1]

            return output, cx_out, cy_out, error_prom

        return output, None, None, None

    # =========================
    # FUNCIÓN PRINCIPAL
    # =========================
    def process_image(self, frame):

        h, w, _ = frame.shape
        center_x = w // 2
        center_y = h // 2

        # 1. Filtrar color
        filtrado = self.filtropasa_rgb(frame)

        # 2. Detectar
        processed, cx, cy, error = self.detectar_cuadrados(filtrado)

        # 3. Construir salida ROBUSTA
        if cx is not None:
            data = {
                "detected": True,
                "cx": cx,
                "cy": cy,
                "center_x": center_x,
                "center_y": center_y,
                "alignment_error": error
            }
        else:
            data = {
                "detected": False,
                "cx": None,
                "cy": None,
                "center_x": center_x,
                "center_y": center_y,
                "alignment_error": None
            }

        return processed, data, None