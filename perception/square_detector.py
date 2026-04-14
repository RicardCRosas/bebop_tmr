import cv2
import numpy as np
import rospy

class BebopCameraProcessor:

    def __init__(self):
        # Color objetivo (RGB)
        self.r, self.g, self.b = 106, 174, 239
        self.tolerancia = 15

        # Parámetros de detección
        self.area_minima = 650

    # =====================================================
    # FILTRO POR COLOR (HSV)
    # =====================================================
    def filtropasa_rgb(self, frame):

        color_bgr = np.uint8([[[self.b, self.g, self.r]]])
        hsv_target = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]

        bajo = np.array([
            max(0, hsv_target[0] - self.tolerancia),
            50,
            50
        ])

        alto = np.array([
            min(180, hsv_target[0] + self.tolerancia),
            255,
            255
        ])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mascara = cv2.inRange(hsv, bajo, alto)

        return cv2.bitwise_and(frame, frame, mask=mascara)

    # =====================================================
    # DETECCIÓN DE CUADRADOS
    # =====================================================
    def detectar_cuadrados(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (11, 11), 0)

        canny = cv2.Canny(gray, 10, 150)

        kernel = np.ones((5, 5), np.uint8)
        canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

        contornos, _ = cv2.findContours(
            canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        output = frame.copy()
        candidatos = []  # (cx, cy, area, aspect_ratio)

        for c in contornos:
            area = cv2.contourArea(c)
            if area < self.area_minima:
                continue

            hull = cv2.convexHull(c)
            epsilon = 0.1 * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, epsilon, True)

            if 4 <= len(approx) <= 6:

                x, y, w, h = cv2.boundingRect(approx)
                if h == 0:
                    continue

                aspect_ratio = float(w) / h
                rect_area = w * h
                extent = area / rect_area

                if 0.75 <= aspect_ratio <= 1.25 and extent > 0.6:

                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx = x + w // 2
                        cy = y + h // 2

                    candidatos.append((cx, cy, area, aspect_ratio))

                    # Dibujar contorno
                    cv2.drawContours(output, [approx], -1, (0, 255, 0), 2)
                    cv2.circle(output, (cx, cy), 4, (255, 0, 0), -1)

        return output, candidatos

    # =====================================================
    # PROCESAMIENTO PRINCIPAL
    # =====================================================
    def process_image(self, frame):

        # Filtrar por color
        frame_filtrado = self.filtropasa_rgb(frame)

        # Detectar cuadrados
        output, candidatos = self.detectar_cuadrados(frame_filtrado)

        h, w = frame.shape[:2]
        center_x = w // 2
        center_y = h // 2

        # Inicializar estructura de salida
        data = {
            "detected": False,
            "cx": None,
            "cy": None,
            "center_x": center_x,
            "center_y": center_y,
            "size": None
        }

        # Si hay candidatos válidos
        if len(candidatos) > 0:

            # Elegir el más grande
            candidatos.sort(key=lambda x: x[2], reverse=True)
            cx, cy, area, ar = candidatos[0]

            data["detected"] = True
            data["cx"] = cx
            data["cy"] = cy
            data["size"] = area

            # Dibujar centro objetivo
            cv2.circle(output, (cx, cy), 8, (0, 0, 255), -1)

            rospy.loginfo(f"Ventana detectada | Área: {area} | Centro: ({cx}, {cy})")

            # Info en pantalla
            cv2.putText(output, f"Area: {int(area)}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.putText(output, f"AR: {ar:.2f}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        else:
            cv2.putText(output, "NO DETECTADO", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Dibujar centro de la imagen
        cv2.circle(output, (center_x, center_y), 5, (255, 255, 255), -1)

        return output, data, None