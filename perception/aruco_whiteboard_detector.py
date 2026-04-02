#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np


class ArucoWhiteboardDetector:
    """
    Detector del ArUco asociado al pizarrón.

    Salida:
      - detected
      - marker_id
      - corners
      - center_x, center_y
      - cx, cy
      - error_x, error_y
      - bbox_w, bbox_h
      - area
      - target_draw_x, target_draw_y

    Nota:
    El reglamento menciona un marcador 5x5 con ID 100.
    Aquí arrancamos con DICT_5X5_250, que es una opción razonable.
    Antes de competir, validen el diccionario con el marcador real impreso.
    """

    def __init__(self):
        self.target_id = 100
        self.marker_size_m = 0.20  # 20 cm

        # Compatibilidad OpenCV nuevo / viejo
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

    def _detect_markers(self, gray):
        # Compatibilidad OpenCV
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )
        return corners, ids, rejected

    def process_image(self, frame):
        h, w = frame.shape[:2]

        data = {
            "detected": False,
            "marker_id": None,
            "corners": None,
            "center_x": w // 2,
            "center_y": h // 2,
            "cx": None,
            "cy": None,
            "error_x": None,
            "error_y": None,
            "bbox_w": 0.0,
            "bbox_h": 0.0,
            "area": 0.0,
            "target_draw_x": None,
            "target_draw_y": None
        }

        output = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self._detect_markers(gray)

        cv2.circle(output, (data["center_x"], data["center_y"]), 5, (0, 0, 255), -1)
        cv2.putText(output, "img_center", (data["center_x"] + 8, data["center_y"] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        if ids is None or len(ids) == 0:
            cv2.putText(output, "No ArUco detected", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return output, data

        ids = ids.flatten()

        for i, marker_id in enumerate(ids):
            if int(marker_id) != self.target_id:
                continue

            pts = corners[i][0].astype(np.float32)

            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))

            x_min = float(np.min(pts[:, 0]))
            x_max = float(np.max(pts[:, 0]))
            y_min = float(np.min(pts[:, 1]))
            y_max = float(np.max(pts[:, 1]))

            bbox_w = x_max - x_min
            bbox_h = y_max - y_min
            area = bbox_w * bbox_h

            # Heurística para objetivo de trazo:
            # El ArUco está fuera del borde izquierdo del pizarrón y más arriba que el centro del pizarrón.
            # Entonces, para iniciar el trazo nos movemos "hacia la derecha" y "hacia abajo" en imagen.
            target_draw_x = int(cx + 1.3 * bbox_w)
            target_draw_y = int(cy + 0.9 * bbox_h)

            data["detected"] = True
            data["marker_id"] = int(marker_id)
            data["corners"] = pts
            data["cx"] = cx
            data["cy"] = cy
            data["error_x"] = cx - data["center_x"]
            data["error_y"] = cy - data["center_y"]
            data["bbox_w"] = bbox_w
            data["bbox_h"] = bbox_h
            data["area"] = area
            data["target_draw_x"] = target_draw_x
            data["target_draw_y"] = target_draw_y

            pts_int = pts.astype(int)
            cv2.polylines(output, [pts_int], True, (0, 255, 0), 2)
            cv2.circle(output, (cx, cy), 5, (255, 0, 0), -1)
            cv2.putText(output, f"id={marker_id}", (int(x_min), int(y_min) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.circle(output, (target_draw_x, target_draw_y), 6, (255, 255, 0), -1)
            cv2.putText(output, "draw_start_ref", (target_draw_x + 8, target_draw_y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

            cv2.putText(output, f"err_x={int(data['error_x'])}", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(output, f"err_y={int(data['error_y'])}", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(output, f"area={int(area)}", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            return output, data

        cv2.putText(output, "Target ArUco not found", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        return output, data