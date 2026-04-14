#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class OrangeTubeDetector:
    def __init__(self):
        rospy.init_node("orange_tube_detector")

        # Topics
        self.image_topic = rospy.get_param("~image_topic", "/bebop/image_raw")
        self.average_topic = rospy.get_param("~average_topic", "/average_color")
        self.bboxes_topic = rospy.get_param("~bboxes_topic", "/orange_bboxes")
        self.camera_topic = rospy.get_param("~camera_topic", "/bebop/camera_control")
        self.activation_topic = rospy.get_param("~activation_topic", "/activacion_dron")
        self.center_distance_topic = rospy.get_param("~center_distance_topic", "/tube_center_distance")

        # Visualización
        self.show_windows = rospy.get_param("~show_windows", True)

        # Tamaño de frame
        self.frame_w = rospy.get_param("~frame_w", 680)
        self.frame_h = rospy.get_param("~frame_h", 480)

        # Máximo número de tubos a reportar
        self.max_objects = rospy.get_param("~max_objects", 3)

        # Filtros geométricos
        self.min_area = rospy.get_param("~min_area", 3500)
        self.min_fill_ratio = rospy.get_param("~min_fill_ratio", 0.55)
        self.min_aspect_ratio = rospy.get_param("~min_aspect_ratio", 1.4) # h/w
        self.max_aspect_ratio = rospy.get_param("~max_aspect_ratio", 8.0)
        self.min_width = rospy.get_param("~min_width", 25)
        self.min_height = rospy.get_param("~min_height", 80)

        # HSV ajustado a naranja rojizo de la cámara del dron
        self.lower_orange = np.array(
            rospy.get_param("~lower_orange", [0, 85, 70]),
            dtype=np.uint8
        )
        self.upper_orange = np.array(
            rospy.get_param("~upper_orange", [18, 255, 255]),
            dtype=np.uint8
        )

        # Morfología
        open_k = rospy.get_param("~open_kernel", 5)
        close_k = rospy.get_param("~close_kernel", 11)
        self.open_kernel = np.ones((open_k, open_k), np.uint8)
        self.close_kernel = np.ones((close_k, close_k), np.uint8)

        self.bridge = CvBridge()

        # Estado de activación
        self.is_active = False

        # Publishers
        self.average_pub = rospy.Publisher(self.average_topic, Float32MultiArray, queue_size=1)
        self.bboxes_pub = rospy.Publisher(self.bboxes_topic, Int32MultiArray, queue_size=1)
        self.camera_pub = rospy.Publisher(self.camera_topic, Twist, queue_size=1)
        self.center_distance_pub = rospy.Publisher(self.center_distance_topic, Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        rospy.Subscriber(self.activation_topic, String, self.activation_callback, queue_size=1)

        rospy.sleep(1.0)
        self.set_camera_pose_zero()

        rospy.loginfo("orange_tube_detector iniciado en espera de READY en %s", self.activation_topic)

    def activation_callback(self, msg):
        command = msg.data.strip().upper()
        if command == "READY":
            if not self.is_active:
                self.is_active = True
                rospy.loginfo("orange_tube_detector ACTIVADO por mensaje READY")
        else:
            rospy.loginfo_throttle(2.0, "Esperando READY en %s. Recibido: %s", self.activation_topic, command)

    def set_camera_pose_zero(self):
        """
        Pone la cámara en pose 0:
        tilt = 0
        pan = 0
        """
        cam = Twist()
        cam.angular.y = 20.0 # tilt
        cam.angular.z = 0.0 # pan
        self.camera_pub.publish(cam)
        rospy.loginfo("Camara ajustada a pose 0")

    def get_orange_mask(self, hsv):
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # Quitar ruido fino
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel, iterations=1)

        # Unir regiones cercanas del mismo tubo
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_kernel, iterations=2)

        # Suavizado extra
        mask = cv2.dilate(mask, self.close_kernel, iterations=1)
        mask = cv2.erode(mask, self.open_kernel, iterations=1)

        return mask

    def evaluate_contour_as_tube(self, cnt):
        area = cv2.contourArea(cnt)
        if area < self.min_area:
            return None

        x, y, w, h = cv2.boundingRect(cnt)
        if w <= 0 or h <= 0:
            return None

        if w < self.min_width or h < self.min_height:
            return None

        bbox_area = float(w * h)
        fill_ratio = area / bbox_area
        aspect_ratio = float(h) / float(w)

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        approx_vertices = len(approx)

        if fill_ratio < self.min_fill_ratio:
            return None

        if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
            return None

        shape_penalty = 1.0
        if approx_vertices > 8:
            shape_penalty = 0.75
        elif approx_vertices < 4:
            shape_penalty = 0.80

        score = area * fill_ratio * aspect_ratio * shape_penalty

        return {
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "area": area,
            "fill_ratio": fill_ratio,
            "aspect_ratio": aspect_ratio,
            "approx_vertices": approx_vertices,
            "score": score,
            "contour": cnt
        }

    def publish_bboxes(self, selected):
        """
        Formato nuevo:
        [x1, y1, x2, y2, x1, y1, x2, y2, ...]
        En el mismo orden que /average_color
        """
        out = []
        for obj in selected:
            x, y, w, h = obj["x"], obj["y"], obj["w"], obj["h"]
            out.extend([x, y, x + w, y + h])

        self.bboxes_pub.publish(Int32MultiArray(data=out))

    def compute_object_orange_percentage(self, mask_orange, contour):
        """
        Calcula el porcentaje de pixeles naranjas de ESTE tubo
        respecto al frame completo, sin cambiar la segmentación.
        """
        obj_mask = np.zeros_like(mask_orange)
        cv2.drawContours(obj_mask, [contour], -1, 255, thickness=-1)

        isolated = cv2.bitwise_and(mask_orange, obj_mask)
        orange_pixels = cv2.countNonZero(isolated)
        total_pixels = float(self.frame_w * self.frame_h)

        return 100.0 * float(orange_pixels) / total_pixels

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr_throttle(1.0, "CvBridge error: %s", str(e))
            return

        frame = cv2.resize(frame, (self.frame_w, self.frame_h))
        display = frame.copy()

        center_frame_x = self.frame_w // 2
        center_frame_y = self.frame_h // 2
        cv2.circle(display, (center_frame_x, center_frame_y), 6, (0, 0, 255), -1)

        if not self.is_active:
            cv2.putText(display, "ESPERANDO READY", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            if self.show_windows:
                black_mask = np.zeros((self.frame_h, self.frame_w), dtype=np.uint8)
                cv2.imshow("Bebop Orange Detection", display)
                cv2.imshow("Bebop Orange Mask", black_mask)
                cv2.waitKey(1)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_orange = self.get_orange_mask(hsv)

        contours, _ = cv2.findContours(
            mask_orange,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        candidates = []
        for cnt in contours:
            candidate = self.evaluate_contour_as_tube(cnt)
            if candidate is None:
                continue

            candidate["orange_percentage"] = self.compute_object_orange_percentage(
                mask_orange, candidate["contour"]
            )
            candidate["cx"] = candidate["x"] + candidate["w"] // 2
            candidate["cy"] = candidate["y"] + candidate["h"] // 2
            candidate["dx_center"] = abs(candidate["cx"] - center_frame_x)
            candidates.append(candidate)

        # Mantener la evaluación como antes
        candidates = sorted(candidates, key=lambda c: c["score"], reverse=True)
        candidates = candidates[:self.max_objects]

        # Orden final por porcentaje naranja descendente
        selected = sorted(candidates, key=lambda c: c["orange_percentage"], reverse=True)

        # Publicaciones alineadas
        if len(selected) > 0:
            avg_msg = Float32MultiArray()
            avg_msg.data = [obj["orange_percentage"] for obj in selected]
            self.average_pub.publish(avg_msg)
            self.publish_bboxes(selected)

            # Mantener este tópico como referencia del candidato principal
            self.center_distance_pub.publish(Float32(data=float(selected[0]["dx_center"])))
        else:
            self.average_pub.publish(Float32MultiArray(data=[]))
            self.bboxes_pub.publish(Int32MultiArray(data=[]))

        # Porcentaje total en pantalla
        total_orange_pixels = cv2.countNonZero(mask_orange)
        total_pixels = self.frame_w * self.frame_h
        total_orange_percentage = 100.0 * float(total_orange_pixels) / float(total_pixels)

        cv2.putText(display, f"Orange total %: {total_orange_percentage:.2f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display, f"Tubes candidates: {len(selected)}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        text_y = 85
        colors = [
            (0, 0, 255), # rojo
            (0, 255, 0), # verde
            (255, 255, 0) # cian
        ]

        for i, obj in enumerate(selected):
            x, y, w, h = obj["x"], obj["y"], obj["w"], obj["h"]
            cx, cy = obj["cx"], obj["cy"]
            color = colors[i % len(colors)]
            label = f"TUBE_{i+1}"

            cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
            cv2.circle(display, (cx, cy), 5, (255, 0, 0), -1)

            cv2.putText(display, label, (x, max(15, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            cv2.putText(
                display,
                f"P:{obj['orange_percentage']:.2f}% A:{int(obj['area'])}",
                (x, min(self.frame_h - 10, y + h + 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )

            cv2.putText(
                display,
                f"{label}_dx: {obj['dx_center']}px",
                (10, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color,
                2
            )
            text_y += 28

            cv2.putText(
                display,
                f"{label}_BBOX: ({x},{y}) ({x+w},{y+h})",
                (10, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                2
            )
            text_y += 24

        if len(selected) == 0:
            cv2.putText(display, "NO TUBES", (10, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if self.show_windows:
            cv2.imshow("Bebop Orange Detection", display)
            cv2.imshow("Bebop Orange Mask", mask_orange)
            cv2.waitKey(1)


if __name__ == "__main__":
    try:
        OrangeTubeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
