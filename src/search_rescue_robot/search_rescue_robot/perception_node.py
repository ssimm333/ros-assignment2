"""Perception node: detects colored objects via camera using HSV filtering."""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# HSV ranges for each target object color
COLOR_RANGES = {
    "green":  {"lower": np.array([35, 80, 80]),  "upper": np.array([85, 255, 255]),  "meaning": "survivor"},
    "blue":   {"lower": np.array([100, 80, 80]), "upper": np.array([130, 255, 255]), "meaning": "dam"},
    "red_lo": {"lower": np.array([0, 80, 80]),   "upper": np.array([10, 255, 255]),  "meaning": "fire"},
    "red_hi": {"lower": np.array([170, 80, 80]), "upper": np.array([180, 255, 255]), "meaning": "fire"},
    "yellow": {"lower": np.array([20, 80, 80]),  "upper": np.array([35, 255, 255]),  "meaning": "medical_kit"},
    "purple": {"lower": np.array([125, 40, 40]), "upper": np.array([160, 255, 255]), "meaning": "exit"},
}

MIN_AREA = 500  # Minimum contour area to count as a detection


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.image_cb, 5)
        self.pub = self.create_publisher(String, "/detected_objects", 10)
        self.get_logger().info("Perception node started")

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detections = []
        for color_name, cfg in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, cfg["lower"], cfg["upper"])

            # Merge both red ranges
            if color_name == "red_hi":
                continue
            if color_name == "red_lo":
                mask_hi = cv2.inRange(hsv, COLOR_RANGES["red_hi"]["lower"], COLOR_RANGES["red_hi"]["upper"])
                mask = cv2.bitwise_or(mask, mask_hi)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2
                meaning = cfg["meaning"]
                detections.append({
                    "color": color_name.replace("_lo", ""),
                    "meaning": meaning,
                    "area": int(area),
                    "center_x": cx,
                    "center_y": cy,
                })

        if detections:
            out = String()
            out.data = json.dumps(detections)
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
