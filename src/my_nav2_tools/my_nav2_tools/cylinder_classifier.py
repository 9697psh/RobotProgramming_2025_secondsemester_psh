#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class CylinderClassifier(Node):
    def __init__(self):
        super().__init__('cylinder_classifier')

        # 카메라 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',   # 필요하면 launch에서 remap 가능
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # 파라미터: 영역/중심 기준 설정
        # (너가 나중에 튜닝해도 됨)
        self.area_threshold_ratio = 0.01   # 이미지 전체 픽셀 수의 1% 이상이면 "가까이" 온 걸로
        self.center_tolerance_ratio = 0.25 # 중앙에서 화면 너비/높이의 25% 안쪽이면 OK

        self.get_logger().info('CylinderClassifier node started.')

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV 이미지 (BGR)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w, _ = cv_image.shape
        img_area = h * w

        # BGR -> HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ===== 1) 노란색 마스크 =====
        # 대략적인 노란색 범위 (필요하면 튜닝)
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # ===== 2) 검은색 마스크 =====
        # 검은색은 밝기(V)가 낮은 영역으로 잡음
        black_lower = np.array([0, 0, 0])
        black_upper = np.array([180, 255, 50])
        black_mask = cv2.inRange(hsv, black_lower, black_upper)

        # 노이즈 조금 제거 (모폴로지)
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)

        # 색별로 가장 큰 컨투어 찾기
        yellow_found, yellow_area, yellow_center = self.find_largest_blob(yellow_mask)
        black_found, black_area, black_center = self.find_largest_blob(black_mask)

        # "범위 안" 판단 기준
        area_threshold = self.area_threshold_ratio * img_area

        def in_center(center):
            if center is None:
                return False
            cx, cy = center
            dx = abs(cx - w / 2.0)
            dy = abs(cy - h / 2.0)
            return (dx < self.center_tolerance_ratio * w) and \
                   (dy < self.center_tolerance_ratio * h)

        good = False
        bad = False

        if yellow_found and yellow_area > area_threshold and in_center(yellow_center):
            good = True

        if black_found and black_area > area_threshold and in_center(black_center):
            bad = True

        # 우선순위: 검은색(Bad) > 노란색(Good) 으로 가정
        if bad:
            self.get_logger().info('Bad')   # 여기에 나중에 다른 동작도 붙일 수 있음
        elif good:
            self.get_logger().info('Good')
        # else:
        #     아무것도 안 찍고 넘어감

        # 디버깅용: 원하면 이미지 창 띄우기 (SSH면 끄는 게 좋음)
        # self.debug_show(cv_image, yellow_mask, black_mask, yellow_center, black_center)

    def find_largest_blob(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return False, 0, None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        M = cv2.moments(largest)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            center = (cx, cy)
        else:
            center = None

        return True, area, center

    def debug_show(self, cv_image, yellow_mask, black_mask, yellow_center, black_center):
        # 디버깅용 시각화 (필요하면 켜서 사용)
        vis = cv_image.copy()

        if yellow_center is not None:
            cv2.circle(vis, yellow_center, 10, (0, 255, 255), 2)
        if black_center is not None:
            cv2.circle(vis, black_center, 10, (0, 0, 0), 2)

        cv2.imshow('camera', vis)
        cv2.imshow('yellow_mask', yellow_mask)
        cv2.imshow('black_mask', black_mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CylinderClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
