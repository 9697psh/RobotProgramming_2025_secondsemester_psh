#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String
from vision_msgs.msg import Detection2DArray


class ObjectScannerNode(Node):
    """
    /waypoint_arrived(Int32) 받으면:
      - 지정된 시간(sample_window_sec) 동안 한 번만 스캔을 진행.
      - 스캔 중 can/box "최대 동시 감지 개수"를 계산.
      - /scan_results(String JSON) publish.
      - /scan_done(Int32 wp번호) publish -> nav2_patrol이 다음 waypoint로 진행.
    """

    def __init__(self):
        super().__init__('object_scanner_node')

        # ===== Parameters =====
        self.declare_parameter('start_delay_sec', 0.5)
        self.declare_parameter('sample_window_sec', 2.0) # 스캔 시간을 2초로 늘림
        self.declare_parameter('target_classes', ['can', 'box'])
        self.declare_parameter('output_topic', '/scan_results')

        self.start_delay = float(self.get_parameter('start_delay_sec').value)
        self.sample_window = float(self.get_parameter('sample_window_sec').value)
        self.target_classes = list(self.get_parameter('target_classes').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        # ===== Subscribers / Publishers =====
        self.wp_sub = self.create_subscription(Int32, '/waypoint_arrived', self.wp_callback, 10)
        self.det_sub = self.create_subscription(Detection2DArray, '/yolo_detections', self.det_callback, 10)
        self.result_pub = self.create_publisher(String, self.output_topic, 10)
        self.scan_done_pub = self.create_publisher(Int32, '/scan_done', 10)

        # ===== Internal state =====
        self.current_wp = None
        self.phase = 'IDLE'  # IDLE / SCANNING
        self.pending_timer = None
        self.scan_max = {c: 0 for c in self.target_classes}

        self.get_logger().info('Object Scanner node started (Single Scan Mode).')
        self.get_logger().info(
            f'Params: start_delay={self.start_delay}, sample_window={self.sample_window}'
        )

    def wp_callback(self, msg: Int32):
        wp = int(msg.data)

        # 이전 스캔이 진행 중이면 취소
        self._cancel_timer()
        self.current_wp = wp
        self.phase = 'IDLE'
        self.scan_max = {c: 0 for c in self.target_classes}

        self.get_logger().info(f'Received /waypoint_arrived: wp{wp}. Scheduling a single scan...')

        # 스캔 윈도우 시작 예약
        self.pending_timer = self.create_timer(self.start_delay, self._start_scan_window)

    def det_callback(self, msg: Detection2DArray):
        if self.phase != 'SCANNING':
            return

        # 현재 프레임에서 클래스별 동시 감지 개수 계산
        frame_counts = {c: 0 for c in self.target_classes}
        for det in msg.detections:
            if not det.results:
                continue
            cls = str(det.results[0].hypothesis.class_id)
            if cls in frame_counts:
                frame_counts[cls] += 1

        # 윈도우 동안 "max" 값 업데이트
        for c in self.target_classes:
            self.scan_max[c] = max(self.scan_max[c], frame_counts[c])

    # ===== Window control =====
    def _start_scan_window(self):
        self._cancel_timer() # 혹시 모를 타이머 정리
        self.phase = 'SCANNING'
        self.get_logger().info(f'Scan window started for {self.sample_window:.2f}s (wp{self.current_wp}).')
        self.pending_timer = self.create_timer(self.sample_window, self._end_scan_window)

    def _end_scan_window(self):
        self._cancel_timer()
        self.get_logger().info(f'Scan window ended. Scan max={self.scan_max}')
        self.phase = 'IDLE'

        if self.current_wp is None:
            return

        # 최종 결과 집계
        payload = {
            "waypoint": int(self.current_wp),
            "final": self.scan_max,
        }

        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(out)
        self.get_logger().info(f'Published {self.output_topic}: {out.data}')

        # 스캔 완료 신호 전송
        self.scan_done_pub.publish(Int32(data=int(self.current_wp)))
        self.get_logger().info(f'Published /scan_done: {int(self.current_wp)}')

    # ===== Timer helpers =====
    def _cancel_timer(self):
        if self.pending_timer:
            try:
                self.pending_timer.cancel()
            except Exception:
                pass
            self.pending_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()