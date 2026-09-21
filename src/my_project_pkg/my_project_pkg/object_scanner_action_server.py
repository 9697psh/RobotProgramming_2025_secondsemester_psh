#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from vision_msgs.msg import Detection2DArray
from my_interfaces_pkg.action import ScanScene


class ObjectScannerActionServer(Node):
    """
    Action name: /scan_scene
    Goal: direction("LEFT"/"RIGHT"), waypoint(int)
    Result: can_count, box_count (윈도우 동안 max 동시 감지)
    """

    def __init__(self):
        super().__init__('object_scanner_action_server')

        self.declare_parameter('window_sec', 0.7)
        self.window_sec = float(self.get_parameter('window_sec').value)

        self.targets = ['can', 'box']
        self.latest_msg = None

        self.sub = self.create_subscription(
            Detection2DArray, '/yolo_detections', self._det_cb, 10
        )

        self.server = ActionServer(
            self, ScanScene, 'scan_scene', self._execute_cb
        )

        self.get_logger().info('✅ /scan_scene Action Server ready')

    def _det_cb(self, msg: Detection2DArray):
        self.latest_msg = msg

    def _count_frame(self):
        counts = {c: 0 for c in self.targets}
        if self.latest_msg is None:
            return counts
        for det in self.latest_msg.detections:
            if not det.results:
                continue
            cls = str(det.results[0].hypothesis.class_id)
            if cls in counts:
                counts[cls] += 1
        return counts

    def _execute_cb(self, goal_handle):
        direction = goal_handle.request.direction
        wp = int(goal_handle.request.waypoint)

        self.get_logger().info(f'📥 Scan request: dir={direction}, wp={wp}')

        start = time.time()
        max_counts = {c: 0 for c in self.targets}

        while time.time() - start < self.window_sec:
            frame = self._count_frame()
            for c in self.targets:
                max_counts[c] = max(max_counts[c], frame[c])

            fb = ScanScene.Feedback()
            fb.status = f'scanning {direction}...'
            goal_handle.publish_feedback(fb)

            time.sleep(0.05)

        goal_handle.succeed()
        result = ScanScene.Result()
        result.can_count = int(max_counts['can'])
        result.box_count = int(max_counts['box'])
        result.success = True
        result.message = f'done {direction} wp{wp}'
        self.get_logger().info(f'✅ Scan done: {max_counts}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ObjectScannerActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

