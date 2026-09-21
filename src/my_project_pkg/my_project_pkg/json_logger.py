#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class JsonLoggerNode(Node):
    def __init__(self):
        super().__init__('json_logger_node')

        self.declare_parameter('output_filename', 'default_day.json')
        self.output_filename = str(self.get_parameter('output_filename').value)

        # 저장 폴더(원하면 변경 가능)
        self.out_dir = Path.home() / "ros2_project_ws" / "report"
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.out_path = self.out_dir / self.output_filename

        self.sub = self.create_subscription(String, '/scan_results', self.cb, 10)

        self.get_logger().info(f'JsonLogger started. Output: {self.out_path}')

    def cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse /scan_results JSON: {e}')
            return

        # 파일 로드(없으면 새로)
        if self.out_path.exists():
            try:
                data = json.loads(self.out_path.read_text(encoding='utf-8'))
            except Exception:
                data = {}
        else:
            data = {}

        # 구조: day/results 리스트 형태로 저장
        if "results" not in data or not isinstance(data["results"], list):
            data["results"] = []

        data["results"].append(payload)

        self.out_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        self.get_logger().info(f'Logged scan result to {self.out_path}')

def main(args=None):
    rclpy.init(args=args)
    node = JsonLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
