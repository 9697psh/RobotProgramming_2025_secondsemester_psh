#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw_rad: float):
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return (0.0, 0.0, qz, qw)


class Nav2PatrolOnly(Node):
    """
    ✅ 목표 동작:
      - wp 도착 성공 -> /waypoint_arrived(wp번호) 발행
      - object_scanner가 /scan_done(wp번호) 발행할 때까지 대기
      - scan_done 받으면 다음 wp로 이동
    ✅ 실패(ABORTED 등) 시:
      - 동일 wp를 max_retries 만큼 재시도
      - 재시도 초과하면 해당 wp는 스킵하고 다음 wp로 진행
    """

    def __init__(self):
        super().__init__('nav2_patrol_node')

        # --- 파라미터(필요하면 launch/yaml로 조절 가능) ---
        self.declare_parameter('scan_timeout_sec', 20.0)   # 스캔 완료 신호가 너무 안 오면 강제로 다음 진행
        self.declare_parameter('max_retries', 2)           # wp 실패 시 재시도 횟수
        self.declare_parameter('retry_delay_sec', 1.0)     # 재시도 딜레이

        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)

        # ✅ (기존 방식 유지) scan_bt.xml 경로
        self.bt_xml_path = (
            '/home/suhyeong/ros2_project_ws/install/clean_nav_pkg/share/'
            'clean_nav_pkg/behavior_trees/scan_bt.xml'
        )
        self.get_logger().info(f'Using custom BT: {self.bt_xml_path}')

        # Nav2 NavigateToPose 액션 클라이언트
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 토픽
        self.waypoint_arrived_pub = self.create_publisher(Int32, '/waypoint_arrived', 10)
        self.scan_done_sub = self.create_subscription(Int32, '/scan_done', self._on_scan_done, 10)

        # (유지) 사람 수 구독 - 지금은 순찰만 계속
        self.person_subscriber = self.create_subscription(
            Int32,
            '/person_count',
            self.person_count_callback,
            10
        )

        # Waypoints
        self.waypoints = [
            self._create_pose(4.927, 3.38, -0.65),   # wp1
            self._create_pose(10.25, 6.66, 0.53),   # wp2
            self._create_pose(5.299, 8.14, 1.459),   # wp3
            self._create_pose(3.48, 5.965, 2.869),   # wp4
        ]

        # wp1 -> wp2 -> wp3 -> wp4 -> wp1(복귀)
        self.patrol_sequence = self.waypoints + [self.waypoints[0]]
        self.patrol_index = 0
        self.current_goal_handle = None

        # 스캔 완료 대기 상태
        self.awaiting_scan_wp = None
        self._scan_timeout_timer = None

        # 재시도 카운터 (patrol_index 기준)
        self.retry_count = {}

    def person_count_callback(self, msg: Int32):
        if msg.data >= 2:
            self.get_logger().info('2명 이상의 사람 감지! (현재는 순찰만 계속합니다)')

    def _create_pose(self, x, y, yaw_rad):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def start_patrol(self):
        self.get_logger().info('Waiting for "navigate_to_pose" action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available.')
        self._send_patrol_goal()  # ✅ 바로 시작

    def _send_patrol_goal(self):
        if self.awaiting_scan_wp is not None:
            # 스캔 완료 기다리는 동안에는 다음 goal 절대 보내지 않음
            return

        if self.patrol_index >= len(self.patrol_sequence) or not rclpy.ok():
            self.get_logger().info('Patrol sequence finished.')
            rclpy.shutdown()
            return

        pose = self.patrol_sequence[self.patrol_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.behavior_tree = self.bt_xml_path  # ✅ BT 적용

        self.get_logger().info(
            f'[{self.patrol_index + 1}/{len(self.patrol_sequence)}] Sending goal: '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_accepted_callback)

    def _goal_accepted_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().warn('Goal rejected. Skipping to next.')
            self.patrol_index += 1
            self._send_patrol_goal()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _start_scan_timeout(self):
        if self._scan_timeout_timer is not None:
            try:
                self._scan_timeout_timer.cancel()
            except Exception:
                pass

        self._scan_timeout_timer = self.create_timer(self.scan_timeout_sec, self._on_scan_timeout)

    def _on_scan_timeout(self):
        # one-shot
        try:
            self._scan_timeout_timer.cancel()
        except Exception:
            pass

        wp = self.awaiting_scan_wp
        self.get_logger().warn(f'Scan timeout waiting /scan_done for wp{wp}. Continue patrol anyway.')
        self.awaiting_scan_wp = None
        self.patrol_index += 1
        self._send_patrol_goal()

    def _on_scan_done(self, msg: Int32):
        if self.awaiting_scan_wp is None:
            return
        if int(msg.data) != int(self.awaiting_scan_wp):
            return

        if self._scan_timeout_timer is not None:
            try:
                self._scan_timeout_timer.cancel()
            except Exception:
                pass
            self._scan_timeout_timer = None

        self.get_logger().info(f'Received /scan_done for wp{msg.data}. Continue patrol.')
        self.awaiting_scan_wp = None
        self.patrol_index += 1
        self._send_patrol_goal()

    def _goal_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            # ✅ wp1~wp4에서만 스캔 게이트 (마지막 복귀 goal은 제외)
            if self.patrol_index < len(self.waypoints):
                wp_num = self.patrol_index + 1
                self.waypoint_arrived_pub.publish(Int32(data=wp_num))
                self.get_logger().info(f'Published /waypoint_arrived: {wp_num}')

                # ✅ 여기서 멈추고 스캔 완료 기다림
                self.awaiting_scan_wp = wp_num
                self._start_scan_timeout()
                return

            # 복귀 goal(마지막)은 그냥 다음으로 진행
            self.get_logger().info(f'Goal {self.patrol_index + 1} succeeded.')
            self.patrol_index += 1
            self._send_patrol_goal()
            return

        # --- 실패 처리(최적 운영: 재시도 -> 스킵) ---
        idx = self.patrol_index
        cnt = self.retry_count.get(idx, 0)

        if cnt < self.max_retries:
            self.retry_count[idx] = cnt + 1
            self.get_logger().warn(
                f'Goal {idx + 1} failed(status={status}). Retry {self.retry_count[idx]}/{self.max_retries}...'
            )
            # 약간 기다렸다가 같은 goal 재전송
            self.create_timer(self.retry_delay_sec, self._retry_once)
            return

        self.get_logger().warn(
            f'Goal {idx + 1} failed(status={status}) after {self.max_retries} retries. Skipping this waypoint.'
        )
        self.retry_count[idx] = 0
        self.patrol_index += 1
        self._send_patrol_goal()

    def _retry_once(self):
        # one-shot timer callback: 다시 현재 patrol_index goal 전송
        # (awaiting_scan_wp가 있으면 재전송 안 함)
        if self.awaiting_scan_wp is None:
            self._send_patrol_goal()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PatrolOnly()
    try:
        node.start_patrol()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

