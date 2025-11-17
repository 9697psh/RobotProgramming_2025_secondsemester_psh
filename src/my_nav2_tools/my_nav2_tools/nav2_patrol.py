#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw_rad: float):
    """
    z축(yaw)만 있는 단순 2D 회전 → quaternion.
    """
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return (0.0, 0.0, qz, qw)


class Nav2SequentialPatrol(Node):
    def __init__(self):
        super().__init__('nav2_sequential_patrol')

        # Nav2 NavigateToPose 액션 클라이언트
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # 여기서 waypoints 정의 (map 기준)
        # TODO: 아래 숫자들을 RViz에서 읽은 값으로 바꿔줘
        self.start_pose = self._create_pose(0.0, 0.0, math.radians(0.0))
        self.waypoints = [
    		self._create_pose(-1.86,  -8.637, math.radians(0.0)),   # wp1
    		self._create_pose(-4.0,    9.9,   math.radians(0.0)),   # wp2
    		self._create_pose(-15.2,   0.37,  math.radians(0.0)),   # wp3
    		self._create_pose(-8.0,   -7.6,   math.radians(0.0)),   # wp4
        ]

    def _create_pose(self, x, y, yaw_rad):
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Nav2는 기본적으로 map 프레임 기준
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def run_patrol(self):
        # 액션 서버가 뜰 때까지 대기
        self.get_logger().info('Waiting for "navigate_to_pose" action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available.')

        # 4개 waypoints + 다시 start_pose 로 순차 주행
        sequence = self.waypoints + [self.start_pose]

        for idx, pose in enumerate(sequence):
            self.get_logger().info(f'[{idx+1}/{len(sequence)}] Sending goal: '
                                   f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

            if not self._send_goal_and_wait(pose):
                self.get_logger().warn(f'Goal {idx+1} failed or canceled. Stopping patrol.')
                break

        self.get_logger().info('Patrol sequence finished.')

    def _send_goal_and_wait(self, pose: PoseStamped) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # header.stamp 는 Nav2가 알아서 업데이트해도 되고,
        # 여기서 지금 시간으로 채워도 됨.
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded.')
            return True
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = Nav2SequentialPatrol()

    try:
        node.run_patrol()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
