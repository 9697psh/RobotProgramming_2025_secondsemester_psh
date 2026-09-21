import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import os
import glob
import json
from collections import defaultdict
import math

def yaw_to_quaternion(yaw_rad: float):
    """Helper function to convert yaw angle to quaternion."""
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return (0.0, 0.0, qz, qw)

class PriorityPatrolNode(Node):
    """
    Analyzes trash logs to patrol waypoints with the most trash first.
    """

    def __init__(self):
        super().__init__('priority_patrol_node')
        self.get_logger().info('🚀 Priority Patrol Node starting...')

        # --- Parameters ---
        self.report_directory = os.path.expanduser('~/ros2_project_ws/report')
        self.bt_xml_path = (
            '/home/suhyeong/ros2_project_ws/install/clean_nav_pkg/share/'
            'clean_nav_pkg/behavior_trees/scan_bt.xml'
        )

        # --- Waypoint Definitions ---
        self.waypoint_coords = {
            1: self._create_pose(4.927, 3.38, -0.65),
            2: self._create_pose(10.25, 6.66, 0.53),
            3: self._create_pose(5.299, 8.14, 1.459),
            4: self._create_pose(3.48, 5.965, 2.869),
        }
        self.get_logger().info(f'{len(self.waypoint_coords)} waypoints defined.')

        # --- Nav2 Action Client ---
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.patrol_sequence = []
        self.patrol_index = 0

    def _create_pose(self, x, y, yaw_rad):
        """Creates a PoseStamped message from coordinates."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        _, _, qz, qw = yaw_to_quaternion(yaw_rad)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def analyze_reports_and_get_route(self):
        """
        Analyzes reports and returns a tuple of:
        (list of PoseStamped for the route, list of sorted waypoint IDs)
        """
        self.get_logger().info(f'Analyzing reports in: {self.report_directory}')
        json_files = glob.glob(os.path.join(self.report_directory, '*.json'))

        if not json_files:
            self.get_logger().warn('No report files found. Cannot create priority route.')
            return [], []

        trash_counts = defaultdict(int)
        for file_path in json_files:
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
                    # The actual data is inside the "results" key
                    scan_results = data.get('results', [])
                    for entry in scan_results:
                        waypoint = entry.get('waypoint')
                        # Handle both old and new formats
                        if 'final' in entry:
                            final_scan = entry.get('final', {})
                        else: # Legacy format with left/right
                            final_scan = {'can': 0, 'box': 0}
                            left = entry.get('left', {})
                            right = entry.get('right', {})
                            final_scan['can'] = left.get('can', 0) + right.get('can', 0)
                            final_scan['box'] = left.get('box', 0) + right.get('box', 0)

                        can_count = final_scan.get('can', 0)
                        box_count = final_scan.get('box', 0)
                        if waypoint:
                            trash_counts[waypoint] += can_count + box_count
            except Exception as e:
                self.get_logger().error(f'Failed to read or parse {file_path}: {e}')

        if not trash_counts:
            self.get_logger().warn('No waypoint data found in reports. Cannot create priority route.')
            return [], []

        sorted_waypoints = sorted(trash_counts.items(), key=lambda item: item[1], reverse=True)
        self.get_logger().info(f'Trash counts per waypoint: {dict(trash_counts)}')

        priority_route = []
        sorted_wp_ids = []
        for wp_id, count in sorted_waypoints:
            if wp_id in self.waypoint_coords:
                priority_route.append(self.waypoint_coords[wp_id])
                sorted_wp_ids.append(wp_id)
            else:
                self.get_logger().warn(f'Waypoint ID {wp_id} from logs not defined in waypoint_coords. Skipping.')
        
        return priority_route, sorted_wp_ids

    def start_patrol(self):
        """Main entry point to start the patrol process."""
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available.')

        self.patrol_sequence, sorted_ids = self.analyze_reports_and_get_route()

        if not self.patrol_sequence:
            self.get_logger().error('Patrol route is empty. Shutting down.')
            return

        # --- User requested log messages ---
        top_priority_wp = sorted_ids[0]
        patrol_order_str = ' -> '.join(map(str, sorted_ids))
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f"무단투기 리포트를 확인한 결과 '{top_priority_wp}' 지점이 가장 많은 무단투기가 발생한 지점이므로 우선적으로 순찰합니다.")
        self.get_logger().info(f"전체 순찰 순서: {patrol_order_str}")
        self.get_logger().info('=' * 50)
        # --- End of user messages ---

        self.get_logger().info('Starting priority patrol...')
        self._send_next_goal()

    def _send_next_goal(self):
        """Sends the next goal in the patrol sequence."""
        if self.patrol_index >= len(self.patrol_sequence):
            self.get_logger().info('Priority patrol finished.')
            rclpy.shutdown()
            return

        pose = self.patrol_sequence[self.patrol_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.behavior_tree = self.bt_xml_path

        self.get_logger().info(
            f'Sending goal {self.patrol_index + 1}/{len(self.patrol_sequence)}: '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_accepted_callback)

    def _goal_accepted_callback(self, future):
        """Callback for when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected. Moving to next waypoint.')
            self.patrol_index += 1
            self._send_next_goal()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Callback for when the goal is completed."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal {self.patrol_index + 1} succeeded.')
        else:
            self.get_logger().warn(
                f'Goal {self.patrol_index + 1} failed with status: {status}. '
                'Moving to next waypoint.'
            )

        self.patrol_index += 1
        self._send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = PriorityPatrolNode()
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