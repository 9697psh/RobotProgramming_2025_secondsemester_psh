#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator: BasicNavigator, x, y, yaw_degrees):
    """Create a PoseStamped message."""
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_degrees)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = create_pose_stamped(navigator, -1.0, 10.3, 0.0)
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Define the waypoints
    waypoints = [
        create_pose_stamped(navigator, -1.0,  10.3, 0.0),   # wp1
        create_pose_stamped(navigator, 9.5,   16.6, 0.0),   # wp2
        create_pose_stamped(navigator, 19.3,  8.0,  0.0),   # wp3
        create_pose_stamped(navigator, 8.0,   0.0,  0.0),   # wp4
    ]

    # Send the waypoints and wait for the result
    navigator.goThroughPoses(waypoints)

    # Keep the script alive to monitor the result
    while rclpy.ok():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Currently navigating to waypoint {feedback.current_waypoint + 1}/{len(waypoints)}...')
        
        result = navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            print('Patrol is complete! Restarting patrol...')
            # Loop the patrol
            navigator.goThroughPoses(waypoints)
        elif result == BasicNavigator.TaskResult.CANCELED:
            print('Patrol was canceled. Shutting down.')
            break
        elif result == BasicNavigator.TaskResult.FAILED:
            print('Patrol failed! Shutting down.')
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
