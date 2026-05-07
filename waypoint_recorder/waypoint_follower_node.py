import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import yaml
import sys
import os


class WaypointFollowerNode(Node):
    def __init__(self, yaml_path: str):
        super().__init__('waypoint_follower')

        self.yaml_path = yaml_path
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.get_logger().info(f'Loading waypoints from: {yaml_path}')
        self.waypoints = self._load_yaml(yaml_path)

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Exiting.')
            return

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')
        self.get_logger().info('Waiting for follow_waypoints action server...')
        self.client.wait_for_server()
        self.get_logger().info('Action server ready. Sending goal...')
        self._send_goal()

    def _load_yaml(self, path: str):
        if not os.path.exists(path):
            self.get_logger().error(f'File not found: {path}')
            return []

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        map_frame = data.get('map_frame', 'map')
        poses = []
        for wp in data.get('waypoints', []):
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wp['position']['x'])
            pose.pose.position.y = float(wp['position']['y'])
            pose.pose.position.z = float(wp['position'].get('z', 0.0))
            pose.pose.orientation.x = float(wp['orientation'].get('x', 0.0))
            pose.pose.orientation.y = float(wp['orientation'].get('y', 0.0))
            pose.pose.orientation.z = float(wp['orientation'].get('z', 0.0))
            pose.pose.orientation.w = float(wp['orientation'].get('w', 1.0))
            poses.append(pose)

        return poses

    def _send_goal(self):
        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        total = len(self.waypoints)
        self.get_logger().info(f'Navigating to waypoint [{idx + 1}/{total}]')

    def _result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            missed = result.missed_waypoints
            if missed:
                self.get_logger().warn(f'Completed with {len(missed)} missed waypoints: {list(missed)}')
            else:
                self.get_logger().info('All waypoints reached successfully!')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: ros2 run waypoint_recorder waypoint_follower <path_to_waypoints.yaml>')
        rclpy.shutdown()
        return

    yaml_path = sys.argv[1]
    node = WaypointFollowerNode(yaml_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()