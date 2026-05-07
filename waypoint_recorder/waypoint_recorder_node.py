import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml
import os
from datetime import datetime


class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # Parameters
        self.declare_parameter('save_path', os.path.expanduser('~/waypoints'))
        self.declare_parameter('filename', '')  # empty = auto timestamp
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('auto_save', True)

        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.filename = self.get_parameter('filename').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.auto_save = self.get_parameter('auto_save').get_parameter_value().bool_value

        os.makedirs(self.save_path, exist_ok=True)

        self.waypoints = []

        # Subscribe to goal_pose
        self.sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Subscribe to commands: 'save', 'clear', 'undo'
        self.cmd_sub = self.create_subscription(
            String,
            '/waypoint_recorder/cmd',
            self.cmd_callback,
            10
        )

        self.get_logger().info('=== Waypoint Recorder started ===')
        self.get_logger().info(f'Subscribing to /goal_pose')
        self.get_logger().info(f'Save path: {self.save_path}')
        self.get_logger().info('Commands via /waypoint_recorder/cmd: save | clear | undo')

    def goal_pose_callback(self, msg: PoseStamped):
        wp = {
            'position': {
                'x': round(msg.pose.position.x, 4),
                'y': round(msg.pose.position.y, 4),
                'z': round(msg.pose.position.z, 4),
            },
            'orientation': {
                'x': round(msg.pose.orientation.x, 6),
                'y': round(msg.pose.orientation.y, 6),
                'z': round(msg.pose.orientation.z, 6),
                'w': round(msg.pose.orientation.w, 6),
            }
        }
        self.waypoints.append(wp)
        idx = len(self.waypoints)
        self.get_logger().info(
            f'[{idx}] Waypoint recorded: x={wp["position"]["x"]}, y={wp["position"]["y"]}, '
            f'yaw_z={wp["orientation"]["z"]:.4f}, w={wp["orientation"]["w"]:.4f}'
        )

        if self.auto_save:
            self._save()

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'save':
            self._save()
        elif cmd == 'clear':
            self.waypoints.clear()
            self.get_logger().info('All waypoints cleared.')
        elif cmd == 'undo':
            if self.waypoints:
                removed = self.waypoints.pop()
                self.get_logger().info(
                    f'Removed last waypoint: x={removed["position"]["x"]}, y={removed["position"]["y"]}'
                )
            else:
                self.get_logger().warn('No waypoints to undo.')
        else:
            self.get_logger().warn(f'Unknown command: {cmd}. Use save | clear | undo')

    def _save(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints to save.')
            return

        if self.filename:
            fname = self.filename if self.filename.endswith('.yaml') else self.filename + '.yaml'
        else:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = f'waypoints_{stamp}.yaml'

        filepath = os.path.join(self.save_path, fname)

        data = {
            'map_frame': self.map_frame,
            'waypoints': self.waypoints
        }

        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints → {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down waypoint recorder...')
        if node.waypoints and not node.auto_save:
            node._save()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()