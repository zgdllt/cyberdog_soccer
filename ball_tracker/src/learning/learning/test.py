import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn/Rigid0/pose',
            self.pose_callback,
            10)
        self.subscription  

    def pose_callback(self, msg):
        self.get_logger().info('Header:')
        self.get_logger().info('  stamp: {} sec {} nanosec'.format(msg.header.stamp.sec, msg.header.stamp.nanosec))
        self.get_logger().info('  frame_id: {}'.format(msg.header.frame_id))
        self.get_logger().info('Pose:')
        self.get_logger().info('  position:')
        self.get_logger().info('    x: {}'.format(msg.pose.position.x))
        self.get_logger().info('    y: {}'.format(msg.pose.position.y))
        self.get_logger().info('    z: {}'.format(msg.pose.position.z))
        self.get_logger().info('  orientation:')
        self.get_logger().info('    x: {}'.format(msg.pose.orientation.x))
        self.get_logger().info('    y: {}'.format(msg.pose.orientation.y))
        self.get_logger().info('    z: {}'.format(msg.pose.orientation.z))
        self.get_logger().info('    w: {}'.format(msg.pose.orientation.w))


def main(args=None):
    rclpy.init(args=args)
    pose_listener = PoseListener()
    try:
        rclpy.spin(pose_listener)
    except KeyboardInterrupt:
        pose_listener.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        pose_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()