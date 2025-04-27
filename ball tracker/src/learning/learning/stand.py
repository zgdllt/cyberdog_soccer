import rclpy
from rclpy.node import Node
from protocol.srv import MotionResultCmd

class basic_cmd(Node):
    def __init__(self,name):
        super().__init__(name)
        self.cli = self.create_client(MotionResultCmd, 'mi_desktop_48_b0_2d_7a_fe_b5/motion_result_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MotionResultCmd.Request()

    def send_request(self):
        self.req.motion_id = 111
        self.future = self.cli.call_async(self.req)
def main(args=None):
    rclpy.init(args=args)

    basic_cmd_node = basic_cmd('basic_cmd')
    basic_cmd_node.send_request()

    while rclpy.ok():
        rclpy.spin_once(basic_cmd_node)
        if basic_cmd_node.future.done():
            try:
                response = basic_cmd_node.future.result()
            except Exception as e:
                basic_cmd_node.get_logger().info('Service call failed %r' % (e,))
            else:
                basic_cmd_node.get_logger().info('Result: %s' % (response,))
            break

    basic_cmd_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()