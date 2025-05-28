import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd
import time

class basic_move(Node):
    def __init__(self,name):
        super().__init__(name)
        self.speed_x, self.speed_y, self.speed_z = 0.0, 0.0, 0.0
        self.motion_id = 303
        self.dog_name="mi_desktop_48_b0_2d_7a_fe_b5"
        self.pub = self.create_publisher(MotionServoCmd, f'{self.dog_name}/motion_servo_cmd',10)
        self.timer=self.create_timer(0.1, self.timer_callback)
    def change_speed(self, speed_x, speed_y, speed_z):
        self.speed_x = speed_x
        self.speed_y = speed_y
        self.speed_z = speed_z
        self.get_logger().info(f"Speed changed to: {self.speed_x}, {self.speed_y}, {self.speed_z}")
    def change_motion_id(self, motion_id):
        self.motion_id = motion_id
        self.get_logger().info(f"Motion ID changed to: {self.motion_id}")
    def timer_callback(self):
        msg = MotionServoCmd()
        msg.motion_id = self.motion_id
        msg.cmd_type=1
        msg.value = 2
        msg.vel_des=[self.speed_x, self.speed_y, self.speed_z]
        msg.step_height=[0.05, 0.05]
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")
def main(args=None):
    rclpy.init(args=args)

    basic_move_node = basic_move('basic_move')

    while rclpy.ok():
        basic_move_node.change_speed(1.0, 0.0, 0.0)
        for i in range(10):
            rclpy.spin_once(basic_move_node)
        # basic_move_node.change_speed(0.0, -1.0, 0.0)
        # for i in range(10):
        #     rclpy.spin_once(basic_move_node)
        # basic_move_node.change_speed(-5.0, 0.0, 0.0)
        # for i in range(10):
        #     rclpy.spin_once(basic_move_node)
        # basic_move_node.change_speed(0.0, 1.0, 0.0)
        # for i in range(10):
        #     rclpy.spin_once(basic_move_node)

    basic_move_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()