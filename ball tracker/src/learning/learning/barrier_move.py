import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd
from sensor_msgs.msg import Range

class DogAvoidanceController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("dog_name", "mi_desktop_48_b0_2d_7a_fe_b5")
        self.declare_parameter("safe_distance", 0.5)
        self.declare_parameter("forward_speed", 0.3)
        self.declare_parameter("turn_speed", 0.5)

        self.dog_name = self.get_parameter('dog_name').get_parameter_value().string_value
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value

        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0

        self.distance = float('inf')

        self.pub = self.create_publisher(MotionServoCmd, f"/{self.dog_name}/motion_servo_cmd", 10)

        self.sub = self.create_subscription(
        Range, f"/{self.dog_name}/ultrasonic_payload", self.sensor_callback, 10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.execute_forward()

    def execute_forward(self):
        self.speed_x = self.forward_speed
        self.speed_y = 0.0
        self.speed_z = 0.0

    def execute_turn(self):
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = self.turn_speed

    def stop(self):
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0

    def sensor_callback(self, msg: Range):
        self.distance = msg.range
        self.get_logger().info(f"Distance to obstacle: {self.distance:.2f} meters")

    def timer_callback(self):
        msg = MotionServoCmd()
        msg.motion_id = 303
        msg.cmd_type = 1
        msg.value = 2
        msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]
        msg.step_height = [0.05, 0.05]

        if self.distance < self.safe_distance:
            self.get_logger().info("Obstacle detected! Turning...")
            self.execute_turn()
        else:
            self.get_logger().info("Path clear, moving forward...")
            self.execute_forward()

        msg.vel_des = [self.speed_x, self.speed_y, self.speed_z]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DogAvoidanceController("dog_avoidance_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()