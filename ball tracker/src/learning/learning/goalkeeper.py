import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import TwistStamped
import math
import move
class LocationSubscriber(Node):
    def __init__(self, name,Rigid):
        super().__init__(name)
        print(f"Subscribing to /vrpn/{Rigid}/pose")
        self.sub_pose = self.create_subscription(PoseStamped, f'/vrpn/{Rigid}/pose', self.listener_callback_pose, 10)
        self.sub_twist = self.create_subscription(TwistStamped, f'/vrpn/{Rigid}/twist', self.listener_callback_twist, 10)
        self.sub_accel = self.create_subscription(AccelStamped, f'/vrpn/{Rigid}/accel', self.listener_callback_accel, 10)
        self.pose = None
        self.twist = None
        self.accel = None
    def listener_callback_pose(self, msg: PoseStamped):
        # Store the latest pose
        self.pose = msg
    def listener_callback_twist(self, msg: TwistStamped):
        # Store the latest twist
        self.twist = msg
    def listener_callback_accel(self, msg: AccelStamped):
        # Store the latest acceleration
        self.accel = msg
    def get_pose(self):
        # Return the latest pose
        return self.pose
    def get_twist(self):
        # Return the latest twist
        return self.twist
    def get_accel(self):
        # Return the latest acceleration
        return self.accel
def main(args=None):
    rclpy.init(args=args)
    self_location_subscriber = LocationSubscriber('self_location_subscriber','betago1')
    location_subscriber = LocationSubscriber('location_subscriber','greenball')
    move_node = move.basic_move('basic_move')
    # Spin ROS node in a separate thread
    pose = None
    mypose = None
    while pose is None or mypose is None:
        rclpy.spin_once(location_subscriber)
        rclpy.spin_once(self_location_subscriber)
        pose = location_subscriber.get_pose()
        mypose = self_location_subscriber.get_pose()
    # Get the latest pose
    pose = location_subscriber.get_pose()
    pos=pose.pose.position
    mypose = self_location_subscriber.get_pose()
    mypos = mypose.pose.position
    twist = location_subscriber.get_twist()
    accel = location_subscriber.get_accel()
    if pose is not None:
        print("Pose: ", pose)
    else:
        print("No pose received yet.")
    if twist is not None:
        print("Twist: ", twist)
    else:
        print("No twist received yet.")
    if accel is not None:
        print("Acceleration: ", accel)
    else:
        print("No acceleration received yet.")
        
    gate_x=0
    gate_y=-3.5
    is_on_position=0
    while 1:
        rclpy.spin_once(location_subscriber)
        rclpy.spin_once(self_location_subscriber)
        pose = location_subscriber.get_pose()
        pos=pose.pose.position
        mypose = self_location_subscriber.get_pose()
        mypos = mypose.pose.position
        myangle = mypose.pose.orientation
        myangle =math.atan2(2*(myangle.z*myangle.w+myangle.x*myangle.y),1-2*(myangle.y*myangle.y+myangle.z*myangle.z))
        angle = math.atan2(pos.y-mypos.y,pos.x-mypos.x)+ math.pi
        twist = location_subscriber.get_twist()
        accel = location_subscriber.get_accel()
        if pose is not None:
            print("Pose: ", pose)
        else:
            print("No pose received yet.")
        if twist is not None:
            print("Twist: ", twist)
        else:
            print("No twist received yet.")
        if accel is not None:
            print("Acceleration: ", accel)
        else:
            print("No acceleration received yet.")
        if (abs(gate_y-mypos.y)< 0.5):
            is_on_position=1
        if (is_on_position==0):
            angle = math.atan2(gate_y-mypos.y,gate_x-mypos.x)+ math.pi
            angle_diff = angle - myangle
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            move_node.change_speed(1.0,0.0,angle_diff)
        else:
            if (math.hypot(pos.x-mypos.x,pos.y-mypos.y) <0.5):
                move_node.change_speed(2*(mypos.x-pos.x), mypos.y-pos.y, -myangle)
            else:
                move_node.change_speed(2*(mypos.x-pos.x),mypos.y-gate_y,-myangle)
        
        # move_node.change_speed(0.0, 0.0, 0.0)
        # move_node.change_speed(-1.0, 0.0, 0.0)
        # for i in range(2):
        rclpy.spin_once(move_node)

    location_subscriber.destroy_node()
    self_location_subscriber.destroy_node()
    move_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()