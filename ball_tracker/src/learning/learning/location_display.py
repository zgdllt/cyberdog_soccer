import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import TwistStamped
import math
import move
from geometry_msgs.msg import Point
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
def navigation(selfpos,targetpos,selfangle):
    vector = Point()
    vector.x = targetpos.x - selfpos.x
    vector.y = targetpos.y - selfpos.y
    myvector1 = Point()
    myvector1.x = math.cos(selfangle)
    myvector1.y = math.sin(selfangle)
    myvector2 = Point()
    myvector2.x = math.cos(selfangle+math.pi/2)
    myvector2.y = math.sin(selfangle+math.pi/2)
    new_vector = Point()
    new_vector.x = vector.x * myvector1.x + vector.y * myvector1.y
    new_vector.y = vector.x * myvector2.x + vector.y * myvector2.y
    return new_vector
def main(args=None):
    rclpy.init(args=args)
    move_node = move.basic_move('basic_move')
    # Spin ROS node in a separate thread
    # 等待直到收到有效的位置消息
    opponent_location_subscriber1 = LocationSubscriber('other_location_subscriber','go91')
    opponent_location_subscriber2 = LocationSubscriber('other_location_subscriber2','go92')
    self_location_subscriber = LocationSubscriber('self_location_subscriber','betago1')
    teammate_location_subscriber = LocationSubscriber('teammate_location_subscriber','betago2')
    location_subscriber = LocationSubscriber('location_subscriber','greenball')
    def refresh():
    # Refresh the node to get the latest data
        rclpy.spin_once(self_location_subscriber)
        rclpy.spin_once(location_subscriber)
        rclpy.spin_once(opponent_location_subscriber1)
        pose = location_subscriber.get_pose()
        mypose = self_location_subscriber.get_pose()
        otpose = opponent_location_subscriber1.get_pose()
        return pose, mypose, otpose
    ballpose = None
    mypose = None
    goal1pose = None
    goal2pose = None
    otpose = None
    print("等待接收位置消息...")
    while ballpose is None or mypose is None or otpose is None:
        ballpose, mypose, otpose = refresh()
    # Get the latest pose
    ballpos=ballpose.pose.position
    mypos = mypose.pose.position
    otpos = otpose.pose.position
    print("Received pose message")
    # Calculate the midpoint between goal1 and goal2
    goalpos = Point()
    goalpos.x = 0.0
    goalpos.y = -4.2
    goalpos.z = 0.0
    origin= Point()
    origin.x = 0.0
    origin.y = 0.0
    origin.z = 0.0
    while True:
        rclpy.spin_once(move_node)
        rclpy.spin_once(self_location_subscriber)
        rclpy.spin_once(location_subscriber)
        rclpy.spin_once(opponent_location_subscriber1)
        rclpy.spin_once(opponent_location_subscriber2)
        ballpose, mypose, otpose = refresh()
        if ballpose is None or mypose is None or otpose is None:
            continue
        ballpos = ballpose.pose.position
        mypos = mypose.pose.position
        otpos = otpose.pose.position
        # Calculate the navigation vector
        selfangle = mypose.pose.orientation.z
        print(f"ballpos:({ballpos.x}, {ballpos.y})")
        print(f"goalpos:({goalpos.x}, {goalpos.y})")
        print(f"mypos:({mypos.x}, {mypos.y})")
        print(f"otpos:({otpos.x}, {otpos.y})")
    self_location_subscriber.destroy_node()
    opponent_location_subscriber1.destroy_node()
    location_subscriber.destroy_node()
    move_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()