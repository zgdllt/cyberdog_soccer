import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
import math
import move


class LocationSubscriber(Node):
    def __init__(self, name,Rigid):
        super().__init__(name)
        print(f"Subscribing to /vrpn/{Rigid}/pose")
        self.declare_parameter("dog_name", "custom_namespace")
        self.dog_name = self.get_parameter('dog_name').get_parameter_value().string_value
        self.sub_pose = self.create_subscription(PoseStamped, f'/vrpn/{Rigid}/pose', self.listener_callback_pose, 10)
        self.sub_twist = self.create_subscription(TwistStamped, f'/vrpn/{Rigid}/twist', self.listener_callback_twist, 10)
        self.sub_accel = self.create_subscription(AccelStamped, f'/vrpn/{Rigid}/accel', self.listener_callback_accel, 10)
        self.pose = None
        self.twist = None
        self.accel = None

        self.distance = float('inf')

        self.sub = self.create_subscription(
        Range, f"/{self.dog_name}/ultrasonic_payload", self.sensor_callback, 10
        )
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
    def sensor_callback(self, msg: Range):
        self.distance = msg.range
        #self.get_logger().info(f"Distance to obstacle: {self.distance:.2f} meters")
def main(args=None):
    rclpy.init(args=args)
    self_location_subscriber = LocationSubscriber('self_location_subscriber','betago2')
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
    '''
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
    '''    
    left_pole_x=-1
    left_pole_y=3.5
    right_pole_x=1
    right_pole_y=3.5
    gate_x=(left_pole_x+right_pole_x)/2
    gate_y=(left_pole_y+right_pole_y)/2
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
        '''
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
        '''
        #self_location_subscriber.get_logger().info(f"Distance to obstacle: {self_location_subscriber.distance:.2f} meters")

        if (mypos.x>left_pole_x and mypos.x<right_pole_x and abs(pos.y-gate_y)>abs(mypos.y-gate_y)):
            is_on_position=1
        if(abs(pos.y-gate_y)<abs(mypos.y-gate_y) or mypos.x>2.5 or mypos.x<-2.5 ):
            is_on_position=0
        if (is_on_position==0):
            angle = math.atan2(gate_y-mypos.y,gate_x-mypos.x)+ math.pi
            angle_diff = angle - myangle
            
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            self_location_subscriber.get_logger().info(f"{angle_diff:.2f} ")
            if  self_location_subscriber.distance < 0.4:
                if (abs(angle_diff) > 30):
                    move_node.change_speed(0.0, 0.0, angle_diff)
                else:
                    if(angle_diff>0):
                        move_node.change_speed(0.0,0.0,0.5)
                    else :
                        move_node.change_speed(0.0,0.0,-0.5)
            else:
                move_node.change_speed(0.4, 0.0, angle_diff)
        else:
            if (abs(myangle) > 1):
                move_node.change_speed(0.0, 0.0, -myangle)
            elif (math.hypot(pos.x-mypos.x,pos.y-mypos.y) <0.5 and abs(mypos.y-gate_y)<2.0):
                move_node.change_speed(mypos.x-pos.x, mypos.y-pos.y, -myangle)
            else:
                target_x=0.4*pos.x+0.6*gate_x
                target_y=0.4*pos.y+0.6*gate_y
                if(abs(target_y-gate_y)>1.5):
                    target_y=gate_y
                    target_x=pos.x
                target_x=max(target_x,left_pole_x)
                target_x=min(target_x,right_pole_x)
                target_y=min(target_y,2.7)
                move_node.change_speed(mypos.x-target_x,mypos.y-target_y,-myangle)
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