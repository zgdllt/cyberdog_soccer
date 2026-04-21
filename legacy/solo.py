import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import TwistStamped
import math
import move
import asyncio
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
async def main(args=None):
    rclpy.init(args=args)
    move_node = move.basic_move('basic_move')
    # Spin ROS node in a separate thread
    # 等待直到收到有效的位置消息
    self_location_subscriber = LocationSubscriber('self_location_subscriber','dog54')
    location_subscriber = LocationSubscriber('location_subscriber','ball')
    async def refresh():
    # Refresh the node to get the latest data
        rclpy.spin_once(self_location_subscriber)
        rclpy.spin_once(location_subscriber)
        pose = location_subscriber.get_pose()
        mypose = self_location_subscriber.get_pose()
        return pose, mypose
    ballpose = None
    mypose = None
    print("等待接收位置消息...")
    while ballpose is None or mypose is None:
        ballpose, mypose=await refresh()
    # Get the latest pose
    ballpos=ballpose.pose.position
    mypos = mypose.pose.position
    print("Received pose message")
    # Calculate the midpoint between goal1 and goal2
    goalpos = Point()
    goalpos.x = 0.0
    goalpos.y = -4.3
    goalpos.z = 0.0
    origin= Point()
    origin.x = 0.0
    origin.y = 0.0
    origin.z = 0.0
    print(f"ballpos:({ballpos.x}, {ballpos.y})")
    print(f"goalpos:({goalpos.x}, {goalpos.y})")
    while abs(ballpos.y)<abs(goalpos.y):
        rclpy.spin_once(self_location_subscriber)
        rclpy.spin_once(location_subscriber)
        ballpose, mypose=await refresh()
        move_node.change_motion_id(308)
        status=None
        if ballpos.y>mypos.y:
            status="return"
            ballpos=ballpose.pose.position
            mypos = mypose.pose.position
            myangle = mypose.pose.orientation
            myangle =math.atan2(2*(myangle.z*myangle.w+myangle.x*myangle.y),1-2*(myangle.y*myangle.y+myangle.z*myangle.z))
            angle = math.pi/2
            print("myangle: ", myangle, "angle: ", angle)
            print(f"mypos:({mypos.x}, {mypos.y})")
            print(f"ballpos:({ballpos.x}, {ballpos.y})")
            print("distance: ", math.hypot(ballpos.x-mypos.x,ballpos.y-mypos.y))
            if abs(mypos.x)>2.5 or abs(mypos.y)>4.5:
                print("out of range")
                return_vector = navigation(mypos,origin,myangle)
                move_node.change_speed(-return_vector.x,-return_vector.y, 0.0)
                rclpy.spin_once(move_node)
            targetpos = Point()
            goal_vector_x = goalpos.x - ballpos.x
            goal_vector_y = goalpos.y - ballpos.y
            goal_vector_norm = math.hypot(goal_vector_x, goal_vector_y)
            if goal_vector_norm > 1e-6:
                targetpos.x = ballpos.x - 0.25 * goal_vector_x / goal_vector_norm
                targetpos.y = ballpos.y - 0.25 * goal_vector_y / goal_vector_norm
            else:
                targetpos.x = ballpos.x
                targetpos.y = ballpos.y
            return_vector = navigation(mypos,targetpos,myangle)
            move_node.change_speed(-return_vector.x,-return_vector.y, 0.5*(angle-myangle))
        elif math.hypot(ballpos.x-mypos.x,ballpos.y-mypos.y) > 0.45:
            status="approach"
            ballpos=ballpose.pose.position
            mypos = mypose.pose.position
            myangle = mypose.pose.orientation
            myangle =math.atan2(2*(myangle.z*myangle.w+myangle.x*myangle.y),1-2*(myangle.y*myangle.y+myangle.z*myangle.z))
            angle = math.atan2(goalpos.y-ballpos.y,goalpos.x-ballpos.x) + math.pi
            print("myangle: ", myangle, "angle: ", angle)
            print(f"mypos:({mypos.x}, {mypos.y})")
            print(f"ballpos:({ballpos.x}, {ballpos.y})")
            print("distance: ", math.hypot(ballpos.x-mypos.x,ballpos.y-mypos.y))
            vector = Point()
            vector.x = ballpos.x - mypos.x
            vector.y = ballpos.y - mypos.y
            myvector1 = Point()
            myvector1.x = math.cos(myangle)
            myvector1.y = math.sin(myangle)
            myvector2 = Point()
            myvector2.x = math.cos(myangle+math.pi/2)
            myvector2.y = math.sin(myangle+math.pi/2)
            # Transform vector to the coordinate system with myvector1 and myvector2 as basis
            new_vector = Point()
            new_vector.x =vector.x * myvector1.x + vector.y * myvector1.y
            new_vector.y = vector.x * myvector2.x + vector.y * myvector2.y
            
            move_node.change_speed(-0.5*new_vector.x,-0.5*new_vector.y,angle-myangle)
            if math.hypot(ballpos.x-mypos.x,ballpos.y-mypos.y)<1.0:
                move_node.change_speed(-new_vector.x,-new_vector.y,angle-myangle)
            if abs(mypos.x)>2.3 or abs(mypos.y)>4:
                print("out of range")
                return_vector = navigation(mypos,origin,myangle)
                move_node.change_speed(-return_vector.x,-return_vector.y, 0.0)
                rclpy.spin_once(move_node)
            # move_node.change_speed(0.0, 0.0, 1.0)
            # move_node.change_speed(-1.0, 0.0, 0.0)
            # for i in range(2):
        else:
            status="kick"
            ballpos=ballpose.pose.position
            mypos = mypose.pose.position
            myangle = mypose.pose.orientation
            myangle =math.atan2(2*(myangle.z*myangle.w+myangle.x*myangle.y),1-2*(myangle.y*myangle.y+myangle.z*myangle.z))
            print("myangle: ", myangle, "angle: ", angle)
            print(f"mypos:({mypos.x}, {mypos.y})")
            print(f"ballpos:({ballpos.x}, {ballpos.y})")
            print(f"goalpos:({goalpos.x}, {goalpos.y})")
            print("distance: ", math.hypot(ballpos.x-mypos.x,ballpos.y-mypos.y))
            if abs(ballpos.y-goalpos.y) < 1 and goalpos.x-0.75 <= ballpos.x <= goalpos.x+0.75:
                angle=math.pi/2
            else:
                angle = math.atan2(goalpos.y-ballpos.y,goalpos.x-ballpos.x)+ math.pi
            if abs(mypos.x)>2.3 or abs(mypos.y)>4:
                print("out of range")
                return_vector = navigation(mypos,origin,myangle)
            if abs(mypos.y)>3.7:
                move_node.change_speed(0.5, 0.0, angle-myangle)
            else:
                move_node.change_speed(1.0, 0.0, angle-myangle)
        rclpy.spin_once(move_node)
        print(f"status: {status}, velocity: ({move_node.speed_x}, {move_node.speed_y}), angular velocity: {move_node.speed_z}")
    move_node.change_motion_id(101)
    rclpy.spin(move_node)
    self_location_subscriber.destroy_node()
    location_subscriber.destroy_node()
    move_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()