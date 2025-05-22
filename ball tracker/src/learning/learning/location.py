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
    other_location_subscriber = LocationSubscriber('other_location_subscriber','go91')
    self_location_subscriber = LocationSubscriber('self_location_subscriber','betago2')
    location_subscriber = LocationSubscriber('location_subscriber','greenball')
    goal_subscriber1= LocationSubscriber('goal_subscriber1','goal1')
    goal_subscriber2= LocationSubscriber('goal_subscriber2','goal2')
    def refresh():
    # Refresh the node to get the latest data
        rclpy.spin_once(self_location_subscriber)
        rclpy.spin_once(location_subscriber)
        rclpy.spin_once(goal_subscriber1)
        rclpy.spin_once(goal_subscriber2)
        rclpy.spin_once(other_location_subscriber)
        pose = location_subscriber.get_pose()
        mypose = self_location_subscriber.get_pose()
        goal1pose = goal_subscriber1.get_pose()
        goal2pose = goal_subscriber2.get_pose()
        otpose = other_location_subscriber.get_pose()
        return pose, mypose, goal1pose, goal2pose, otpose
    ballpose = None
    mypose = None
    goal1pose = None
    goal2pose = None
    otpose = None
    print("等待接收位置消息...")
    while ballpose is None or mypose is None or otpose is None:
        ballpose, mypose, goal1pose, goal2pose,otpose = refresh()
    # Get the latest pose
    ballpos=ballpose.pose.position
    mypos = mypose.pose.position
    otpos = otpose.pose.position
    print("Received pose message")
    # Calculate the midpoint between goal1 and goal2
    goalpos = Point()
    if goal1pose is not None and goal2pose is not None:
        goalpos.x = (goal1pose.pose.position.x + goal2pose.pose.position.x) / 2
        goalpos.y = (goal1pose.pose.position.y + goal2pose.pose.position.y) / 2
        goalpos.z = (goal1pose.pose.position.z + goal2pose.pose.position.z) / 2
    else:
        # Default to one of the goals if the other is not available
        valid_pose = goal1pose if goal1pose is not None else goal2pose
        if valid_pose is not None:
            goalpos = valid_pose.pose.position
        else:
            print("No goal positions available")
    origin= Point()
    origin.x = 0.0
    origin.y = 0.0
    origin.z = 0.0
    print(f"ballpos:({ballpos.x}, {ballpos.y})")
    print(f"goalpos:({goalpos.x}, {goalpos.y})")
    while abs(ballpos.y)<abs(goalpos.y):
        move_node.change_motion_id(305)
        while ballpos.y>mypos.y:
            ballpose, mypose, goal1pose, goal2pose,otpose = refresh()
            goalpos = Point()
            if goal1pose is not None and goal2pose is not None:
                goalpos.x = (goal1pose.pose.position.x + goal2pose.pose.position.x) / 2
                goalpos.y = (goal1pose.pose.position.y + goal2pose.pose.position.y) / 2
                goalpos.z = (goal1pose.pose.position.z + goal2pose.pose.position.z) / 2
            else:
                # Default to one of the goals if the other is not available
                valid_pose = goal1pose if goal1pose is not None else goal2pose
                if valid_pose is not None:
                    goalpos = valid_pose.pose.position
                else:
                    print("No goal positions available")
            ballpos=ballpose.pose.position
            mypos = mypose.pose.position
            otpose = otpose.pose.position
            myangle = mypose.pose.orientation
            myangle =math.atan2(2*(myangle.z*myangle.w+myangle.x*myangle.y),1-2*(myangle.y*myangle.y+myangle.z*myangle.z))
            angle = math.pi/2
            # print("myangle: ", myangle, "angle: ", angle)
            # print(f"mypos:({mypos.x}, {mypos.y})")
            # print(f"ballpos:({ballpos.x}, {ballpos.y})")
            # print("distance: ", abs(ballpos.y-mypos.y))
            if abs((ballpos.x-mypos.x)/(ballpos.y-mypos.y)) < 0.1 or (abs(otpos.y)<abs(mypos.y) and abs((otpos.x-mypos.x)/(otpos.y-mypos.y))<0.1):
                move_node.change_speed(-1.0,1.0,angle-myangle)
            else:
                move_node.change_speed(-1.0,0.0,angle-myangle)
            if abs(mypos.x)>2.5 or abs(mypos.y)>4:
                print("out of range")
                return_vector = navigation(mypos,origin,myangle)
                move_node.change_speed(-return_vector.x,-return_vector.y, 0.0)
                rclpy.spin_once(move_node)
                break
            rclpy.spin_once(move_node)
        while math.hypot(ballpos.x-mypos.x,ballpos.y-mypos.y) > 0.5:
            ballpose, mypose, goal1pose, goal2pose,otpose = refresh()
            ballpos=ballpose.pose.position
            mypos = mypose.pose.position
            otpos = otpose.pose.position
            goalpos = Point()
            if goal1pose is not None and goal2pose is not None:
                goalpos.x = (goal1pose.pose.position.x + goal2pose.pose.position.x) / 2
                goalpos.y = (goal1pose.pose.position.y + goal2pose.pose.position.y) / 2
                goalpos.z = (goal1pose.pose.position.z + goal2pose.pose.position.z) / 2
            else:
                # Default to one of the goals if the other is not available
                valid_pose = goal1pose if goal1pose is not None else goal2pose
                if valid_pose is not None:
                    goalpos = valid_pose.pose.position
                else:
                    print("No goal positions available")
            myangle = mypose.pose.orientation
            myangle =math.atan2(2*(myangle.z*myangle.w+myangle.x*myangle.y),1-2*(myangle.y*myangle.y+myangle.z*myangle.z))
            angle = math.atan2(goalpos.y-ballpos.y,goalpos.x-ballpos.x)+ math.pi
            print("myangle: ", myangle, "angle: ", angle)
            print(f"mypos:({mypos.x}, {mypos.y})")
            print(f"ballpos:({ballpos.x}, {ballpos.y})")
            print(f"otpos:({otpos.x}, {otpos.y})")
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
            new_vector.x = vector.x * myvector1.x + vector.y * myvector1.y + 0.2
            new_vector.y = vector.x * myvector2.x + vector.y * myvector2.y
            ot_vector = Point()
            ot_vector.x = otpos.x - mypos.x
            ot_vector.y = otpos.y - mypos.y
            cos=(ot_vector.x * vector.x + ot_vector.y * vector.y)/(math.hypot(ot_vector.x,ot_vector.y)*math.hypot(vector.x,vector.y))
            print("cos: ", cos)
            if cos>0.8 and math.hypot(ot_vector.x,ot_vector.y)<math.hypot(vector.x,vector.y):
                print("otpos is in front of me")
                move_node.change_speed(-0.5*new_vector.x,1.0,angle-myangle)
            else:
                move_node.change_speed(-0.5*new_vector.x,-0.5*new_vector.y,angle-myangle)
            if abs(mypos.x)>2.5 or abs(mypos.y)>4:
                print("out of range")
                return_vector = navigation(mypos,origin,myangle)
                move_node.change_speed(-return_vector.x,-return_vector.y, 0.0)
                rclpy.spin_once(move_node)
                break
            # move_node.change_speed(0.0, 0.0, 1.0)
            # move_node.change_speed(-1.0, 0.0, 0.0)
            # for i in range(2):
            rclpy.spin_once(move_node)
        move_node.change_motion_id(305)
        while math.hypot(ballpos.x-mypos.x,ballpos.y-mypos.y) <= 0.5:
            ballpose, mypose, goal1pose, goal2pose,otpose = refresh()
            ballpos=ballpose.pose.position
            mypos = mypose.pose.position
            otpos = otpose.pose.position
            goalpos = Point()
            if goal1pose is not None and goal2pose is not None:
                goalpos.x = (goal1pose.pose.position.x + goal2pose.pose.position.x) / 2
                goalpos.y = (goal1pose.pose.position.y + goal2pose.pose.position.y) / 2
                goalpos.z = (goal1pose.pose.position.z + goal2pose.pose.position.z) / 2
            else:
                # Default to one of the goals if the other is not available
                valid_pose = goal1pose if goal1pose is not None else goal2pose
                if valid_pose is not None:
                    goalpos = valid_pose.pose.position
                else:
                    print("No goal positions available")
            myangle = mypose.pose.orientation
            myangle =math.atan2(2*(myangle.z*myangle.w+myangle.x*myangle.y),1-2*(myangle.y*myangle.y+myangle.z*myangle.z))
            if abs(ballpos.y-goalpos.y) < 1 and goalpos.x-1 <= ballpos.x <= goalpos.x+1:
                angle=math.pi/2
            else:
                angle = math.atan2(goalpos.y-ballpos.y,goalpos.x-ballpos.x)+ math.pi
            if abs(mypos.x)>2.5 or abs(mypos.y)>4:
                print("out of range")
                return_vector = navigation(mypos,origin,myangle)
                move_node.change_speed(-return_vector.x,-return_vector.y, 0.0)
                rclpy.spin_once(move_node)
                break
            move_node.change_speed(1.0, 0.0, angle-myangle)
            rclpy.spin_once(move_node)

    location_subscriber.destroy_node()
    self_location_subscriber.destroy_node()
    move_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()