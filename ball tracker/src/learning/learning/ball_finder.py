import rclpy
import barrier_move
import move
import camera
from rclpy.node import Node
from sensor_msgs.msg import Range
import cv2
class distance(Node):
    def __init__(self, name):
        super().__init__(name)
        self.distance = float('inf')
        self.dog_name = "mi_desktop_48_b0_2d_7a_fe_b5"
        self.sub = self.create_subscription(Range, f"/{self.dog_name}/ultrasonic_payload", self.sensor_callback, 10)
    def sensor_callback(self, msg: Range):
        self.distance = msg.range
        self.get_logger().info(f"Distance to obstacle: {self.distance:.2f} meters")
    def get_distance(self):
        return self.distance
def judge(img):
    hsv= cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    thresh= cv2.inRange(hsv, (30, 50, 0), (85, 255, 255))
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    try:
        max_contour = max(contours, key=cv2.contourArea)
        print(cv2.contourArea(max_contour))
        return cv2.contourArea(max_contour)>3000
    except:
        return False
def main(args=None):
    rclpy.init(args=args)
    move_node = move.basic_move('basic_move')
    AvoidNode=barrier_move.DogAvoidanceController("AvoidNode")
    camera_node = camera.CameraSubscriber('camera_subscriber')
    distance_node = distance('distance')
    rclpy.spin_once(camera_node)
    img=camera_node.get_image()
    while not judge(img):
        rclpy.spin_once(camera_node)
        img=camera_node.get_image()
        rclpy.spin_once(AvoidNode)
    s=0
    while s<20000:
        rclpy.spin_once(camera_node)
        rclpy.spin_once(distance_node)
        image = camera_node.get_image()
        # if image is not None:
        #     # Display the image using OpenCV
        #     cv2.imwrite('latest_image.jpg', image)
        #     cv2.waitKey(0)
        # else:
        #     print("No image received yet.")
        hsv= cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        thresh= cv2.inRange(hsv, (30, 50, 0), (85, 255, 255))
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        max_contour = max(contours, key=cv2.contourArea)
        s=cv2.contourArea(max_contour)
        (x,y),r= cv2.minEnclosingCircle(max_contour)
        center = (int(x), int(y))
        # Get image dimensions
        height, width = image.shape[:2]
        image_center = width // 2
        speed_x,speed_y,speed_z=0.3,0.0,(image_center - center[0])/image_center
        print(center[0])
        move_node.change_speed(speed_x, speed_y, speed_z)
        for i in range(1):
            rclpy.spin_once(move_node)
    move_node.change_motion_id(101)
    rclpy.spin_once(move_node)
    move_node.destroy_node()
    camera_node.destroy_node()
    distance_node.destroy_node()
    AvoidNode.destroy_node()
    rclpy.shutdown()
if __name__=="__main__":
    main()