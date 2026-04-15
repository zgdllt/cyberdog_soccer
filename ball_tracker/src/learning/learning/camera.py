import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import threading
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_rgb', self.listener_callback, 10)
        self.cv_image = None

    def listener_callback(self, msg: Image):
        # Convert the ROS Image message to a CV2 image
        self.cv_image = self.convert_ros_to_cv(msg)
        cv2.imwrite('latest_image.jpg',self.cv_image)

    def convert_ros_to_cv(self, ros_image):
        # Convert the ROS Image message to a CV2 image
        return self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
    def get_image(self):
        # Return the latest image
        return self.cv_image
def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber('camera_subscriber')
    # Spin ROS node in a separate thread
    rclpy.spin_once(camera_subscriber)
    # Get the latest image
    image = camera_subscriber.get_image()
    if image is not None:
        # Display the image using OpenCV
        cv2.imwrite('latest_image.jpg', image)
        cv2.waitKey(0)
    else:
        print("No image received yet.")
    camera_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()